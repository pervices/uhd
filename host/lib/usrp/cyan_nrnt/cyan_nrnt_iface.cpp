//
// Copyright 2014-2015 Per Vices Corporation
// Copyright 2022, 2024 Per Vices Corporation
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// TODO: consolidate cyan and crimson versions of this

#include <uhd/exception.hpp>
#include <uhd/utils/safe_call.hpp>
#include <inttypes.h>
#include "cyan_nrnt_fw_common.h"
#include "cyan_nrnt_iface.hpp"

// Used for manipulating the Python GIL
#include <Python.h>

using namespace uhd;
using namespace uhd::transport;

static uint32_t seq = 1;

/***********************************************************************
 * Structors
 **********************************************************************/
cyan_nrnt_iface::cyan_nrnt_iface(udp_simple::sptr ctrl_transport):
    _ctrl_transport(ctrl_transport),
    _ctrl_seq_num(0),
    _protocol_compat(0)
{
    memset( _buff, '\0', sizeof( _buff ) );
}

/***********************************************************************
 * Peek and Poke
 **********************************************************************/
// Never call this function by itself, always call through cyan_nrnt_iface::get/set()
// else it will mess up the protocol with the sequencing and will contian no error checks.
void cyan_nrnt_iface::poke_str(std::string data) {
    // populate the command string with sequence number
    data = data.insert(0, (boost::lexical_cast<std::string>(seq++) + ","));
    _ctrl_transport->send( data.c_str(), data.length() );
    return;
}

// Never call this function by itself, always call through cyan_nrnt_iface::get/set(),
// else it will mess up the protocol with the sequencing and will contian no error checks.
// Format: <sequence number>,<error code>,<data>
std::string cyan_nrnt_iface::peek_str( float timeout_s ) {
    uint32_t iseq;
    std::vector<std::string> tokens;
    uint8_t tries = 0;
    uint8_t num_tries = 5;

    do {
        // clears the buffer and receives the message
        memset( _buff, 0, sizeof( _buff ) );
        const size_t nbytes = _ctrl_transport -> recv(_buff, CYAN_NRNT_MAX_MTU, timeout_s );
        if (nbytes == 0) return "TIMEOUT";

        // parses it through tokens: seq, status, [data]
        this -> parse(tokens, _buff, ',');

        // Malformed packet
        if (tokens.size() < 2) {
            return "ERROR";
        // Error while reading property
        } else if(tokens[1].c_str()[0] == CMD_ERROR) {
            return "GET_ERROR";
        }

        // if seq is incorrect, return an error
        sscanf(tokens[0].c_str(), "%" SCNd32, &iseq);

    } while(iseq != seq - 1 && tries++ < num_tries);

    // exits with an error if can't find a matching sequence
    if (tries == num_tries) return "INVLD_SEQ";

    // Return the message (tokens[2])
    // If the error checking passed and tokens is size 2, then the intended data is "" (since parse will not add "" if the packets ends with a ","
    if(tokens.size() < 3) {
        return "";
    } else {
        return tokens[2];
    }
}

std::string cyan_nrnt_iface::peek_str() {
    return peek_str( 8 );
}

// Gets a property on the device
std::string cyan_nrnt_iface::get_string(std::string req) {
    std::string ret;

    // Release Python GIL. Prevents blocking io from making Python hang
    // Does nothing if not using the Python UHD API
    // NOTE: Py_BEGIN_ALLOW_THREADS to Py_END_ALLOW_THREADS is it's own scope
    Py_BEGIN_ALLOW_THREADS

    std::lock_guard<std::mutex> _lock( _iface_lock );

    // Send the get request
    poke_str("get," + req);

    // peek (read) back the data
    ret = peek_str();

    // Obtain Python GIL
    Py_END_ALLOW_THREADS

    if(ret == "GET_ERROR") {
        throw uhd::lookup_error("cyan_nrnt_iface::get_string - Unable to read property on the server: " + req + "\nPlease Verify that the server is up to date");
    }
    else if (ret == "TIMEOUT") {
        throw uhd::runtime_error("cyan_nrnt_iface::get_string - UDP resp. timed out: get: " + req);
    }
    else  if(ret == "ERROR") {
        throw uhd::runtime_error("cyan_nrnt_iface::get_string - UDP unpecified error: " + req);
    }
    else {
        return ret;
    }
}
// Sets a property on the device
void cyan_nrnt_iface::set_string(const std::string pre, std::string data) {
    std::string ret;

    // Release Python GIL. Prevents blocking io from making Python hang
    // Does nothing if not using the Python UHD API
    // NOTE: Py_BEGIN_ALLOW_THREADS to Py_END_ALLOW_THREADS is it's own scope
    Py_BEGIN_ALLOW_THREADS

	std::lock_guard<std::mutex> _lock( _iface_lock );

	// Send the set request
	poke_str("set," + pre + "," + data);

	// peek (read) anyways for error check, since Crimson will reply back
	ret = peek_str();

    // Obtain Python GIL
    Py_END_ALLOW_THREADS

    if(ret == "GET_ERROR") {
        throw uhd::lookup_error("cyan_nrnt_iface::set_string - Unable to read property on the server: " + pre + "\nPlease Verify that the server is up to date");
    }
    else if (ret == "TIMEOUT") {
        throw uhd::runtime_error("cyan_nrnt_iface::set_string - UDP resp. timed out: set: " + pre + " = " + data);
    }
    else  if(ret == "ERROR") {
        throw uhd::runtime_error("cyan_nrnt_iface::set_string - UDP unpecified error: " + pre);
    }
    else {
        return;
    }
}

// wrapper for type <double> through the ASCII Crimson interface
double cyan_nrnt_iface::get_double(std::string req) {
    try { return boost::lexical_cast<double>( get_string(req) );
    } catch(boost::bad_lexical_cast &e) {
        UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "Failed to get double property: " << e.what();
    }
    return 0;
}
void cyan_nrnt_iface::set_double(const std::string pre, double data){
    set_string(pre, boost::lexical_cast<std::string>(data));
}

// wrapper for type <bool> through the ASCII Crimson interface
bool cyan_nrnt_iface::get_bool(std::string req) {
    try { return boost::lexical_cast<bool>( get_string(req) );
    } catch(boost::bad_lexical_cast &e) {
        UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "Failed to get bool property: " << e.what();
    }
    return 0;
}
void cyan_nrnt_iface::set_bool(const std::string pre, bool data){
    set_string(pre, boost::lexical_cast<std::string>(data));
}

// wrapper for type <int> through the ASCII Crimson interface
int cyan_nrnt_iface::get_int(std::string req) {
	try { return boost::lexical_cast<int>( get_string(req) );
    } catch(boost::bad_lexical_cast &e) {
        UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "Failed to get int property: " << e.what();
    }
    return 0;
}
void cyan_nrnt_iface::set_int(const std::string pre, int data){
    set_string(pre, boost::lexical_cast<std::string>(data));
}

// wrapper for type <mboard_eeprom_t> through the ASCII Crimson interface
uhd::usrp::mboard_eeprom_t cyan_nrnt_iface::get_mboard_eeprom(std::string req) {
    (void)req;
    uhd::usrp::mboard_eeprom_t temp;
    temp["name"]     = get_string("fpga/about/name");
    temp["vendor"]   = "Per Vices";
    temp["serial"]   = get_string("fpga/about/serial");
    return temp;
}
void cyan_nrnt_iface::set_mboard_eeprom(const std::string pre, uhd::usrp::mboard_eeprom_t data) {
    (void)pre;
    (void)data;
    // no eeprom settings on Crimson
    return;
}

// wrapper for type <dboard_eeprom_t> through the ASCII Crimson interface
uhd::usrp::dboard_eeprom_t cyan_nrnt_iface::get_dboard_eeprom(std::string req) {
    (void)req;
    uhd::usrp::dboard_eeprom_t temp;
    //temp.id       = dboard_id_t( boost::lexical_cast<boost::uint16_t>(get_string("product,get,serial")) );
    temp.serial   = "";//get_string("product,get,serial");
    //temp.revision = get_string("product,get,hw_version");
    return temp;
}
void cyan_nrnt_iface::set_dboard_eeprom(const std::string pre, uhd::usrp::dboard_eeprom_t data) {
    (void)pre;
    (void)data;
    // no eeprom settings on Crimson
    return;
}

// wrapper for type <sensor_value_t> through the ASCII Crimson interface
sensor_value_t cyan_nrnt_iface::get_sensor_value(std::string req) {
    // Property values are only updated when written to
    // Set sensor to it's current value in order to update it
    try {
        std::string original_value = get_string(req);
        set_string(req, original_value);
    } catch (...) { }


    std::string reply;
    try {
        reply = get_string(req);
    } catch (...) {
        reply = "bad";
    }

    // Shifts reply to lower case
    for(size_t i = 0; i < reply.size(); i++) {
        if(reply[i] >= 'A' && reply[i] <= 'Z') {
            reply[i] = reply[i] - 'A' + 'a';
        }
    }

    // Result good if reply does not contain unlocked or bad
    bool sensor_good = (reply.find("unlocked") == std::string::npos) && (reply.find("bad") == std::string::npos);

    // Determines the sensor name based on the path
    if(req.find("lmk_lockdetect") != std::string::npos) {
        return sensor_value_t( "Reference", sensor_good, "locked", "unlocked" );
    } else if(req.find("rfpll_lock") != std::string::npos) {
        return sensor_value_t( "rfpll", sensor_good, "locked", "unlocked" );
    } else {
        UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "sensor implementation not validated: " << req;
        return sensor_value_t( req, sensor_good, "good", "bad" );
    }
}
void cyan_nrnt_iface::set_sensor_value(const std::string pre, sensor_value_t data) {
    try { set_string(pre, data.to_pp_string());
    } catch (...) { }

    return;
}

// we should get back time in the form "12345.6789" from Crimson, where it is seconds elapsed relative to Crimson bootup or the time set this boot
// NOTE: use cyan_nrnt_impl::get_time_now for anything that requires precision
time_spec_t cyan_nrnt_iface::get_time_spec(std::string req) {
    // Get time via management port
    double fracpart, intpart;
    fracpart = modf(get_double(req), &intpart);
    time_spec_t temp = time_spec_t((time_t)intpart, fracpart);
    return temp;
}

void cyan_nrnt_iface::set_time_spec( const std::string pre, time_spec_t value ) {
    set_double(pre, value.get_real_secs());
}

/***********************************************************************
 * Public make function for cyan_nrnt interface
 **********************************************************************/
cyan_nrnt_iface::sptr cyan_nrnt_iface::make(udp_simple::sptr ctrl_transport){
    return std::make_shared<cyan_nrnt_iface>(ctrl_transport);
}

/***********************************************************************
 * Helper Functions
 **********************************************************************/
void cyan_nrnt_iface::parse(std::vector<std::string> &tokens, char* data, const char delim) {
	int i = 0;
	while (data[i]) {
		std::string token = "";
		while (data[i] && data[i] != delim) {
			token.push_back(data[i]);
			if (data[i+1] == 0 || data[i+1] == delim)
				tokens.push_back(token);
			i++;
		}
		i++;
	}
	return;
}
