//
// Copyright 2014-2015 Per Vices Corporation
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

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include "apply_corrections.hpp"
#include <uhd/utils/static.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/images.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/functional/hash.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/transport/udp_constants.hpp>
#include <uhd/transport/nirio_zero_copy.hpp>
#include <uhd/transport/nirio/niusrprio_session.h>
#include <uhd/utils/platform.hpp>
#include <string>
#include <vector>
#include <uhd/types/ranges.hpp>
#include "crimson_tng_impl.hpp"

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

static void destroy_other_processes_using_crimson();

// This is a lock to prevent multiple threads from requesting commands from
// the device at the same time. This is important in GNURadio, as they spawn
// a new thread per block. If not protected, UDP commands would time out.
//boost::mutex tng_udp_mutex;

/***********************************************************************
 * Helper Functions
 **********************************************************************/
// seperates the input data into the vector tokens based on delim
void tng_csv_parse(std::vector<std::string> &tokens, char* data, const char delim) {
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

// base wrapper that calls the simple UDP interface to get messages to and from Crimson
std::string crimson_tng_impl::get_string(std::string req) {
	this->_udp_mutex.lock();

	// format the string and poke (write)
    	_iface -> poke_str("get," + req);

	// peek (read) back the data
	std::string ret = _iface -> peek_str();

	this->_udp_mutex.unlock();
	if (ret == "TIMEOUT") 	throw uhd::runtime_error("crimson_tng_impl::get_string - UDP resp. timed out: " + req);
	else 			return ret;
}
void crimson_tng_impl::set_string(const std::string pre, std::string data) {
	this->_udp_mutex.lock();

	// format the string and poke (write)
	_iface -> poke_str("set," + pre + "," + data);

	// peek (read) anyways for error check, since Crimson will reply back
	std::string ret = _iface -> peek_str();

	this->_udp_mutex.unlock();
	if (ret == "TIMEOUT" || ret == "ERROR")
		throw uhd::runtime_error("crimson_tng_impl::set_string - UDP resp. timed out: set: " + pre + " = " + data);
	else
		return;
}

// wrapper for type <double> through the ASCII Crimson interface
double crimson_tng_impl::get_double(std::string req) {
	try { return boost::lexical_cast<double>( get_string(req) );
	} catch (...) { return 0; }
}
void crimson_tng_impl::set_double(const std::string pre, double data){
	try { set_string(pre, boost::lexical_cast<std::string>(data));
	} catch (...) { }
}

// wrapper for type <bool> through the ASCII Crimson interface
bool crimson_tng_impl::get_bool(std::string req) {
	try { return boost::lexical_cast<bool>( get_string(req) );
	} catch (...) { return 0; }
}
void crimson_tng_impl::set_bool(const std::string pre, bool data){
	try { set_string(pre, boost::lexical_cast<std::string>(data));
	} catch (...) { }
}

// wrapper for type <int> through the ASCII Crimson interface
int crimson_tng_impl::get_int(std::string req) {
	try { return boost::lexical_cast<int>( get_string(req) );
	} catch (...) { return 0; }
}
void crimson_tng_impl::set_int(const std::string pre, int data){
	try { set_string(pre, boost::lexical_cast<std::string>(data));
	} catch (...) { }
}

// wrapper for type <mboard_eeprom_t> through the ASCII Crimson interface
mboard_eeprom_t crimson_tng_impl::get_mboard_eeprom(std::string req) {
	mboard_eeprom_t temp;
	temp["name"]     = get_string("fpga/about/name");
	temp["vendor"]   = "Per Vices";
	temp["serial"]   = get_string("fpga/about/serial");
	return temp;
}
void crimson_tng_impl::set_mboard_eeprom(const std::string pre, mboard_eeprom_t data) {
	// no eeprom settings on Crimson
	return;
}

// wrapper for type <dboard_eeprom_t> through the ASCII Crimson interface
dboard_eeprom_t crimson_tng_impl::get_dboard_eeprom(std::string req) {
	dboard_eeprom_t temp;
	//temp.id       = dboard_id_t( boost::lexical_cast<boost::uint16_t>(get_string("product,get,serial")) );
	temp.serial   = "";//get_string("product,get,serial");
	//temp.revision = get_string("product,get,hw_version");
	return temp;
}
void crimson_tng_impl::set_dboard_eeprom(const std::string pre, dboard_eeprom_t data) {
	// no eeprom settings on Crimson
	return;
}

// wrapper for type <sensor_value_t> through the ASCII Crimson interface
sensor_value_t crimson_tng_impl::get_sensor_value(std::string req) {
	// no sensors on Crimson
	return sensor_value_t("NA", "0", "NA");
}
void crimson_tng_impl::set_sensor_value(const std::string pre, sensor_value_t data) {
	// no sensors on Crimson
	return;
}

// wrapper for type <meta_range_t> through the ASCII Crimson interface
meta_range_t crimson_tng_impl::get_meta_range(std::string req) {
	throw uhd::not_implemented_error("set_meta_range not implemented, Crimson does not support range settings");
	meta_range_t temp;
	return temp;
}
void crimson_tng_impl::set_meta_range(const std::string pre, meta_range_t data) {
	throw uhd::not_implemented_error("set_meta_range not implemented, Crimson does not support range settings");
	return;
}

// wrapper for type <complex<double>> through the ASCII Crimson interface
std::complex<double>  crimson_tng_impl::get_complex_double(std::string req) {
	std::complex<double> temp;
	return temp;
}
void crimson_tng_impl::set_complex_double(const std::string pre, std::complex<double> data) {
	return;
}

// wrapper for type <stream_cmd_t> through the ASCII Crimson interface
stream_cmd_t crimson_tng_impl::get_stream_cmd(std::string req) {
	stream_cmd_t::stream_mode_t mode = stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
	stream_cmd_t temp = stream_cmd_t(mode);
	return temp;
}
void crimson_tng_impl::set_stream_cmd(const std::string pre, stream_cmd_t data) {
	return;
}

// wrapper for type <time_spec_t> through the ASCII Crimson interface
// we should get back time in the form "12345.6789" from Crimson, where it is seconds elapsed relative to Crimson bootup.
time_spec_t crimson_tng_impl::get_time_spec(std::string req) {
	double fracpart, intpart;
	fracpart = modf(get_double(req), &intpart);
	time_spec_t temp = time_spec_t((time_t)intpart, fracpart);
	return temp;
}
void crimson_tng_impl::set_time_spec(const std::string pre, time_spec_t data) {
	set_double(pre, (double)data.get_full_secs() + data.get_frac_secs());
	return;
}

/***********************************************************************
 * Discovery over the udp transport
 **********************************************************************/
// This find function will be called if a hint is passed onto the find function
static device_addrs_t crimson_tng_find_with_addr(const device_addr_t &hint)
{
    // temporarily make a UDP device only to look for devices
    // loop for all the available ports, if none are available, that means all 8 are open already
    udp_simple::sptr comm = udp_simple::make_broadcast(
        hint["addr"], BOOST_STRINGIZE(CRIMSON_TNG_FW_COMMS_UDP_PORT));

    //send request for echo
    comm->send(asio::buffer("1,get,fpga/about/name", sizeof("1,get,fpga/about/name")));

    //loop for replies from the broadcast until it times out
    device_addrs_t addrs;
    while (true)
    {
        char buff[CRIMSON_TNG_FW_COMMS_MTU] = {};
        const size_t nbytes = comm->recv(asio::buffer(buff), 0.050);
        if (nbytes == 0) break;

        // parse the return buffer and store it in a vector
        std::vector<std::string> tokens;
        tng_csv_parse(tokens, buff, ',');
        if (tokens.size() < 3) break;
        if (tokens[1].c_str()[0] == CMD_ERROR) break;
        if (tokens[2] != "crimson_tng") break;

        device_addr_t new_addr;
        new_addr["type"]    = tokens[2];
        new_addr["addr"]    = comm->get_recv_addr();
        new_addr["name"]    = "";
        new_addr["serial"]  = "001"; // tokens[2];

        //filter the discovered device below by matching optional keys
        if (
            (not hint.has_key("name")    or hint["name"]    == new_addr["name"])    and
            (not hint.has_key("serial")  or hint["serial"]  == new_addr["serial"])  and
            (not hint.has_key("product") or hint["product"] == new_addr["product"])
        ){
            addrs.push_back(new_addr);
        }
    }

    return addrs;
}

// This is the core find function that will be called when uhd:device find() is called because this is registered
static device_addrs_t crimson_tng_find(const device_addr_t &hint_)
{
    //handle the multi-device discovery
    device_addrs_t hints = separate_device_addr(hint_);
    if (hints.size() > 1)
    {
        device_addrs_t found_devices;
        std::string error_msg;
        BOOST_FOREACH(const device_addr_t &hint_i, hints)
        {
            device_addrs_t found_devices_i = crimson_tng_find(hint_i);
            if (found_devices_i.size() != 1) error_msg += str(boost::format(
                "Could not resolve device hint \"%s\" to a single device."
            ) % hint_i.to_string());
            else found_devices.push_back(found_devices_i[0]);
        }
        if (found_devices.empty()) return device_addrs_t();
        if (not error_msg.empty()) throw uhd::value_error(error_msg);

        return device_addrs_t(1, combine_device_addrs(found_devices));
    }

    //initialize the hint for a single device case
    UHD_ASSERT_THROW(hints.size() <= 1);
    hints.resize(1); //in case it was empty
    device_addr_t hint = hints[0];
    device_addrs_t addrs;
    if (hint.has_key("type") and hint["type"] != "crimson_tng") return addrs;


    //use the address given
    if (hint.has_key("addr"))
    {
        device_addrs_t reply_addrs;
        try
        {
            reply_addrs = crimson_tng_find_with_addr(hint);
        }
        catch(const std::exception &ex)
        {
            UHD_MSG(error) << "CRIMSON_TNG Network discovery error " << ex.what() << std::endl;
        }
        catch(...)
        {
            UHD_MSG(error) << "CRIMSON_TNG Network discovery unknown error " << std::endl;
        }
        BOOST_FOREACH(const device_addr_t &reply_addr, reply_addrs)
        {
            device_addrs_t new_addrs = crimson_tng_find_with_addr(reply_addr);
            addrs.insert(addrs.begin(), new_addrs.begin(), new_addrs.end());
        }
        return addrs;
    }

    if (!hint.has_key("resource"))
    {
        //otherwise, no address was specified, send a broadcast on each interface
        BOOST_FOREACH(const if_addrs_t &if_addrs, get_if_addrs())
        {
            //avoid the loopback device
            if (if_addrs.inet == asio::ip::address_v4::loopback().to_string()) continue;

            //create a new hint with this broadcast address
            device_addr_t new_hint = hint;
            new_hint["addr"] = if_addrs.bcast;

            //call discover with the new hint and append results
            device_addrs_t new_addrs = crimson_tng_find(new_hint);
            addrs.insert(addrs.begin(), new_addrs.begin(), new_addrs.end());
        }
    }

    return addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
// Returns a pointer to the Crimson device, casted to the UHD base class
static device::sptr crimson_tng_make(const device_addr_t &device_addr)
{
    return device::sptr(new crimson_tng_impl(device_addr));
}

// This is the core function that registers itself with uhd::device base class. The base device class
// will have a reference to all the registered devices and upon device find/make it will loop through
// all the registered devices' find and make functions.
UHD_STATIC_BLOCK(register_crimson_tng_device)
{
    device::register_device(&crimson_tng_find, &crimson_tng_make, device::CRIMSON_TNG);
}

/***********************************************************************
 * Structors
 **********************************************************************/
// Macro to create the tree, all properties created with this are R/W properties
#define TREE_CREATE_RW(PATH, PROP, TYPE, HANDLER)						\
	do { _tree->create<TYPE> (PATH)								\
    		.set( get_ ## HANDLER (PROP))							\
		.subscribe(boost::bind(&crimson_tng_impl::set_ ## HANDLER, this, (PROP), _1))	\
		.publish  (boost::bind(&crimson_tng_impl::get_ ## HANDLER, this, (PROP)    ));	\
	} while(0)

// Macro to create the tree, all properties created with this are RO properties
#define TREE_CREATE_RO(PATH, PROP, TYPE, HANDLER)						\
	do { _tree->create<TYPE> (PATH)								\
    		.set( get_ ## HANDLER (PROP))							\
		.publish  (boost::bind(&crimson_tng_impl::get_ ## HANDLER, this, (PROP)    ));	\
	} while(0)

// Macro to create the tree, all properties created with this are static
#define TREE_CREATE_ST(PATH, TYPE, VAL) 	( _tree->create<TYPE>(PATH).set(VAL) )

crimson_tng_impl::crimson_tng_impl(const device_addr_t &dev_addr)
{
    UHD_MSG(status) << "Opening a Crimson TNG device..." << std::endl;
    _type = device::CRIMSON_TNG;
    _addr = dev_addr;

    // Makes the UDP comm connection
    _iface = crimson_tng_iface::make( udp_simple::make_connected(
    			dev_addr["addr"], BOOST_STRINGIZE(CRIMSON_TNG_FW_COMMS_UDP_PORT)) );

    // TODO make transports for each RX/TX chain
    // TODO check if locked already
    // TODO lock the Crimson device to this process, this will prevent the Crimson device being used by another program

    // Property paths
    const fs_path mb_path   = "/mboards/0";
    const fs_path time_path = mb_path / "time";
    const fs_path tx_path   = mb_path / "tx";
    const fs_path rx_path   = mb_path / "rx";

    // Create the file tree of properties.
    // Crimson only has support for one mother board, and the RF chains will show up individually as daughter boards.
    // All the initial settings are read from the current status of the board.
    _tree = uhd::property_tree::make();

    static const std::vector<std::string> time_sources = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);

    static const std::vector<double> external_freq_options = boost::assign::list_of(10e6);
    _tree->create<std::vector<double> >(mb_path / "clock_source" / "external" / "freq" / "options");
    static const std::vector<std::string> clock_source_options = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_source_options);

    TREE_CREATE_ST("/name", std::string, "Crimson_TNG Device");

    TREE_CREATE_ST(mb_path / "vendor", std::string, "Per Vices");
    TREE_CREATE_ST(mb_path / "name",   std::string, "FPGA Board");
    TREE_CREATE_RW(mb_path / "id",         "fpga/about/id",     std::string, string);
    TREE_CREATE_RW(mb_path / "serial",     "fpga/about/serial", std::string, string);
    TREE_CREATE_RW(mb_path / "fw_version", "fpga/about/fw_ver", std::string, string);
    TREE_CREATE_RW(mb_path / "hw_version", "fpga/about/hw_ver", std::string, string);
    TREE_CREATE_RW(mb_path / "sw_version", "fpga/about/sw_ver", std::string, string);
    TREE_CREATE_RW(mb_path / "blink", "fpga/board/led", int, int);
    TREE_CREATE_RW(mb_path / "temp", "fpga/board/temp", std::string, string);

    TREE_CREATE_RW(mb_path / "gps_time", "fpga/board/gps_time", int, int);
    TREE_CREATE_RW(mb_path / "gps_frac_time", "fpga/board/gps_frac_time", int, int);
    TREE_CREATE_RW(mb_path / "gps_sync_time", "fpga/board/gps_sync_time", int, int);

    TREE_CREATE_RW(mb_path / "fpga/board/flow_control/sfpa_port", "fpga/board/flow_control/sfpa_port", int, int);
    TREE_CREATE_RW(mb_path / "fpga/board/flow_control/sfpb_port", "fpga/board/flow_control/sfpb_port", int, int);

    TREE_CREATE_ST(time_path / "name", std::string, "Time Board");
    TREE_CREATE_RW(time_path / "id",         "time/about/id",     std::string, string);
    TREE_CREATE_RW(time_path / "serial",     "time/about/serial", std::string, string);
    TREE_CREATE_RW(time_path / "fw_version", "time/about/fw_ver", std::string, string);
    TREE_CREATE_RW(time_path / "sw_version", "time/about/sw_ver", std::string, string);

    TREE_CREATE_ST(rx_path / "name",   std::string, "RX Board");
    TREE_CREATE_ST(rx_path / "spec",   std::string, "4 RX RF chains, 322MHz BW and DC-6GHz each");
    TREE_CREATE_RW(rx_path / "id",         "rx_a/about/id",     std::string, string);
    TREE_CREATE_RW(rx_path / "serial",     "rx_a/about/serial", std::string, string);
    TREE_CREATE_RW(rx_path / "fw_version", "rx_a/about/fw_ver", std::string, string);
    TREE_CREATE_RW(rx_path / "sw_version", "rx_a/about/sw_ver", std::string, string);

    TREE_CREATE_ST(tx_path / "name", std::string, "TX Board");
    TREE_CREATE_ST(tx_path / "spec", std::string, "4 TX RF chains, 322MHz BW and DC-6GHz each");
    TREE_CREATE_RW(tx_path / "id",         "tx_a/about/id",     std::string, string);
    TREE_CREATE_RW(tx_path / "serial",     "tx_a/about/serial", std::string, string);
    TREE_CREATE_RW(tx_path / "fw_version", "tx_a/about/fw_ver", std::string, string);
    TREE_CREATE_RW(tx_path / "sw_version", "tx_a/about/sw_ver", std::string, string);

    // Link max rate refers to ethernet link rate
    TREE_CREATE_RW(mb_path / "link_max_rate", "fpga/link/rate", double, double);

    // SFP settings
    TREE_CREATE_RW(mb_path / "link" / "sfpa" / "ip_addr",  "fpga/link/sfpa/ip_addr", std::string, string);
    TREE_CREATE_RW(mb_path / "link" / "sfpa" / "pay_len", "fpga/link/sfpa/pay_len", int, int);
    TREE_CREATE_RW(mb_path / "link" / "sfpb" / "ip_addr",     "fpga/link/sfpb/ip_addr", std::string, string);
    TREE_CREATE_RW(mb_path / "link" / "sfpb" / "pay_len", "fpga/link/sfpb/pay_len", int, int);

    // This is the master clock rate
    TREE_CREATE_ST(mb_path / "tick_rate", double, CRIMSON_TNG_MASTER_CLOCK_RATE);

    TREE_CREATE_ST(time_path / "cmd", time_spec_t, time_spec_t(0.0));
    TREE_CREATE_RW(time_path / "now", "time/clk/cur_time", time_spec_t, time_spec);
    TREE_CREATE_RW(time_path / "pps", "time/clk/pps", 	   time_spec_t, time_spec);

    TREE_CREATE_ST(mb_path / "eeprom", mboard_eeprom_t, mboard_eeprom_t());

    // This property chooses internal or external clock source
    TREE_CREATE_RW(mb_path / "time_source"  / "value",  	"time/source/ref",  	std::string, string);
    TREE_CREATE_RW(mb_path / "clock_source" / "value",          "time/source/ref",	std::string, string);
    TREE_CREATE_RW(mb_path / "clock_source" / "external",	"time/source/ref",	std::string, string);
    TREE_CREATE_ST(mb_path / "clock_source" / "external" / "value", double, CRIMSON_TNG_EXT_CLK_RATE);
    TREE_CREATE_ST(mb_path / "clock_source" / "output", bool, true);
    TREE_CREATE_ST(mb_path / "time_source"  / "output", bool, true);

    TREE_CREATE_ST(mb_path / "sensors" / "ref_locked", sensor_value_t, sensor_value_t("NA", "0", "NA"));

    // No GPSDO support on Crimson
    // TREE_CREATE_ST(mb_path / "sensors" / "ref_locked", sensor_value_t, sensor_value_t("NA", "0", "NA"));

    // loop for all RF chains
    for (int chain = 0; chain < 4; chain ++) {
		std::string lc_num  = boost::lexical_cast<std::string>((char)(chain + 97));
		std::string num     = boost::lexical_cast<std::string>((char)(chain + 65));
		std::string chan    = "Channel_" + num;

		const fs_path rx_codec_path = mb_path / "rx_codecs" / num;
		const fs_path tx_codec_path = mb_path / "tx_codecs" / num;
		const fs_path rx_fe_path    = mb_path / "dboards" / num / "rx_frontends" / chan;
		const fs_path tx_fe_path    = mb_path / "dboards" / num / "tx_frontends" / chan;
		const fs_path db_path       = mb_path / "dboards" / num;
		const fs_path rx_dsp_path   = mb_path / "rx_dsps" / chan;
		const fs_path tx_dsp_path   = mb_path / "tx_dsps" / chan;
		const fs_path rx_link_path  = mb_path / "rx_link" / chan;
		const fs_path tx_link_path  = mb_path / "tx_link" / chan;

		static const std::vector<std::string> antenna_options = boost::assign::list_of("SMA")("None");
		_tree->create<std::vector<std::string> >(rx_fe_path / "antenna" / "options").set(antenna_options);
		_tree->create<std::vector<std::string> >(tx_fe_path / "antenna" / "options").set(antenna_options);

		static const std::vector<std::string> sensor_options = boost::assign::list_of("lo_locked");
		_tree->create<std::vector<std::string> >(rx_fe_path / "sensors").set(sensor_options);
		_tree->create<std::vector<std::string> >(tx_fe_path / "sensors").set(sensor_options);

		// Actual frequency values
		TREE_CREATE_RW(rx_path / chan / "freq" / "value", "rx_"+lc_num+"/rf/freq/val", double, double);
		TREE_CREATE_RW(tx_path / chan / "freq" / "value", "tx_"+lc_num+"/rf/freq/val", double, double);

		// Power status
		TREE_CREATE_RW(rx_path / chan / "pwr", "rx_"+lc_num+"/pwr", std::string, string);
		TREE_CREATE_RW(tx_path / chan / "pwr", "tx_"+lc_num+"/pwr", std::string, string);

		// Channel Stream Status
		TREE_CREATE_RW(rx_link_path / "stream", "rx_"+lc_num+"/stream", std::string, string);

		// Codecs, phony properties for Crimson
		TREE_CREATE_RW(rx_codec_path / "gains", "rx_"+lc_num+"/dsp/gain", int, int);
		TREE_CREATE_ST(rx_codec_path / "name", std::string, "RX Codec");

		TREE_CREATE_RW(tx_codec_path / "gains", "tx_"+lc_num+"/dsp/gain", int, int);
		TREE_CREATE_ST(tx_codec_path / "name", std::string, "TX Codec");

		// Daughter Boards' Frontend Settings
		TREE_CREATE_ST(rx_fe_path / "name",   std::string, "RX Board");
		TREE_CREATE_ST(tx_fe_path / "name",   std::string, "TX Board");

		TREE_CREATE_ST(rx_fe_path / "gains" / "LMH+PE" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_RF_RX_GAIN_RANGE_START, CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP, CRIMSON_TNG_RF_RX_GAIN_RANGE_STEP));
		TREE_CREATE_ST(tx_fe_path / "gains" / "PE" / "range",	meta_range_t,
			meta_range_t(CRIMSON_TNG_RF_TX_GAIN_RANGE_START, CRIMSON_TNG_RF_TX_GAIN_RANGE_STOP, CRIMSON_TNG_RF_TX_GAIN_RANGE_STEP));

		TREE_CREATE_ST(rx_fe_path / "freq", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));
		TREE_CREATE_ST(tx_fe_path / "freq", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));

		TREE_CREATE_ST(rx_fe_path / "dc_offset" / "enable", bool, false);
		TREE_CREATE_ST(rx_fe_path / "dc_offset" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));
		TREE_CREATE_ST(rx_fe_path / "iq_balance" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));

		TREE_CREATE_ST(tx_fe_path / "dc_offset" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));
		TREE_CREATE_ST(tx_fe_path / "iq_balance" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));

		TREE_CREATE_RW(rx_fe_path / "connection",  "rx_"+lc_num+"/link/iface", std::string, string);
		TREE_CREATE_RW(tx_fe_path / "connection",  "tx_"+lc_num+"/link/iface", std::string, string);

		TREE_CREATE_ST(rx_fe_path / "use_lo_offset", bool, false);
		TREE_CREATE_ST(tx_fe_path / "use_lo_offset", bool, false);
		TREE_CREATE_RW(tx_fe_path / "lo_offset" / "value", "tx_"+lc_num+"/rf/dac/nco", double, double);

		TREE_CREATE_ST(tx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));
		TREE_CREATE_ST(rx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));
		TREE_CREATE_ST(rx_fe_path / "gain" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_RF_RX_GAIN_RANGE_START, CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP, CRIMSON_TNG_RF_RX_GAIN_RANGE_STEP));
		TREE_CREATE_ST(tx_fe_path / "gain" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_RF_TX_GAIN_RANGE_START, CRIMSON_TNG_RF_TX_GAIN_RANGE_STOP, CRIMSON_TNG_RF_TX_GAIN_RANGE_STEP));

		TREE_CREATE_RW(rx_fe_path / "freq"  / "value", "rx_"+lc_num+"/rf/freq/val" , double, double);
		TREE_CREATE_RW(rx_fe_path / "gain"  / "value", "rx_"+lc_num+"/rf/gain/val" , double, double);
		TREE_CREATE_RW(rx_fe_path / "atten" / "value", "rx_"+lc_num+"/rf/atten/val", double, double);
		TREE_CREATE_RW(tx_fe_path / "freq"  / "value", "tx_"+lc_num+"/rf/freq/val" , double, double);
		TREE_CREATE_RW(tx_fe_path / "gain"  / "value", "tx_"+lc_num+"/rf/gain/val" , double, double);

		// RF band
		TREE_CREATE_RW(rx_fe_path / "freq" / "band", "rx_"+lc_num+"/rf/freq/band", int, int);
		TREE_CREATE_RW(tx_fe_path / "freq" / "band", "tx_"+lc_num+"/rf/freq/band", int, int);

		// RF receiver LNA
		TREE_CREATE_RW(rx_fe_path / "freq"  / "lna", "rx_"+lc_num+"/rf/freq/lna" , int, int);

		// these are phony properties for Crimson
		TREE_CREATE_ST(db_path / "rx_eeprom",  dboard_eeprom_t, dboard_eeprom_t());
		TREE_CREATE_ST(db_path / "tx_eeprom",  dboard_eeprom_t, dboard_eeprom_t());
		TREE_CREATE_ST(db_path / "gdb_eeprom", dboard_eeprom_t, dboard_eeprom_t());

		// DSPs
		switch( chain + 'A' ) {
		case 'A':
		case 'B':
			TREE_CREATE_ST(rx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			break;
		case 'C':
		case 'D':
			TREE_CREATE_ST(rx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2.0 , CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP / 2.0, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2.0, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2.0, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP / 2.0, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2.0, CRIMSON_TNG_RATE_RANGE_STEP));
			break;
		}

		TREE_CREATE_RW(rx_dsp_path / "rate" / "value", "rx_"+lc_num+"/dsp/rate",    double, double);
		TREE_CREATE_RW(rx_dsp_path / "freq" / "value", "rx_"+lc_num+"/dsp/nco_adj", double, double);
		TREE_CREATE_RW(rx_dsp_path / "bw" / "value",   "rx_"+lc_num+"/dsp/rate",    double, double);
		//TREE_CREATE_ST(rx_dsp_path / "stream_cmd",     stream_cmd_t, (stream_cmd_t)0);

		TREE_CREATE_RW(tx_dsp_path / "rate" / "value", "tx_"+lc_num+"/dsp/rate",    double, double);
		TREE_CREATE_RW(tx_dsp_path / "bw" / "value",   "tx_"+lc_num+"/dsp/rate",    double, double);

		TREE_CREATE_RW(tx_dsp_path / "freq" / "value", "tx_"+lc_num+"/dsp/nco_adj", double, double);

		TREE_CREATE_RW(rx_dsp_path / "nco", "rx_"+lc_num+"/dsp/nco_adj", double, double);
		TREE_CREATE_RW(tx_dsp_path / "nco", "tx_"+lc_num+"/dsp/nco_adj", double, double);
		TREE_CREATE_RW(tx_fe_path / "nco", "tx_"+lc_num+"/rf/dac/nco", double, double);

		// Link settings
		TREE_CREATE_RW(rx_link_path / "vita_en", "rx_"+lc_num+"/link/vita_en", std::string, string);
		TREE_CREATE_RW(rx_link_path / "ip_dest", "rx_"+lc_num+"/link/ip_dest", std::string, string);
		TREE_CREATE_RW(rx_link_path / "port",    "rx_"+lc_num+"/link/port",    std::string, string);
		TREE_CREATE_RW(rx_link_path / "iface",   "rx_"+lc_num+"/link/iface",   std::string, string);

		TREE_CREATE_RW(tx_link_path / "vita_en", "tx_"+lc_num+"/link/vita_en", std::string, string);
		TREE_CREATE_RW(tx_link_path / "port",    "tx_"+lc_num+"/link/port",    std::string, string);
		TREE_CREATE_RW(tx_link_path / "iface",   "tx_"+lc_num+"/link/iface",   std::string, string);
    }

	const fs_path cm_path  = mb_path / "cm";

	// Common Mode
	TREE_CREATE_RW(cm_path / "chanmask-rx", "cm/chanmask-rx", int, int);
	TREE_CREATE_RW(cm_path / "chanmask-tx", "cm/chanmask-tx", int, int);
	TREE_CREATE_RW(cm_path / "rx/atten/val", "cm/rx/atten/val", double, double);
	TREE_CREATE_RW(cm_path / "rx/gain/val", "cm/rx/gain/val", double, double);
	TREE_CREATE_RW(cm_path / "tx/gain/val", "cm/tx/gain/val", double, double);
	TREE_CREATE_RW(cm_path / "trx/freq/val", "cm/trx/freq/val", double, double);
	TREE_CREATE_RW(cm_path / "trx/nco_adj", "cm/trx/nco_adj", double, double);


	// kb 4001: stop-gap solution until kb #4000 is fixed!!!
	destroy_other_processes_using_crimson();
}

crimson_tng_impl::~crimson_tng_impl(void)
{
    // TODO send commands to mute all radio chains, mute everything
    // unlock the Crimson device to this process
}

// kb 4001: stop-gap solution until kb #4000 is fixed!!!
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>

#include <regex>

static void destroy_other_processes_using_crimson() {

	static const std::regex re( "^crimson-([1-9][0-9]*).lock$" );
	DIR *dp;
	std::string lock_file_name;
	std::smatch match;
	uint64_t pidn;
	std::stringstream ss;
	int fd;

	dp = opendir( "/tmp" );
	if ( NULL == dp ) {
		throw runtime_error(
			(
				boost::format( "opendir: %s (%d)" )
				% std::strerror( errno )
				% errno
			).str()
		);
	}

	for( dirent *de = readdir( dp ); NULL != de; de = readdir( dp ) ) {
		if ( DT_REG != de->d_type ) {
			continue;
		}
		lock_file_name = std::string( de->d_name );
		if ( ! std::regex_search( lock_file_name, match, re ) ) {
			continue;
		}

		ss = std::stringstream( match.str( 1 ) );
		ss >> pidn;

		if ( 0 == kill( pid_t( pidn ), SIGKILL ) ) {
			UHD_MSG( warning ) << "killed hung process " << pidn << std::endl;
		}

		lock_file_name = "/tmp/" + lock_file_name;
		if ( 0 == remove( lock_file_name.c_str() ) ) {
			UHD_MSG( warning ) << "removed stale lockfile " << lock_file_name << std::endl;
		} else {
			UHD_MSG( warning ) << "failed to remove stale lockfile " << lock_file_name << std::endl;
		}
	}
	closedir( dp );

	ss.clear();
	ss << "/tmp/crimson-" << (uint64_t) getpid() << ".lock";

	lock_file_name = ss.str();

	fd = open( lock_file_name.c_str(), O_RDWR | O_CREAT );
	if ( -1 == fd ) {
		UHD_MSG( warning ) << "failed to create lockfile " << lock_file_name << ": " << strerror( errno ) << std::endl;
	} else {
		close( fd );
	}
}
