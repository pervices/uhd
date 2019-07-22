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

#include <boost/assign.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/endian/buffers.hpp>

#include "crimson_tng_impl.hpp"

#include "uhd/transport/if_addrs.hpp"
#include "uhd/transport/udp_stream_zero_copy.hpp"
#include "uhd/transport/udp_simple.hpp"
#include "uhd/types/stream_cmd.hpp"
#include "uhd/utils/static.hpp"

#include "../../transport/super_recv_packet_handler.hpp"
#include "../../transport/super_send_packet_handler.hpp"

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

#ifndef DEFAULT_NUM_FRAMES
#define DEFAULT_NUM_FRAMES 32
#endif

#if 0
    #ifndef
      #define DEBUG_COUT
    #endif
#endif

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

	std::lock_guard<std::mutex> _lock( _iface_lock );

	// format the string and poke (write)
    _mbc[ "0" ].iface -> poke_str("get," + req);

	// peek (read) back the data
	std::string ret = _mbc[ "0" ].iface -> peek_str();

	if (ret == "TIMEOUT") 	throw uhd::runtime_error("crimson_tng_impl::get_string - UDP resp. timed out: " + req);
	else 			return ret;
}
void crimson_tng_impl::set_string(const std::string pre, std::string data) {

	std::lock_guard<std::mutex> _lock( _iface_lock );

	// format the string and poke (write)
	_mbc[ "0" ].iface -> poke_str("set," + pre + "," + data);

	// peek (read) anyways for error check, since Crimson will reply back
	std::string ret = _mbc[ "0" ].iface -> peek_str();

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
	(void)req;
	mboard_eeprom_t temp;
	temp["name"]     = get_string("fpga/about/name");
	temp["vendor"]   = "Per Vices";
	temp["serial"]   = get_string("fpga/about/serial");
	return temp;
}
void crimson_tng_impl::set_mboard_eeprom(const std::string pre, mboard_eeprom_t data) {
	(void)pre;
	(void)data;
	// no eeprom settings on Crimson
	return;
}

// wrapper for type <dboard_eeprom_t> through the ASCII Crimson interface
dboard_eeprom_t crimson_tng_impl::get_dboard_eeprom(std::string req) {
	(void)req;
	dboard_eeprom_t temp;
	//temp.id       = dboard_id_t( boost::lexical_cast<boost::uint16_t>(get_string("product,get,serial")) );
	temp.serial   = "";//get_string("product,get,serial");
	//temp.revision = get_string("product,get,hw_version");
	return temp;
}
void crimson_tng_impl::set_dboard_eeprom(const std::string pre, dboard_eeprom_t data) {
	(void)pre;
	(void)data;
	// no eeprom settings on Crimson
	return;
}

// wrapper for type <sensor_value_t> through the ASCII Crimson interface
sensor_value_t crimson_tng_impl::get_sensor_value(std::string req) {
	(void)req;
	// no sensors on Crimson
	return sensor_value_t("NA", "0", "NA");
}
void crimson_tng_impl::set_sensor_value(const std::string pre, sensor_value_t data) {
	(void)pre;
	(void)data;
	// no sensors on Crimson
	return;
}

// wrapper for type <meta_range_t> through the ASCII Crimson interface
meta_range_t crimson_tng_impl::get_meta_range(std::string req) {
	(void)req;
	throw uhd::not_implemented_error("set_meta_range not implemented, Crimson does not support range settings");
}
void crimson_tng_impl::set_meta_range(const std::string pre, meta_range_t data) {
	(void)pre;
	(void)data;
	throw uhd::not_implemented_error("set_meta_range not implemented, Crimson does not support range settings");
}

// wrapper for type <complex<double>> through the ASCII Crimson interface
std::complex<double>  crimson_tng_impl::get_complex_double(std::string req) {
	(void)req;
	std::complex<double> temp;
	return temp;
}
void crimson_tng_impl::set_complex_double(const std::string pre, std::complex<double> data) {
	(void)pre;
	(void)data;
	return;
}

static size_t pre_to_ch( const std::string & pre ) {
	char x = -1;
	if ( 1 != sscanf( pre.c_str(), "rx_%c/stream", & x) ) {
		throw value_error( "Invalid 'pre' argument '" + pre + "'" );
	}
	size_t ch = x - 'a';

	return ch;
}

stream_cmd_t crimson_tng_impl::get_stream_cmd(std::string req) {
	(void)req;
	// XXX: @CF: 20180214: stream_cmd is basically a write-only property, but we have to return a dummy variable of some kind
	stream_cmd_t::stream_mode_t mode = stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
	stream_cmd_t temp = stream_cmd_t(mode);
	return temp;
}
void crimson_tng_impl::set_stream_cmd( const std::string pre, const stream_cmd_t stream_cmd ) {

	const size_t ch = pre_to_ch( pre );
	const uhd::time_spec_t now = get_time_now();
#ifdef DEBUG_COUT
    std::cout
        << std::fixed << std::setprecision(6)
        << now.get_real_secs()
        << ": "
        << stream_cmd.stream_mode
        << ": "
        << pre
        << ": SETTING STREAM COMMAND: "
        << stream_cmd.num_samps << ": "
        << stream_cmd.stream_now << ": "
        << stream_cmd.time_spec.get_real_secs() << std::endl;
#endif

	uhd::usrp::rx_stream_cmd rx_stream_cmd;

	make_rx_stream_cmd_packet( stream_cmd, now, ch, rx_stream_cmd );
	send_rx_stream_cmd_req( rx_stream_cmd );
}

// wrapper for type <time_spec_t> through the ASCII Crimson interface
// we should get back time in the form "12345.6789" from Crimson, where it is seconds elapsed relative to Crimson bootup.
time_spec_t crimson_tng_impl::get_time_spec(std::string req) {
	if ( false ) {
	} else if ( "time/clk/cur_time" == req ) {
		return get_time_now();
	} else if ( "time/clk/pps" == req ) {
		return uhd::time_spec_t( get_time_now().get_full_secs() );
	} else {
		double fracpart, intpart;
		fracpart = modf(get_double(req), &intpart);
		time_spec_t temp = time_spec_t((time_t)intpart, fracpart);
		return temp;
	}
}
void crimson_tng_impl::set_time_spec( const std::string key, time_spec_t value ) {
	if ( "time/clk/cur_time" == key ) {
		//std::cout << __func__ << "(): " << std::fixed << std::setprecision( 12 ) << value.get_real_secs() << std::endl;
		stop_bm();
	}
	set_double(key, (double)value.get_full_secs() + value.get_frac_secs());
	if ( "time/clk/cur_time" == key ) {
		start_bm();
	}

	if ( "time/clk/cmd" == key ) {
        _command_time = value; // Handles set_command_time() and clear_command_time()
        #ifdef DEBUG_COUT
        std::cout << "updating command time to: " << _command_time.get_real_secs() << std::endl;
        #endif
    }
}

user_reg_t crimson_tng_impl::get_user_reg(std::string req) {

    (void) req;

    // Returns nothing.
    return user_reg_t(0, 0);
}

void crimson_tng_impl::send_gpio_burst_req(const gpio_burst_req& req) {
	_time_diff_iface->send(boost::asio::const_buffer(&req, sizeof(req)));
}

void crimson_tng_impl::set_user_reg(const std::string key, user_reg_t value) {

    (void) key;

    const uint8_t  address = value.first;
    const uint64_t setting = (uint64_t) value.second;

#ifdef PV_TATE
    static uint64_t pins[NUMBER_OF_GPIO_REGS] = {0x0, 0x0};
    static uint64_t mask[NUMBER_OF_GPIO_REGS] = {0x0, 0x0};
#else
    static uint64_t pins[NUMBER_OF_GPIO_REGS] = {0x0};
    static uint64_t mask[NUMBER_OF_GPIO_REGS] = {0x0};
#endif


    // Sanity check to make sure that user is not exceeding legal GPIO range
    uint32_t pin_number = setting;
    int i = 0;
    // Determine what is the biggest GPIO pin number in the setting
    for (i = 0; i < 32; i++) {
        pin_number = pin_number > 1;
        if (pin_number == 1) {
            break;
        }
    }
    pin_number = (pin_number > 0) ? (((address/2)*32)+i) : 0;
    if ( setting != 0 && pin_number > NUMBER_OF_GPIO_SIGNALS-1) {
        char buff [100];
        sprintf(buff, "FATAL: GPIO %d out of bounds (total %d GPIOs)", pin_number, NUMBER_OF_GPIO_SIGNALS);
        std::string error_msg = buff;
        throw runtime_error(error_msg);
    }

    const uint64_t all = 0x00000000FFFFFFFF;
#ifdef PV_TATE
    // Clearing first 32-bits
    if(address == 0) pins[0] &= ~(all << 0x00);
    if(address == 1) mask[0] &= ~(all << 0x00);
    // Clearing second 32-bits
    if(address == 2) pins[0] &= ~(all << 0x20);
    if(address == 3) mask[0] &= ~(all << 0x20);
    // Clearing first 32-bits
    if(address == 4) pins[1] &= ~(all << 0x00);
    if(address == 5) mask[1] &= ~(all << 0x00);
    // Clearing second 32-bits
    if(address == 6) pins[1] &= ~(all << 0x20);
    if(address == 7) mask[1] &= ~(all << 0x20);

    // Setting first 32-bits
    if(address == 0) pins[0] |= (setting << 0x00);
    if(address == 1) mask[0] |= (setting << 0x00);
    // Setting second 32-bits
    if(address == 2) pins[0] |= (setting << 0x20);
    if(address == 3) mask[0] |= (setting << 0x20);
    // Setting first 32-bits
    if(address == 4) pins[1] |= (setting << 0x00);
    if(address == 5) mask[1] |= (setting << 0x00);
    // Setting second 32-bits
    if(address == 6) pins[1] |= (setting << 0x20);
    if(address == 7) mask[1] |= (setting << 0x20);

#else
    // Clearing first 32-bits
    if(address == 0) pins[0] &= ~(all << 0x00);
    if(address == 1) mask[0] &= ~(all << 0x00);
    // Clearing second 32-bits
    if(address == 2) pins[0] &= ~(all << 0x20);
    if(address == 3) mask[0] &= ~(all << 0x20);

    // Setting first 32-bits
    if(address == 0) pins[0] |= (setting << 0x00);
    if(address == 1) mask[0] |= (setting << 0x00);
    // Setting second 32-bits
    if(address == 2) pins[0] |= (setting << 0x20);
    if(address == 3) mask[0] |= (setting << 0x20);
#endif

    if(address > 3)
        std::cout << "UHD: WARNING: User defined registers [4:256] not defined" << std::endl;

    // Ship if address 3 was written to.
    if(address == 3)
    {
        gpio_burst_req pkt;
#ifdef PV_TATE
	    pkt.header = (((uint64_t) 0x3) << 32) + 1;
#else
	    pkt.header = ((uint64_t) 0x3) << 32;
#endif
        pkt.tv_sec = _command_time.get_full_secs();
        pkt.tv_psec = _command_time.get_frac_secs() * 1e12;
#ifdef PV_TATE
        pkt.pins[1] = pins[1];
        pkt.mask[1] = mask[1];
#endif
        pkt.pins[0] = pins[0];
        pkt.mask[0] = mask[0];

        std::printf(
            "SHIPPING(set_user_reg):\n"
            "0x%016llX\n"
            "0x%016llX\n"
            "0x%016llX\n"
            "0x%016llX\n"
            "0x%016llX\n", pkt.header, pkt.tv_sec, pkt.tv_psec, pkt.pins, pkt.mask);

        boost::endian::native_to_big_inplace(pkt.header);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.tv_sec);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.tv_psec);
#ifdef PV_TATE
        boost::endian::native_to_big_inplace((uint64_t&) pkt.pins[1]);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.mask[1]);
#endif
        boost::endian::native_to_big_inplace((uint64_t&) pkt.pins[0]);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.mask[0]);
        #ifdef DEBUG_COUT
        std::cout << "GPIO packet size: " << sizeof(pkt) << " bytes" << std::endl;
        #endif

        send_gpio_burst_req(pkt);
    }
}

void crimson_tng_impl::set_properties_from_addr() {

	static const std::string crimson_prop_prefix( "crimson:" );
	static const std::vector<std::string> blacklist { "crimson:sob" };

	for( auto & prop: device_addr.keys() ) {
		if ( 0 == prop.compare( 0, crimson_prop_prefix.length(), crimson_prop_prefix ) ) {

			bool is_blacklisted = false;
			for( auto & e: blacklist ) {
				if ( e == prop ) {
					is_blacklisted = true;
				}
			}
			if ( is_blacklisted ) {
				continue;
			}

			std::string key = prop.substr( crimson_prop_prefix.length() );
			std::string expected_string = device_addr[ prop ];

			set_string( key, expected_string );

			std::string actual_string = get_string( key );
			if ( actual_string != expected_string ) {
				UHD_LOGGER_ERROR("CRIMSON_IMPL")
					<< __func__ << "(): "
					<< "Setting Crimson property failed: "
					<< "key: '"<< key << "', "
					<< "expected val: '" << expected_string << "', "
					<< "actual val: '" << actual_string  << "'"
					<< std::endl;
			}
		}
	}
}

/***********************************************************************
 * Discovery over the udp transport
 **********************************************************************/
// This find function will be called if a hint is passed onto the find function
static device_addrs_t crimson_tng_find_with_addr(const device_addr_t &hint)
{
	uhd::time_spec_t then, now;

    // temporarily make a UDP device only to look for devices
    // loop for all the available ports, if none are available, that means all 8 are open already
    udp_simple::sptr comm = udp_simple::make_broadcast(
        hint["addr"], BOOST_STRINGIZE(CRIMSON_TNG_FW_COMMS_UDP_PORT));

    then = uhd::get_system_time();

    //send request for echo
    comm->send(asio::buffer("1,get,fpga/about/name", sizeof("1,get,fpga/about/name")));

    //loop for replies from the broadcast until it times out
    device_addrs_t addrs;
    char buff[CRIMSON_TNG_FW_COMMS_MTU] = {};

    for(
		float to = 0.2;
    	comm->recv(asio::buffer(buff), to);
    	to = 0.05
    ) {
        now = uhd::get_system_time();

        // parse the return buffer and store it in a vector
        std::vector<std::string> tokens;
        tng_csv_parse(tokens, buff, ',');
        if (tokens.size() < 3) break;
        if (tokens[1].c_str()[0] == CMD_ERROR) break;
#ifdef PV_TATE
        if (tokens[2] != "tate") break;
#else
        if (tokens[2] != "crimson_tng") break;
#endif

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
            //UHD_LOGGER_INFO( "CRIMSON_IMPL" ) << "Found crimson_tng at " << new_addr[ "addr" ] << " in " << ( (now - then).get_real_secs() ) << " s" << std::endl;
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

#ifdef PV_TATE
    if (hint.has_key("type") and hint["type"] != "tate") return addrs;
#else
    if (hint.has_key("type") and hint["type"] != "crimson_tng") return addrs;
#endif

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
            UHD_LOGGER_ERROR("CRIMSON_IMPL") << "CRIMSON_TNG Network discovery error " << ex.what() << std::endl;
        }
        catch(...)
        {
            UHD_LOGGER_ERROR("CRIMSON_IMPL") << "CRIMSON_TNG Network discovery unknown error " << std::endl;
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
        for( auto & if_addrs: get_if_addrs() )
        {
            //avoid the loopback device
            if ( asio::ip::address_v4::loopback().to_string() == if_addrs.inet ) {
            	continue;
            }

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

/**
 * Buffer Management / Time Diff
 */

// SoB: Time Diff (Time Diff mechanism is used to get an accurate estimate of Crimson's absolute time)
static constexpr double tick_period_ns = 2.0 / CRIMSON_TNG_MASTER_CLOCK_RATE * 1e9;
static inline int64_t ticks_to_nsecs( int64_t tv_tick ) {
	return (int64_t)( (double) tv_tick * tick_period_ns ) /* [tick] * [ns/tick] = [ns] */;
}
static inline int64_t nsecs_to_ticks( int64_t tv_nsec ) {
	return (int64_t)( (double) tv_nsec / tick_period_ns )  /* [ns] / [ns/tick] = [tick] */;
}

static inline void make_time_diff_packet( time_diff_req & pkt, time_spec_t ts = uhd::get_system_time() ) {
	pkt.header = (uint64_t)0x20002 << 16;
	pkt.tv_sec = ts.get_full_secs();
	pkt.tv_tick = nsecs_to_ticks( (int64_t) ( ts.get_frac_secs() * 1e9 ) );

	boost::endian::native_to_big_inplace( pkt.header );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_sec );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_tick );
}

void crimson_tng_impl::make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, const uhd::time_spec_t & now, const size_t channel, uhd::usrp::rx_stream_cmd & pkt ) {

    typedef boost::tuple<bool, bool, bool, bool> inst_t;
    static const uhd::dict<stream_cmd_t::stream_mode_t, inst_t> mode_to_inst = boost::assign::map_list_of
                                                            //reload, chain, samps, stop
        (stream_cmd_t::STREAM_MODE_START_CONTINUOUS,   inst_t(true,  true,  false, false))
        (stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS,    inst_t(false, false, false, true))
        (stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE, inst_t(false, false, true,  false))
        (stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE, inst_t(false, true,  true,  false))
    ;
    static const uint8_t channel_bits = 16;
    static const uint64_t channel_mask = ( 1 << channel_bits ) - 1;

    // XXX: @CF: 20180404: header should be 0x10001
	pkt.header = ( 0x1 << channel_bits ) | ( channel & channel_mask );

    //setup the instruction flag values
    bool inst_reload, inst_chain, inst_samps, inst_stop;
    boost::tie(inst_reload, inst_chain, inst_samps, inst_stop) = mode_to_inst[cmd.stream_mode];

    pkt.header |= inst_reload ? ( 0b1000LL << 36 ) : 0;
    pkt.header |= inst_chain  ? ( 0b0100LL << 36 ) : 0;
    pkt.header |= inst_samps  ? ( 0b0010LL << 36 ) : 0;
    pkt.header |= inst_stop   ? ( 0b0001LL << 36 ) : 0;

	uhd::time_spec_t ts = cmd.stream_now ? now : cmd.time_spec;
	pkt.tv_sec = ts.get_full_secs();
	pkt.tv_psec = ts.get_frac_secs() * 1e12;

	pkt.nsamples = inst_samps ? cmd.num_samps : 0;

//	std::cout << "header: " << std::hex << std::setw( 16 ) << std::setfill('0') << pkt.header << std::endl;
//	std::cout << "tv_sec: " << std::dec << pkt.tv_sec << std::endl;
//	std::cout << "tv_psec: " << std::dec << pkt.tv_psec << std::endl;
//	std::cout << "nsampls: " << std::dec << pkt.nsamples << std::endl;

	boost::endian::native_to_big_inplace( pkt.header );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_sec );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_psec );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.nsamples );
}

void crimson_tng_impl::send_rx_stream_cmd_req( const rx_stream_cmd & req ) {
	_time_diff_iface->send( boost::asio::const_buffer( & req, sizeof( req ) ) );
}

/// SoB Time Diff: send sync packet (must be done before reading flow iface)
void crimson_tng_impl::time_diff_send( const uhd::time_spec_t & crimson_now ) {

	time_diff_req pkt;

	// Input to Process (includes feedback from PID Controller)
	make_time_diff_packet(
		pkt,
		crimson_now
	);

	_time_diff_iface->send( boost::asio::const_buffer( &pkt, sizeof( pkt ) ) );
}

bool crimson_tng_impl::time_diff_recv( time_diff_resp & tdr ) {

	size_t r;

	r = _time_diff_iface->recv( boost::asio::mutable_buffer( & tdr, sizeof( tdr ) ) );

	if ( 0 == r ) {
		return false;
	}

	boost::endian::big_to_native_inplace( tdr.tv_sec );
	boost::endian::big_to_native_inplace( tdr.tv_tick );

	return true;
}

/// SoB Time Diff: feed the time diff error back into out control system
void crimson_tng_impl::time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now ) {

	static const double sp = 0.0;

	double pv = (double) tdr.tv_sec + (double)ticks_to_nsecs( tdr.tv_tick ) / 1e9;

	double cv = _time_diff_pidc.update_control_variable( sp, pv, now.get_real_secs() );
	_time_diff_converged = _time_diff_pidc.is_converged( now.get_real_secs() );

	// For SoB, record the instantaneous time difference + compensation
	if ( _time_diff_converged ) {
		time_diff_set( cv );
	}
}

void crimson_tng_impl::start_bm() {

	std::lock_guard<std::mutex> _lock( _bm_thread_mutex );

	if ( ! _bm_thread_needed ) {
		return;
	}

	if ( ! _bm_thread_running ) {

		_bm_thread_should_exit = false;
		_bm_thread = std::thread( bm_thread_fn, this );

		_time_diff_converged = false;
		for(
			time_spec_t time_then = uhd::get_system_time(),
				time_now = time_then
				;
			! time_diff_converged()
				;
			time_now = uhd::get_system_time()
		) {
			if ( (time_now - time_then).get_full_secs() > 20 ) {
				UHD_LOGGER_ERROR("CRIMSON_IMPL")
					<< "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling Crimson?"
					<< std::endl;
				throw runtime_error( "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling Crimson?" );
			}
			usleep( 100000 );
		}
	}
}

void crimson_tng_impl::stop_bm() {

	if ( _bm_thread_running ) {

		_bm_thread_should_exit = true;
		_bm_thread.join();

	}
}

bool crimson_tng_impl::time_diff_converged() {
	return _time_diff_converged;
}

// the buffer monitor thread
void crimson_tng_impl::bm_thread_fn( crimson_tng_impl *dev ) {

	dev->_bm_thread_running = true;

	const uhd::time_spec_t T( 1.0 / (double) CRIMSON_TNG_UPDATE_PER_SEC );
	std::vector<size_t> fifo_lvl( CRIMSON_TNG_TX_CHANNELS );
	uhd::time_spec_t now, then, dt;
	uhd::time_spec_t crimson_now;
	struct timespec req, rem;

	double time_diff;

	struct time_diff_resp tdr;

	//Gett offset
	now = uhd::get_system_time();
	dev->time_diff_send( now );
	dev->time_diff_recv( tdr );
	dev->_time_diff_pidc.set_offset((double) tdr.tv_sec + (double)ticks_to_nsecs( tdr.tv_tick ) / 1e9);

	for(
		now = uhd::get_system_time(),
			then = now + T
			;

		! dev->_bm_thread_should_exit
			;

		then += T,
			now = uhd::get_system_time()
	) {

		dt = then - now;
		if ( dt > 0.0 ) {
			req.tv_sec = dt.get_full_secs();
			req.tv_nsec = dt.get_frac_secs() * 1e9;
			nanosleep( &req, &rem );
		}

		time_diff = dev->_time_diff_pidc.get_control_variable();
		now = uhd::get_system_time();
		crimson_now = now + time_diff;

		dev->time_diff_send( crimson_now );
		if ( ! dev->time_diff_recv( tdr ) ) {
			continue;
		}
		dev->time_diff_process( tdr, now );
		//dev->fifo_update_process( tdr );

#if 0
			// XXX: overruns - we need to fix this
			now = uhd::get_system_time();

			if ( now >= then + T ) {
				UHD_LOGGER_INFO( "CRIMSON_IMPL" )
					<< __func__ << "(): Overran time for update by " << ( now - ( then + T ) ).get_real_secs() << " s"
					<< std::endl;
			}
#endif
	}
	dev->_bm_thread_running = false;
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
	set_log_level( uhd::log::severity_level::info );
    device::register_device(&crimson_tng_find, &crimson_tng_make, device::USRP);
}

/***********************************************************************
 * Structors
 **********************************************************************/
// Macro to create the tree, all properties created with this are R/W properties
#define TREE_CREATE_RW(PATH, PROP, TYPE, HANDLER)						\
	do { _tree->create<TYPE> (PATH)								\
    		.set( get_ ## HANDLER (PROP))							\
		.add_desired_subscriber(boost::bind(&crimson_tng_impl::set_ ## HANDLER, this, (PROP), _1))	\
		.set_publisher(boost::bind(&crimson_tng_impl::get_ ## HANDLER, this, (PROP)    ));	\
	} while(0)

// Macro to create the tree, all properties created with this are RO properties
#define TREE_CREATE_RO(PATH, PROP, TYPE, HANDLER)						\
	do { _tree->create<TYPE> (PATH)								\
    		.set( get_ ## HANDLER (PROP))							\
		.publish  (boost::bind(&crimson_tng_impl::get_ ## HANDLER, this, (PROP)    ));	\
	} while(0)

// Macro to create the tree, all properties created with this are static
#define TREE_CREATE_ST(PATH, TYPE, VAL) 	( _tree->create<TYPE>(PATH).set(VAL) )

crimson_tng_impl::crimson_tng_impl(const device_addr_t &_device_addr)
:
	device_addr( _device_addr ),
	_time_diff( 0 ),
	_time_diff_converged( false ),
	_bm_thread_needed( false ),
	_bm_thread_running( false ),
	_bm_thread_should_exit( false ),
    _command_time()
{
    _type = device::CRIMSON_TNG;
    device_addr = _device_addr;

    //setup the dsp transport hints (default to a large recv buff)
    if (not device_addr.has_key("recv_buff_size")){
        #if defined(UHD_PLATFORM_MACOS) || defined(UHD_PLATFORM_BSD)
            //limit buffer resize on macos or it will error
            device_addr["recv_buff_size"] = "1e6";
        #elif defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)
            //set to half-a-second of buffering at max rate
            device_addr["recv_buff_size"] = "50e6";
        #endif
    }
    if (not device_addr.has_key("send_buff_size")){
        //The buffer should be the size of the SRAM on the device,
        //because we will never commit more than the SRAM can hold.
        device_addr["send_buff_size"] = boost::lexical_cast<std::string>( CRIMSON_TNG_BUFF_SIZE * sizeof( std::complex<int16_t> ) );
    }

    device_addrs_t device_args = separate_device_addr(device_addr);

    // XXX: @CF: 20180227: we need the property tree to extract actual values from hardware
    //extract the user's requested MTU size or default
//    mtu_result_t user_mtu;
//    user_mtu.recv_mtu = size_t(device_addr.cast<double>("recv_frame_size", udp_simple::mtu));
//    user_mtu.send_mtu = size_t(device_addr.cast<double>("send_frame_size", udp_simple::mtu));
//
//    try{
//        //calculate the minimum send and recv mtu of all devices
//        mtu_result_t mtu = determine_mtu(device_args[0]["addr"], user_mtu);
//        for (size_t i = 1; i < device_args.size(); i++){
//            mtu_result_t mtu_i = determine_mtu(device_args[i]["addr"], user_mtu);
//            mtu.recv_mtu = std::min(mtu.recv_mtu, mtu_i.recv_mtu);
//            mtu.send_mtu = std::min(mtu.send_mtu, mtu_i.send_mtu);
//        }
//
//        device_addr["recv_frame_size"] = boost::lexical_cast<std::string>(mtu.recv_mtu);
//        device_addr["send_frame_size"] = boost::lexical_cast<std::string>(mtu.send_mtu);
//
//        UHD_LOGGER_INFO("CRIMSON_IMPL") << boost::format("Current recv frame size: %d bytes") % mtu.recv_mtu << std::endl;
//        UHD_LOGGER_INFO("CRIMSON_IMPL") << boost::format("Current send frame size: %d bytes") % mtu.send_mtu << std::endl;
//    }
//    catch(const uhd::not_implemented_error &){
//        //just ignore this error, makes older fw work...
//    }

    device_args = separate_device_addr(device_addr); //update args for new frame sizes

    static const size_t mbi = 0;
    static const std::string mb = std::to_string( mbi );
    // Makes the UDP comm connection
    _mbc[mb].iface = crimson_tng_iface::make(
		udp_simple::make_connected(
			_device_addr["addr"],
			BOOST_STRINGIZE( CRIMSON_TNG_FW_COMMS_UDP_PORT )
		)
    );

    // TODO make transports for each RX/TX chain
    // TODO check if locked already
    // TODO lock the Crimson device to this process, this will prevent the Crimson device being used by another program

    // Property paths
    const fs_path mb_path   = "/mboards/0";
    const fs_path time_path = mb_path / "time";
    const fs_path tx_path   = mb_path / "tx";
    const fs_path rx_path   = mb_path / "rx";

    std::string lc_num;

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

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////
#ifdef PV_TATE
    static const std::vector<size_t> default_map { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
#else
    static const std::vector<size_t> default_map { 0, 1, 2, 3 };
#endif
    _tree->create<std::vector<size_t> >(mb_path / "rx_chan_dsp_mapping").set(default_map);
    _tree->create<std::vector<size_t> >(mb_path / "tx_chan_dsp_mapping").set(default_map);
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec").add_coerced_subscriber(boost::bind(&crimson_tng_impl::update_rx_subdev_spec, this, mb, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec").add_coerced_subscriber(boost::bind(&crimson_tng_impl::update_tx_subdev_spec, this, mb, _1));

    TREE_CREATE_ST(mb_path / "vendor", std::string, "Per Vices");
    TREE_CREATE_ST(mb_path / "name",   std::string, "FPGA Board");
    TREE_CREATE_RW(mb_path / "id",         "fpga/about/id",     std::string, string);
    TREE_CREATE_RW(mb_path / "serial",     "fpga/about/serial", std::string, string);
    TREE_CREATE_RW(mb_path / "fw_version", "fpga/about/fw_ver", std::string, string);
    TREE_CREATE_RW(mb_path / "hw_version", "fpga/about/hw_ver", std::string, string);
    TREE_CREATE_RW(mb_path / "sw_version", "fpga/about/sw_ver", std::string, string);
    TREE_CREATE_RW(mb_path / "blink", "fpga/board/led", int, int);
    TREE_CREATE_RW(mb_path / "temp", "fpga/board/temp", std::string, string);

    TREE_CREATE_RW(mb_path / "user/regs", "fpga/user/regs", user_reg_t, user_reg);

    TREE_CREATE_RW(mb_path / "sfpa/ip_addr",  "fpga/link/sfpa/ip_addr",  std::string, string);
    TREE_CREATE_RW(mb_path / "sfpa/mac_addr", "fpga/link/sfpa/mac_addr", std::string, string);
    TREE_CREATE_RW(mb_path / "sfpa/pay_len",  "fpga/link/sfpa/pay_len",  std::string, string);
    TREE_CREATE_RW(mb_path / "sfpb/ip_addr",  "fpga/link/sfpb/ip_addr",  std::string, string);
    TREE_CREATE_RW(mb_path / "sfpb/mac_addr", "fpga/link/sfpb/mac_addr", std::string, string);
    TREE_CREATE_RW(mb_path / "sfpb/pay_len",  "fpga/link/sfpb/pay_len",  std::string, string);
    TREE_CREATE_RW(mb_path / "trigger/sma_dir", "fpga/trigger/sma_dir",  std::string, string);
    TREE_CREATE_RW(mb_path / "trigger/sma_pol", "fpga/trigger/sma_pol",  std::string, string);

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
    TREE_CREATE_ST(mb_path / "tick_rate", double, CRIMSON_TNG_MASTER_CLOCK_RATE / 2);

    TREE_CREATE_RW(time_path / "cmd", "time/clk/cmd",      time_spec_t, time_spec);
    TREE_CREATE_RW(time_path / "now", "time/clk/cur_time", time_spec_t, time_spec);
    TREE_CREATE_RW(time_path / "pps", "time/clk/pps", 	   time_spec_t, time_spec);

    TREE_CREATE_ST(mb_path / "eeprom", mboard_eeprom_t, mboard_eeprom_t());

    // This property chooses internal or external clock source
    TREE_CREATE_RW(mb_path / "time_source"  / "value",  	"time/source/ref",  	std::string, string);
    TREE_CREATE_RW(mb_path / "clock_source" / "value",      "time/source/ref",	std::string, string);
    TREE_CREATE_RW(mb_path / "clock_source" / "external",	"time/source/ref",	std::string, string);
    TREE_CREATE_ST(mb_path / "clock_source" / "external" / "value", double, CRIMSON_TNG_EXT_CLK_RATE);
    TREE_CREATE_ST(mb_path / "clock_source" / "output", bool, true);
    TREE_CREATE_ST(mb_path / "time_source"  / "output", bool, true);

    TREE_CREATE_ST(mb_path / "sensors" / "ref_locked", sensor_value_t, sensor_value_t( "Reference", true, "unlocked", "locked" ) );

    // No GPSDO support on Crimson
    // TREE_CREATE_ST(mb_path / "sensors" / "ref_locked", sensor_value_t, sensor_value_t("NA", "0", "NA"));

    // loop for all RX chains
    for( size_t dspno = 0; dspno < CRIMSON_TNG_RX_CHANNELS; dspno++ ) {
		std::string lc_num  = boost::lexical_cast<std::string>((char)(dspno + 'a'));
		std::string num     = boost::lexical_cast<std::string>((char)(dspno + 'A'));
		std::string chan    = "Channel_" + num;

		const fs_path rx_codec_path = mb_path / "rx_codecs" / num;
		const fs_path rx_fe_path    = mb_path / "dboards" / num / "rx_frontends" / chan;
		const fs_path db_path       = mb_path / "dboards" / num;
		const fs_path rx_dsp_path   = mb_path / "rx_dsps" / dspno;
		const fs_path rx_link_path  = mb_path / "rx_link" / dspno;

		static const std::vector<std::string> antenna_options = boost::assign::list_of("SMA")("None");
		_tree->create<std::vector<std::string> >(rx_fe_path / "antenna" / "options").set(antenna_options);

		_tree->create<std::string>(rx_fe_path / "antenna" / "value").set("N/A");

		static const std::vector<std::string> sensor_options = boost::assign::list_of("lo_locked");
		_tree->create<std::vector<std::string> >(rx_fe_path / "sensors").set(sensor_options);

        // RX Triggers
        TREE_CREATE_RW(rx_path / dspno / "/trigger/sma_mode"       , "rx_" + lc_num + "/trigger/sma_mode"       , std::string, string);
        TREE_CREATE_RW(rx_path / dspno / "/trigger/trig_sel"       , "rx_" + lc_num + "/trigger/trig_sel"       , std::string, string);
        TREE_CREATE_RW(rx_path / dspno / "/trigger/edge_backoff"   , "rx_" + lc_num + "/trigger/edge_backoff"   , std::string, string);
        TREE_CREATE_RW(rx_path / dspno / "/trigger/edge_sample_num", "rx_" + lc_num + "/trigger/edge_sample_num", std::string, string);
        TREE_CREATE_RW(rx_path / dspno / "/trigger/ufl_mode"       , "rx_" + lc_num + "/trigger/ufl_mode"       , std::string, string);
        TREE_CREATE_RW(rx_path / dspno / "/trigger/ufl_dir"        , "rx_" + lc_num + "/trigger/ufl_dir"        , std::string, string);
        TREE_CREATE_RW(rx_path / dspno / "/trigger/ufl_pol"        , "rx_" + lc_num + "/trigger/ufl_pol"        , std::string, string);

		// Power status
		TREE_CREATE_RW(rx_path / dspno / "pwr", "rx_"+lc_num+"/pwr", std::string, string);

		// Channel Stream Status
		TREE_CREATE_RW(rx_path / dspno / "stream", "rx_"+lc_num+"/stream", std::string, string);

		// Codecs, phony properties for Crimson
		TREE_CREATE_RW(rx_codec_path / "gains", "rx_"+lc_num+"/dsp/gain", int, int);
		TREE_CREATE_ST(rx_codec_path / "name", std::string, "RX Codec");

		// Daughter Boards' Frontend Settings
		TREE_CREATE_ST(rx_fe_path / "name",   std::string, "RX Board");

	    // RX bandwidth
		TREE_CREATE_ST(rx_fe_path / "bandwidth" / "value", double, CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0 );
		TREE_CREATE_ST(rx_fe_path / "bandwidth" / "range", meta_range_t, meta_range_t( CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0, CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0 ) );

		TREE_CREATE_ST(rx_fe_path / "freq", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));

		TREE_CREATE_ST(rx_fe_path / "dc_offset" / "enable", bool, false);
		TREE_CREATE_ST(rx_fe_path / "dc_offset" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));
		TREE_CREATE_ST(rx_fe_path / "iq_balance" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));

		TREE_CREATE_RW(rx_fe_path / "connection",  "rx_"+lc_num+"/link/iface", std::string, string);

		TREE_CREATE_ST(rx_fe_path / "use_lo_offset", bool, true );
		TREE_CREATE_ST(rx_fe_path / "lo_offset" / "value", double, 15e6 );

		TREE_CREATE_ST(rx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));
		TREE_CREATE_ST(rx_fe_path / "gain" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_RF_RX_GAIN_RANGE_START, CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP, CRIMSON_TNG_RF_RX_GAIN_RANGE_STEP));

		TREE_CREATE_RW(rx_fe_path / "freq"  / "value", "rx_"+lc_num+"/rf/freq/val" , double, double);
		TREE_CREATE_ST(rx_fe_path / "gains", std::string, "gain" );
		TREE_CREATE_RW(rx_fe_path / "gain"  / "value", "rx_"+lc_num+"/rf/gain/val" , double, double);
		TREE_CREATE_RW(rx_fe_path / "atten" / "value", "rx_"+lc_num+"/rf/atten/val", double, double);

		// RF band
		TREE_CREATE_RW(rx_fe_path / "freq" / "band", "rx_"+lc_num+"/rf/freq/band", int, int);

		// RF receiver LNA
		TREE_CREATE_RW(rx_fe_path / "freq"  / "lna", "rx_"+lc_num+"/rf/freq/lna" , int, int);

		// these are phony properties for Crimson
		TREE_CREATE_ST(db_path / "rx_eeprom",  dboard_eeprom_t, dboard_eeprom_t());
		TREE_CREATE_ST(db_path / "gdb_eeprom", dboard_eeprom_t, dboard_eeprom_t());

		// DSPs

                
		switch( dspno + 'A' ) {
		case 'A':
		case 'B':
#ifdef PV_TATE
                default:
#endif
                    TREE_CREATE_ST(rx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			break;
#ifndef PV_TATE
                case 'C':
		case 'D':
			TREE_CREATE_ST(rx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP  / 2, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP , CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(rx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2, CRIMSON_TNG_RATE_RANGE_STEP));
			break;
#endif
		}

		_tree->create<double> (rx_dsp_path / "rate" / "value")
			.set( get_double ("rx_"+lc_num+"/dsp/rate"))
			.add_desired_subscriber(boost::bind(&crimson_tng_impl::update_rx_samp_rate, this, mb, (size_t) dspno, _1))
			.set_publisher(boost::bind(&crimson_tng_impl::get_double, this, ("rx_"+lc_num+"/dsp/rate")    ));

		TREE_CREATE_RW(rx_dsp_path / "freq" / "value", "rx_"+lc_num+"/dsp/nco_adj", double, double);
		TREE_CREATE_RW(rx_dsp_path / "bw" / "value",   "rx_"+lc_num+"/dsp/rate",    double, double);

		typedef stream_cmd_t stream_cmd;
		TREE_CREATE_RW(rx_dsp_path / "stream_cmd",  "rx_"+lc_num+"/stream_cmd", stream_cmd, stream_cmd);

		TREE_CREATE_RW(rx_dsp_path / "nco", "rx_"+lc_num+"/dsp/nco_adj", double, double);

		// Link settings
		TREE_CREATE_RW(rx_link_path / "vita_en", "rx_"+lc_num+"/link/vita_en", std::string, string);
		TREE_CREATE_RW(rx_link_path / "ip_dest", "rx_"+lc_num+"/link/ip_dest", std::string, string);
		TREE_CREATE_RW(rx_link_path / "port",    "rx_"+lc_num+"/link/port",    std::string, string);
		TREE_CREATE_RW(rx_link_path / "iface",   "rx_"+lc_num+"/link/iface",   std::string, string);

		zero_copy_xport_params zcxp;
		udp_zero_copy::buff_params bp;

	    static const size_t ip_udp_size = 0
	    	+ 60 // IPv4 Header
			+ 8  // UDP Header
	    ;
		const size_t bpp = CRIMSON_TNG_MAX_MTU - ip_udp_size;

		zcxp.send_frame_size = 0;
		zcxp.recv_frame_size = bpp;
		zcxp.num_send_frames = 0;
		zcxp.num_recv_frames = DEFAULT_NUM_FRAMES;

		_mbc[mb].rx_dsp_xports.push_back(
			udp_stream_zero_copy::make(
				_tree->access<std::string>( rx_link_path / "ip_dest" ).get(),
				std::stoi( _tree->access<std::string>( rx_link_path / "port" ).get() ),
				"127.0.0.1",
				1,
				zcxp,
				bp,
				device_addr
			)
		);
    }

    // loop for all TX chains
    for( int dspno = 0; dspno < CRIMSON_TNG_TX_CHANNELS; dspno++ ) {
		std::string lc_num  = boost::lexical_cast<std::string>((char)(dspno + 'a'));
		std::string num     = boost::lexical_cast<std::string>((char)(dspno + 'A'));
		std::string chan    = "Channel_" + num;

		const fs_path tx_codec_path = mb_path / "tx_codecs" / num;
		const fs_path tx_fe_path    = mb_path / "dboards" / num / "tx_frontends" / chan;
		const fs_path db_path       = mb_path / "dboards" / num;
		const fs_path tx_dsp_path   = mb_path / "tx_dsps" / dspno;
		const fs_path tx_link_path  = mb_path / "tx_link" / dspno;

		static const std::vector<std::string> antenna_options = boost::assign::list_of("SMA")("None");
		_tree->create<std::vector<std::string> >(tx_fe_path / "antenna" / "options").set(antenna_options);

		_tree->create<std::string>(tx_fe_path / "antenna" / "value").set("N/A");

		static const std::vector<std::string> sensor_options = boost::assign::list_of("lo_locked");
		_tree->create<std::vector<std::string> >(tx_fe_path / "sensors").set(sensor_options);

        // TX Triggers
        TREE_CREATE_RW(tx_path / dspno / "/trigger/sma_mode"       , "tx_" + lc_num + "/trigger/sma_mode"       , std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/trig_sel"       , "tx_" + lc_num + "/trigger/trig_sel"       , std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/edge_backoff"   , "tx_" + lc_num + "/trigger/edge_backoff"   , std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/edge_sample_num", "tx_" + lc_num + "/trigger/edge_sample_num", std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/ufl_mode"       , "tx_" + lc_num + "/trigger/ufl_mode"       , std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/ufl_dir"        , "tx_" + lc_num + "/trigger/ufl_dir"        , std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/ufl_pol"        , "tx_" + lc_num + "/trigger/ufl_pol"        , std::string, string);
        TREE_CREATE_RW(tx_path / dspno / "/trigger/gating"         , "tx_" + lc_num + "/trigger/gating"         , std::string, string);

		// Actual frequency values
		TREE_CREATE_RW(tx_path / chan / "freq" / "value", "tx_"+lc_num+"/rf/freq/val", double, double);

		// Power status
		TREE_CREATE_RW(tx_path / dspno / "pwr", "tx_"+lc_num+"/pwr", std::string, string);

		// Codecs, phony properties for Crimson
		TREE_CREATE_RW(tx_codec_path / "gains", "tx_"+lc_num+"/dsp/gain", int, int);
		TREE_CREATE_ST(tx_codec_path / "name", std::string, "TX Codec");

		// Daughter Boards' Frontend Settings
		TREE_CREATE_ST(tx_fe_path / "name",   std::string, "TX Board");

	    // TX bandwidth
		TREE_CREATE_ST(tx_fe_path / "bandwidth" / "value", double, CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0 );
		TREE_CREATE_ST(tx_fe_path / "bandwidth" / "range", meta_range_t, meta_range_t( CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0, CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0 ) );

		TREE_CREATE_ST(tx_fe_path / "freq", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));

		TREE_CREATE_ST(tx_fe_path / "dc_offset" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));
		TREE_CREATE_ST(tx_fe_path / "iq_balance" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));

		TREE_CREATE_RW(tx_fe_path / "connection",  "tx_"+lc_num+"/link/iface", std::string, string);

		TREE_CREATE_ST(tx_fe_path / "use_lo_offset", bool, false);
		TREE_CREATE_RW(tx_fe_path / "lo_offset" / "value", "tx_"+lc_num+"/rf/dac/nco", double, double);

		TREE_CREATE_ST(tx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));
		TREE_CREATE_ST(tx_fe_path / "gain" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_RF_TX_GAIN_RANGE_START, CRIMSON_TNG_RF_TX_GAIN_RANGE_STOP, CRIMSON_TNG_RF_TX_GAIN_RANGE_STEP));

		TREE_CREATE_RW(tx_fe_path / "freq"  / "value", "tx_"+lc_num+"/rf/freq/val" , double, double);
		TREE_CREATE_ST(tx_fe_path / "gains", std::string, "gain" );
		TREE_CREATE_RW(tx_fe_path / "gain"  / "value", "tx_"+lc_num+"/rf/gain/val" , double, double);

		// RF band
		TREE_CREATE_RW(tx_fe_path / "freq" / "band", "tx_"+lc_num+"/rf/freq/band", int, int);

		// these are phony properties for Crimson
		TREE_CREATE_ST(db_path / "tx_eeprom",  dboard_eeprom_t, dboard_eeprom_t());

		// DSPs
		switch( dspno + 'A' ) {
		case 'A':
		case 'B':
#ifdef PV_TATE
                default:
#endif
			TREE_CREATE_ST(tx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
			break;
#ifndef PV_TATE
		case 'C':
		case 'D':
			TREE_CREATE_ST(tx_dsp_path / "rate" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2, CRIMSON_TNG_RATE_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "freq" / "range", meta_range_t,
				meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
			TREE_CREATE_ST(tx_dsp_path / "bw" / "range",   meta_range_t,
				meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP / 2, CRIMSON_TNG_RATE_RANGE_STEP));
			break;
#endif
		}

		_tree->create<double> (tx_dsp_path / "rate" / "value")
			.set( get_double ("tx_"+lc_num+"/dsp/rate"))
			.add_desired_subscriber(boost::bind(&crimson_tng_impl::update_tx_samp_rate, this, mb, (size_t) dspno, _1))
			.set_publisher(boost::bind(&crimson_tng_impl::get_double, this, ("tx_"+lc_num+"/dsp/rate")    ));

		TREE_CREATE_RW(tx_dsp_path / "bw" / "value",   "tx_"+lc_num+"/dsp/rate",    double, double);

		TREE_CREATE_RW(tx_dsp_path / "freq" / "value", "tx_"+lc_num+"/dsp/nco_adj", double, double);

		TREE_CREATE_RW(tx_dsp_path / "nco", "tx_"+lc_num+"/dsp/nco_adj", double, double);
		TREE_CREATE_RW(tx_fe_path / "nco", "tx_"+lc_num+"/rf/dac/nco", double, double);

		// Link settings
		TREE_CREATE_RW(tx_link_path / "vita_en", "tx_"+lc_num+"/link/vita_en", std::string, string);
		TREE_CREATE_RW(tx_link_path / "port",    "tx_"+lc_num+"/link/port",    std::string, string);
		TREE_CREATE_RW(tx_link_path / "iface",   "tx_"+lc_num+"/link/iface",   std::string, string);


		zero_copy_xport_params zcxp;
		udp_zero_copy::buff_params bp;

	    static const size_t ip_udp_size = 0
	    	+ 60 // IPv4 Header
			+ 8  // UDP Header
	    ;
		const size_t bpp = CRIMSON_TNG_MAX_MTU - ip_udp_size;

		zcxp.send_frame_size = bpp;
		zcxp.recv_frame_size = 0;
		zcxp.num_send_frames = CRIMSON_TNG_BUFF_SIZE * sizeof( std::complex<int16_t> ) / bpp;
		zcxp.num_recv_frames = 0;

		std::string ip_addr;
		uint16_t udp_port;
		std::string sfp;
		get_tx_endpoint( _tree, dspno, ip_addr, udp_port, sfp );

		_mbc[mb].tx_dsp_xports.push_back(
			udp_zero_copy::make(
				ip_addr,
				std::to_string( udp_port ),
				zcxp,
				bp,
				device_addr
			)
		);

		_mbc[mb].fifo_ctrl_xports.push_back(
			udp_simple::make_connected(
				_tree->access<std::string>( mb_path / "link" / sfp / "ip_addr" ).get(),
				std::to_string( _tree->access<int>( mb_path / "fpga" / "board" / "flow_control" / ( sfp + "_port" ) ).get() )
			)
		);
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

	this->io_init();

    //do some post-init tasks
    this->update_rates();
    for(const std::string &mb:  _mbc.keys()){
        fs_path root = "/mboards/" + mb;

        //reset cordic rates and their properties to zero
//        for(const std::string &name:  _tree->list(root / "rx_dsps")){
//            _tree->access<double>(root / "rx_dsps" / name / "freq" / "value").set(0.0);
//        }
//        for(const std::string &name:  _tree->list(root / "tx_dsps")){
//            _tree->access<double>(root / "tx_dsps" / name / "freq" / "value").set(0.0);
//        }

#ifdef PV_TATE
		_tree->access<subdev_spec_t>(root / "rx_subdev_spec").set(subdev_spec_t( "A:Channel_A B:Channel_B C:Channel_C D:Channel_D E:Channel_E F:Channel_F G:Channel_G H:Channel_H I:Channel_I J:Channel_J K:Channel_K L:Channel_L M:Channel_M N:Channel_N O:Channel_O P:Channel_P" ));
		_tree->access<subdev_spec_t>(root / "tx_subdev_spec").set(subdev_spec_t( "A:Channel_A B:Channel_B C:Channel_C D:Channel_D E:Channel_E F:Channel_F G:Channel_G H:Channel_H I:Channel_I J:Channel_J K:Channel_K L:Channel_L M:Channel_M N:Channel_N O:Channel_O P:Channel_P" ));
#else
		_tree->access<subdev_spec_t>(root / "rx_subdev_spec").set(subdev_spec_t( "A:Channel_A B:Channel_B C:Channel_C D:Channel_D" ));
		_tree->access<subdev_spec_t>(root / "tx_subdev_spec").set(subdev_spec_t( "A:Channel_A B:Channel_B C:Channel_C D:Channel_D" ));
#endif
        _tree->access<std::string>(root / "clock_source/value").set("internal");
        _tree->access<std::string>(root / "time_source/value").set("none");

        //GPS installed: use external ref, time, and init time spec
//        if (_mbc[mb].gps and _mbc[mb].gps->gps_detected()){
//            _mbc[mb].time64->enable_gpsdo();
//            UHD_LOGGER_INFO("USRP2") << "Setting references to the internal GPSDO" ;
//            _tree->access<std::string>(root / "time_source/value").set("gpsdo");
//            _tree->access<std::string>(root / "clock_source/value").set("gpsdo");
//        }
    }

	// it does not currently matter whether we use the sfpa or sfpb port atm, they both access the same fpga hardware block
	int sfpa_port = _tree->access<int>( mb_path / "fpga/board/flow_control/sfpa_port" ).get();
	std::string time_diff_ip = _tree->access<std::string>( mb_path / "link" / "sfpa" / "ip_addr" ).get();
	std::string time_diff_port = std::to_string( sfpa_port );
	_time_diff_iface = udp_simple::make_connected( time_diff_ip, time_diff_port );


	_bm_thread_needed = is_bm_thread_needed();
	if ( _bm_thread_needed ) {

		//Initialize "Time Diff" mechanism before starting flow control thread
		time_spec_t ts = uhd::get_system_time();
		_streamer_start_time = ts.get_real_secs();

		// The problem is that this class does not hold a multi_crimson instance
		//Dont set time. Crimson can compensate from 0. Set time will only be used for GPS

		// Tyreus-Luyben tuned PID controller
		_time_diff_pidc = uhd::pidc_tl(
			0.0, // desired set point is 0.0s error
			1.0, // measured K-ultimate occurs with Kp = 1.0, Ki = 0.0, Kd = 0.0
			// measured P-ultimate is inverse of 1/2 the flow-control sample rate
			2.0 / (double)CRIMSON_TNG_UPDATE_PER_SEC
		);

		_time_diff_pidc.set_error_filter_length( CRIMSON_TNG_UPDATE_PER_SEC );

		// XXX: @CF: 20170720: coarse to fine for convergence
		// we coarsely lock on at first, to ensure the class instantiates properly
		// and then switch to a finer error tolerance
		_time_diff_pidc.set_max_error_for_convergence( 100e-6 );
		start_bm();
		_time_diff_pidc.set_max_error_for_convergence( 10e-6 );
	}
}

crimson_tng_impl::~crimson_tng_impl(void)
{
       stop_bm();
}

bool crimson_tng_impl::is_bm_thread_needed() {
	bool r = true;

#ifndef __APPLE__ // eventually use something like HAVE_PROGRAM_INVOCATION_NAME
	static const std::vector<std::string> utils {
		"uhd_find_devices",
		"uhd_usrp_probe",
	};

	// see `man 3 program_invocation_short_name'
	std::string pin( program_invocation_short_name );

	for( auto & s: utils ) {
		if ( s == pin ) {
			r = false;
			break;
		}
	}

#endif

	return r;
}

void crimson_tng_impl::get_tx_endpoint( uhd::property_tree::sptr tree, const size_t & chan, std::string & ip_addr, uint16_t & udp_port, std::string & sfp ) {

	switch( chan ) {
#ifndef PV_TATE
        case 0:
	case 2:
		sfp = "sfpa";
		break;
	case 1:
	case 3:
		sfp = "sfpb";
		break;
#else
        case 0:
	case 1:
        case 2:
        case 3:
		sfp = "sfpa";
                break;
        case 4:
	case 5:
        case 6:
        case 7:
                sfp = "sfpb";
                break;
        case 8:
	case 9:
        case 10:
        case 11:
                sfp = "sfpa"; //TODO: Fix to sfpC
		break;
        case 12:
        case 13:
        case 14:
        case 15:
                sfp = "sfpb"; //TODO: Fix to sfpD
		break;
#endif
	}

	const std::string chan_str( 1, 'A' + chan );
	const fs_path mb_path   = "/mboards/0";
	const fs_path prop_path = mb_path / "tx_link";

	const std::string udp_port_str = tree->access<std::string>(prop_path / std::to_string( chan ) / "port").get();

	std::stringstream udp_port_ss( udp_port_str );
	udp_port_ss >> udp_port;

	ip_addr = tree->access<std::string>( mb_path / "link" / sfp / "ip_addr").get();
}
