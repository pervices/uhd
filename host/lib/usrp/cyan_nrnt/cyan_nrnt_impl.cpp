//
// Copyright 2014-2015 Per Vices Corporation
// Copyright 2022-2024 Per Vices Corporation
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
#include <functional>
#include <boost/foreach.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/endian/conversion.hpp>

#include <numeric>

#include "cyan_nrnt_impl.hpp"
#include "cyan_nrnt_fw_common.h"

#include "uhd/transport/if_addrs.hpp"
#include "uhd/transport/udp_simple.hpp"
#include "uhd/types/stream_cmd.hpp"
#include "uhd/utils/static.hpp"

#include <uhdlib/transport/udp_common.hpp>
#ifdef HAVE_DPDK
#    include <uhdlib/transport/dpdk_simple.hpp>
#endif

namespace link_cyan_nrnt {
    const char *mtu_ref = "9000";
}

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace ph = std::placeholders;
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
static void do_samp_rate_warning_message(
    double target_rate, double actual_rate, const std::string& xx, const size_t chan)
{
    static const double max_allowed_error = 1.0; // Sps
    if (std::abs(target_rate - actual_rate) > max_allowed_error) {
        UHD_LOGGER_WARNING("MULTI_USRP")
            << boost::format(
                   "The hardware does not support the requested %s sample rate on ch %li:\n"
                   "Target sample rate: %f MSps\n"
                   "Actual sample rate: %f MSps\n")
                   % xx % chan % (target_rate / 1e6) % (actual_rate / 1e6);
    }
}

// Constants for paths in UHD side state tree
const fs_path tx_path   = CYAN_NRNT_MB_PATH / "tx";
const fs_path rx_path   = CYAN_NRNT_MB_PATH / "rx";

static std::string mb_root(const size_t mboard = 0) {
    return "/mboards/" + std::to_string(mboard);
}

static std::string rx_dsp_root(const size_t channel, const size_t mboard = 0) {
    return mb_root(mboard) + "/rx_dsps/" + std::to_string(channel);
}

static std::string rx_rf_fe_root(const size_t channel, const size_t mboard = 0) {
    auto letter = std::string(1, 'A' + channel);
    return mb_root(mboard) + "/dboards/" + letter + "/rx_frontends/Channel_" + letter;
}

std::string cyan_nrnt_impl::rx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/rx_link/" + std::to_string(channel);
}

std::string cyan_nrnt_impl::tx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/tx_link/" + std::to_string(channel);
}

std::string cyan_nrnt_impl::tx_dsp_root(const size_t channel, const size_t mboard) {
    return mb_root(mboard) + "/tx_dsps/" + std::to_string(channel);
}

static std::string tx_rf_fe_root(const size_t channel, const size_t mboard = 0) {
    auto letter = std::string(1, 'A' + channel);
    return mb_root(mboard) + "/dboards/" + letter + "/tx_frontends/Channel_" + letter;
}

//figures out the channel number from the rx stream path
static size_t pre_to_ch( const std::string & pre ) {
	char x = -1;
	if ( 1 != sscanf( pre.c_str(), "rx_%c/stream", & x) ) {
		throw value_error( "Invalid 'pre' argument '" + pre + "'" );
	}
	size_t ch = x - 'a';

	return ch;
}


// TODO: refactor so this function can be called even if this has been destructed
// NOTE: this is called via the state tree and via a bound function to rx streamers. When refactoring make sure both used are handled
void cyan_nrnt_impl::set_stream_cmd( const std::string pre, stream_cmd_t stream_cmd ) {
    const size_t ch = pre_to_ch( pre );

    rx_stream_cmd_issuer[ch].issue_stream_command(stream_cmd);
}

// Loop that polls Crimson to verify the PPS is working
void cyan_nrnt_impl::detect_pps( cyan_nrnt_impl *dev ) {

    dev->_pps_thread_running = true;
    int pps_detected;

    while (! dev->_pps_thread_should_exit) {
        dev->get_tree()->access<int>(CYAN_NRNT_TIME_PATH / "pps_detected").set(1);
        pps_detected = dev->get_tree()->access<int>(CYAN_NRNT_TIME_PATH / "pps_detected").get();

        if (pps_detected == 0) {
            std::cout << "WARNING: PPS has not been detected in the past two seconds " << std::endl;
            // Stop PPS monitoring after one failure to avoid spamming the user with the same warning message
            dev->_pps_thread_should_exit = true;
        }
#ifdef DEBUG_COUT
            std::cout << "PPS flag: " << pps_detected << std::endl;
#endif
        // Check if it should exit every 10ms for up to 2s
        for (size_t i = 0; i < 200; i++) {
            usleep(10000);
            if (dev->_pps_thread_should_exit)
                break;
        }
    }
    dev->_pps_thread_running = false;
}

void cyan_nrnt_impl::set_command_time( const std::string key, time_spec_t value ) {
    (void) key;

    _command_time = value;
}

void cyan_nrnt_impl::send_gpio_burst_req(const gpio_burst_req& req) {
    // TODO: figure out if this can be sent on any SFP port and if so use _which_time_diff_iface instead of 0
    _time_diff_iface[0]->send(boost::asio::const_buffer(&req, sizeof(req)));
}

//TODO: validate if this works
// It is okay to leave set/get user reg here because it requires use of the SFP ports (which iface doesn't have) and will never be accessed by a streamer
void cyan_nrnt_impl::set_user_reg(const std::string key, user_reg_t value) {
    (void) key;

    const uint8_t  address = value.first;
    const uint64_t setting = value.second;

    static uint64_t pins = 0x0;
    static uint64_t mask = 0x0;

    // Clearing.
    const uint64_t all = 0xFFFFFFFF;
    if(address == 0) pins &= ~(all << 0x00);
    if(address == 1) pins &= ~(all << 0x20);
    if(address == 2) mask &= ~(all << 0x00);
    if(address == 3) mask &= ~(all << 0x20);

    // Setting.
    if(address == 0) pins |= (setting << 0x00);
    if(address == 1) pins |= (setting << 0x20);
    if(address == 2) mask |= (setting << 0x00);
    if(address == 3) mask |= (setting << 0x20);

    if(address > 3)
        std::cout << "UHD: WARNING: User defined registers [4:256] not defined" << std::endl;

    // Ship if address 3 was written to.
    if(address == 3)
    {
        gpio_burst_req pkt;
        pkt.header = ((uint64_t) 0x3) << 32;
        // TODO: replace with the time set by time/clk/cmd
        pkt.tv_sec = _command_time.get_full_secs();
        pkt.tv_psec = _command_time.get_frac_secs() * 1e12;
        pkt.pins = pins;
        pkt.mask = mask;

        std::printf(
            "SHIPPING(set_user_reg):\n"
            "0x%016lX\n"
            "0x%016lX\n"
            "0x%016lX\n"
            "0x%016lX\n"
            "0x%016lX\n", pkt.header, pkt.tv_sec, pkt.tv_psec, pkt.pins, pkt.mask);

        boost::endian::native_to_big_inplace(pkt.header);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.tv_sec);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.tv_psec);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.pins);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.mask);
        #ifdef DEBUG_COUT
        std::cout << "GPIO packet size: " << sizeof(pkt) << " bytes" << std::endl;
        #endif

        send_gpio_burst_req(pkt);
    }
}

void cyan_nrnt_impl::set_time_now(const time_spec_t& time_spec, size_t mboard) {
    _tree->access<time_spec_t>(mb_root(mboard) / "time/now").set(time_spec);
    request_resync_time_diff();
}

// TODO: change the rx functions that bind to this to use their own clock_sync_shared_info
uhd::time_spec_t cyan_nrnt_impl::get_time_now() {
    // Waits for clock to be stable before getting time
    if(_bm_thread_running) {
        device_clock_sync_info->wait_for_sync();
        double diff = device_clock_sync_info->get_time_diff();
        return uhd::get_system_time() + diff;
    // If clock sync thread is not running reset the time diff pid and use the initial offset
    // Will get the time but without taking into account network latency (which would require the clock sync thread)
    } else {
        reset_time_diff_pid();
        return uhd::get_system_time() - _time_diff_pidc->get_offset();
    }
}

void cyan_nrnt_impl::set_properties_from_addr() {

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

            _mbc.iface->set_string( key, expected_string );

            std::string actual_string = _mbc.iface->get_string( key );
			if ( actual_string != expected_string ) {
				UHD_LOGGER_ERROR(CYAN_NRNT_DEBUG_NAME_C "_IMPL")
					<< __func__ << "(): "
					<< "Setting " CYAN_NRNT_DEBUG_NAME_S "  property failed: "
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
device_addrs_t cyan_nrnt_impl::cyan_nrnt_find_with_addr(const device_addr_t &hint)
{
    // temporarily make a UDP device only to look for devices
    // loop for all the available ports, if none are available, that means all 8 are open already
    udp_simple::sptr comm = udp_simple::make_broadcast(
        hint["addr"], BOOST_STRINGIZE(CYAN_NRNT_FW_COMMS_UDP_PORT));

    //send request for echo
    comm->send(asio::buffer("1,get,fpga/about/name", sizeof("1,get,fpga/about/name")));

    // List is Crimsons connected
    device_addrs_t cyan_nrnt_addrs;
    char buff[CYAN_NRNT_FW_COMMS_MTU] = {};

    // Checks for all connected Cyan NRNT
    for(
		float to = 0.2;
    	comm->recv(asio::buffer(buff), to);
    	to = 0.05
    ) {
        // parse the return buffer for the device type (from fpga/about/name)
        std::vector<std::string> tokens;
        tng_csv_parse(tokens, buff, ',');

        // Checks if type matches Crimson name matches
        if (tokens.size() < 3) {
            continue;
        }
        if (tokens[1].c_str()[0] == CMD_ERROR) {
            continue;
        }
        if (tokens[2] != "cyan_nrnt") {
            continue;
        }

        device_addr_t new_addr;
        new_addr["type"]    = tokens[2];
        new_addr["addr"]    = comm->get_recv_addr();
        new_addr["name"]    = "";
        cyan_nrnt_addrs.push_back(new_addr);
    }

    // List of devices that match the filter
    device_addrs_t matching_addr;

    // Gets the Serial number for all connected Cyans found in the previous loop, and adds them to the return list if all required parameters match filters
    for(auto& addr : cyan_nrnt_addrs) {
        udp_simple::sptr comm = udp_simple::make_connected(
        addr["addr"], BOOST_STRINGIZE(CYAN_NRNT_FW_COMMS_UDP_PORT));


        size_t bytes_sent = comm->send(asio::buffer("1,get,fpga/about/serial", sizeof("1,get,fpga/about/serial")));

        if(bytes_sent != sizeof("1,get,fpga/about/serial")) {
            std::cerr << "Error when sending serial number request" << std::endl;
            continue;
        }

        comm->recv(asio::buffer(buff), 5);

        // parse the return buffer for the device serial (from fpga/about/serial)
        std::vector<std::string> tokens;
        tng_csv_parse(tokens, buff, ',');
        if (tokens.size() < 3) {
            UHD_LOGGER_ERROR(CYAN_NRNT_DEBUG_NAME_C " failed to get serial number");
            addr["serial"]  = "0000000000000000";
        }
        else if (tokens[1].c_str()[0] == CMD_ERROR) {
            UHD_LOGGER_ERROR(CYAN_NRNT_DEBUG_NAME_C " failed to get serial number");
            addr["serial"]  = "0000000000000000";
        } else {
            addr["serial"] = tokens[2];
        }

        //filter the discovered device below by matching optional keys
        if (
            (not hint.has_key("name")    or hint["name"]    == addr["name"])    and
            (not hint.has_key("serial")  or hint["serial"]  == addr["serial"])  and
            (not hint.has_key("product") or hint["product"] == addr["product"])
        ) {
            matching_addr.push_back(addr);
        }
    }

    return matching_addr;
}

// This is the core find function that will be called when uhd:device find() is called because this is registered
device_addrs_t cyan_nrnt_impl::cyan_nrnt_find(const device_addr_t &hint_)
{
    //handle the multi-device discovery
    device_addrs_t hints = separate_device_addr(hint_);
    if (hints.size() > 1)
    {
        device_addrs_t found_devices;
        std::string error_msg;
        BOOST_FOREACH(const device_addr_t &hint_i, hints)
        {
            device_addrs_t found_devices_i = cyan_nrnt_find(hint_i);
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

    if (hint.has_key("type") and hint["type"] != "cyan_nrnt") {
        return addrs;
    }

    //use the address given
    if (hint.has_key("addr"))
    {
        device_addrs_t reply_addrs;
        try
        {
            reply_addrs = cyan_nrnt_find_with_addr(hint);
        }
        catch(const std::exception &ex)
        {
            UHD_LOGGER_ERROR(CYAN_NRNT_DEBUG_NAME_C) << "CYAN_NRNT Network discovery error " << ex.what() << std::endl;
        }
        catch(...)
        {
            UHD_LOGGER_ERROR(CYAN_NRNT_DEBUG_NAME_C) << "CYAN_NRNT Network discovery unknown error " << std::endl;
        }
        BOOST_FOREACH(const device_addr_t &reply_addr, reply_addrs)
        {
            device_addrs_t new_addrs = cyan_nrnt_find_with_addr(reply_addr);
            addrs.insert(addrs.end(), new_addrs.begin(), new_addrs.end());
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
            device_addrs_t new_addrs = cyan_nrnt_find(new_hint);
            addrs.insert(addrs.end(), new_addrs.begin(), new_addrs.end());
        }
    }

    return addrs;
}

/**
 * Buffer Management / Time Diff
 */

// SoB: Time Diff (Time Diff mechanism is used to get an accurate estimate of Crimson's absolute time)
static constexpr double tick_period_ns = 1.0 / CYAN_NRNT_TICK_RATE * 1e9;
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

void cyan_nrnt_impl::time_diff_send( const uhd::time_spec_t & crimson_now ) {

	time_diff_req pkt;

	// Input to Process (includes feedback from PID Controller)
	make_time_diff_packet(
		pkt,
		crimson_now
	);

    if (_which_time_diff_iface >= NUMBER_OF_XG_CONTROL_INTF) {
        throw runtime_error( "XG Control interface offset out of bound!" );
    }
	_time_diff_iface[_which_time_diff_iface]->send( boost::asio::const_buffer( &pkt, sizeof( pkt ) ) );
}

bool cyan_nrnt_impl::time_diff_recv( time_diff_resp & tdr ) {

	size_t r;

    if (_which_time_diff_iface >= NUMBER_OF_XG_CONTROL_INTF) {
        throw runtime_error( "XG Control interface offset out of bound!" );
    }
	r = _time_diff_iface[_which_time_diff_iface]->recv( boost::asio::mutable_buffer( & tdr, sizeof( tdr ) ) );

	if ( 0 == r ) {
		return false;
	}

	boost::endian::big_to_native_inplace( tdr.tv_sec );
	boost::endian::big_to_native_inplace( tdr.tv_tick );

	return true;
}

void cyan_nrnt_impl::reset_time_diff_pid() {
    // Get mutex before getting time incase it needs to wait for the mutex
    _sfp_control_mutex[_which_time_diff_iface]->lock();

    auto reset_now = uhd::get_system_time();
    struct time_diff_resp reset_tdr;

    time_diff_send( reset_now );
    time_diff_recv( reset_tdr );
    _sfp_control_mutex[_which_time_diff_iface]->unlock();

    double new_offset = (double) reset_tdr.tv_sec + (double)ticks_to_nsecs( reset_tdr.tv_tick ) / 1e9;
    _time_diff_pidc->reset(reset_now, new_offset);
}

/// SoB Time Diff: feed the time diff error back into out control system
void cyan_nrnt_impl::time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now ) {

	static const double sp = 0.0;

	double pv = (double) tdr.tv_sec + (double)ticks_to_nsecs( tdr.tv_tick ) / 1e9;

	double cv = _time_diff_pidc->update_control_variable( sp, pv, now );

    bool reset_advised = false;

    bool time_diff_converged = _time_diff_pidc->is_converged( now, &reset_advised );

    if(reset_advised) {
        reset_time_diff_pid();
    }

    // For SoB, record the instantaneous time difference + compensation
    if (time_diff_converged ) {
        device_clock_sync_info->set_time_diff( cv );
    }
}

//performs clock synchronization
void cyan_nrnt_impl::start_bm() {

    //checks if the current task is excempt from need clock synchronization
	if ( ! _bm_thread_needed ) {
		return;
	}

	//checks if clock synchronization is already being done
	if ( ! _bm_thread_running ) {

		_bm_thread_should_exit = false;
        //starts the thread that synchronizes the clocks
        request_resync_time_diff();
        _bm_thread_running = true;
		_bm_thread = std::thread( bm_thread_fn, this );

        //Note: anything relying on this will require waiting time_diff_converged()
	}
}

//stops clock synchronization
void cyan_nrnt_impl::stop_bm() {

	if ( _bm_thread_running ) {

		_bm_thread_should_exit = true;
		_bm_thread.join();

	}
}

void cyan_nrnt_impl::start_pps_dtc() {

    if ( ! _pps_thread_needed ) {
        return;
    }

    if ( ! _pps_thread_running ) {
        _pps_thread_should_exit = false;
        _pps_thread = std::thread( detect_pps, this );
    }
}

void cyan_nrnt_impl::stop_pps_dtc() {

    if ( ! _pps_thread_needed ) {
        return;
    }

    if(_pps_thread.joinable()) {
        _pps_thread_should_exit = true;
        _pps_thread.join();
    }
}

// Synchronizes clocks between the host and device
// This function should be run in its own thread
// When calling it verify that it is not already running (_bm_thread_running)
void cyan_nrnt_impl::bm_thread_fn( cyan_nrnt_impl *dev ) {

    // Flag so that we only print the error message for failed recv once
    bool dropped_recv_message_printed = false;
    
	const uhd::time_spec_t T( 1.0 / (double) CYAN_NRNT_UPDATE_PER_SEC );
	uhd::time_spec_t now, then, dt;
    //the predicted time on the unit
	uhd::time_spec_t crimson_now;
	struct timespec req, rem;

	double time_diff;

	struct time_diff_resp tdr;

	//Get offset
    dev->_sfp_control_mutex[dev->get_which_time_diff_iface()]->lock();
	now = uhd::get_system_time();
	dev->time_diff_send( now );
	dev->time_diff_recv( tdr );
    dev->_sfp_control_mutex[dev->get_which_time_diff_iface()]->unlock();
    dev->_time_diff_pidc->set_offset((double) tdr.tv_sec + (double)ticks_to_nsecs( tdr.tv_tick ) / 1e9);

	for(
		now = uhd::get_system_time(),
			then = now + T
			;

		! dev->_bm_thread_should_exit
			;

		then += T,
			now = uhd::get_system_time()
	) {
        if(dev->device_clock_sync_info->is_resync_requested()) {
            // Record that the resync request has been ackcknowledged (also sets it as desynced)
            dev->device_clock_sync_info->resync_acknowledge();
            // Reset PID to clear old values
            dev->reset_time_diff_pid();
        }

		dt = then - now;
		if ( dt > 0.0 ) {
			req.tv_sec = dt.get_full_secs();
			req.tv_nsec = dt.get_frac_secs() * 1e9;
			nanosleep( &req, &rem );
		} else {
            continue;
        }

        time_diff = dev->_time_diff_pidc->get_control_variable();
        dev->_sfp_control_mutex[dev->get_which_time_diff_iface()]->lock();
        now = uhd::get_system_time();
        crimson_now = now + time_diff;

        // Send the predicted time
        dev->time_diff_send( crimson_now );
        // Get the difference between the predicted and real time
        bool reply_good =  dev->time_diff_recv( tdr );

        // Unlock sfp control mutex here
        // It is no longer needed, and having it will deadlock if time_diff_process triggers a reset
        dev->_sfp_control_mutex[0]->unlock();
        if (reply_good) {
            dev->time_diff_process( tdr, now );
         } else if (!dropped_recv_message_printed && dev->clock_sync_desired) {
             UHD_LOG_ERROR(CYAN_NRNT_DEBUG_NAME_C, "Failed to receive packet used by clock synchronization");
             dropped_recv_message_printed = true;
         }
	}
	dev->_bm_thread_running = false;
}

/***********************************************************************
 * Make
 **********************************************************************/
// Returns a pointer to the SDR device, casted to the UHD base class
static device::sptr cyan_nrnt_make(const device_addr_t &device_addr)
{
    bool use_dpdk;
    if(device_addr.has_key("use_dpdk")) {
        if(device_addr["use_dpdk"] == "" || device_addr["use_dpdk"] == "true") {
#ifdef HAVE_DPDK
            use_dpdk = true;
#else
            UHD_LOG_WARNING("DPDK", "Detected use_dpdk argument, but DPDK support not built in.");
            use_dpdk = false;
#endif
        } else {
            use_dpdk = false;
        }
    } else {
        use_dpdk = false;
    }
    return device::sptr(new cyan_nrnt_impl(device_addr, use_dpdk));
}

// This is the core function that registers itself with uhd::device base class. The base device class
// will have a reference to all the registered devices and upon device find/make it will loop through
// all the registered devices' find and make functions.
UHD_STATIC_BLOCK(register_cyan_nrnt_device)
{
	set_log_level( uhd::log::severity_level::info );
    device::register_device(&cyan_nrnt_impl::cyan_nrnt_find, &cyan_nrnt_make, device::USRP);
}

/***********************************************************************
 * Structors
 **********************************************************************/
// .set in TREE_CREATE_ sets the value of the property retrieved by calling get without having a publisher assigned
// The value for set is irrelevant for RW and RO since a publisher is always assigned, and irrlevant for WO
// Macro to create the tree, all properties created with this are R/W properties
#define TREE_CREATE_RW(PATH, PROP, TYPE, HANDLER)\
    do { _tree->create<TYPE> (PATH)\
        .add_desired_subscriber(std::bind(&pv_iface::set_ ## HANDLER, _mbc.iface, (PROP), ph::_1))\
        .set_publisher(std::bind(&pv_iface::get_ ## HANDLER, _mbc.iface, (PROP) ));\
    } while(0)

// Macro to create the tree, all properties created with this are RO properties
#define TREE_CREATE_RO(PATH, PROP, TYPE, HANDLER)\
    do { _tree->create<TYPE> (PATH)\
        .set_publisher(std::bind(&pv_iface::get_ ## HANDLER, _mbc.iface, (PROP) ));\
    } while(0)

// Macro to create the tree, all properties created with this are WO properties
#define TREE_CREATE_WO(PATH, PROP, TYPE, HANDLER)\
    do { _tree->create<TYPE> (PATH)\
        .add_desired_subscriber(std::bind(&pv_iface::set_ ## HANDLER, _mbc.iface, (PROP), ph::_1));\
    } while(0)

// Macro to create the tree, all properties created with this are static
#define TREE_CREATE_ST(PATH, TYPE, VAL) 	( _tree->create<TYPE>(PATH).set(VAL) )

cyan_nrnt_impl::cyan_nrnt_impl(const device_addr_t &_device_addr, bool use_dpdk, double freq_range_stop)
:
	device_addr( _device_addr ),
	// Put _time_diff_pidc on their own cache line to avoid false sharing
	_time_diff_pidc((uhd::pidc*) aligned_alloc(CACHE_LINE_SIZE, padded_pidc_tcl_size)),
	_bm_thread_needed( true ),
	_bm_thread_running( false ),
	_bm_thread_should_exit( false ),
	_pps_thread_running( false ),
	_pps_thread_should_exit( false ),
	_command_time( 0.0 ),
    _freq_range_stop(freq_range_stop),
    _use_dpdk(use_dpdk)
{
    if(_use_dpdk) {
        std::cout << "DPDK implementation in progress" << std::endl;
    }

    _type = device::CYAN_NRNT;
    device_addr = _device_addr;

    // Initialize the mutexes to control access to the SFP ports
    _sfp_control_mutex.reserve(NUMBER_OF_XG_CONTROL_INTF);
    for(size_t n = 0; n < NUMBER_OF_XG_CONTROL_INTF; n++) {
        _sfp_control_mutex.emplace_back(new std::mutex());
    }

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

    // Makes the UDP comm connection
    _mbc.iface = pv_iface::make(
		udp_simple::make_connected(
			_device_addr["addr"],
			BOOST_STRINGIZE( CYAN_NRNT_FW_COMMS_UDP_PORT )
		)
    );

    // TODO check if locked already
    // TODO lock the Crimson device to this process, this will prevent the Crimson device being used by another program

    // Create the file tree of properties.
    // Cyan NrNt only has support for one mother board, and the RF chains will show up individually as daughter boards.
    // All the initial settings are read from the current status of the board.
    _tree = uhd::property_tree::make();

    // The state tree functions do not have 64 bit ints, so the properties are called using doubles then converted
    // The buffer size in number of samples
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "system/get_max_buffer_level", "system/get_max_buffer_level", double, double);
    max_buffer_level = (int64_t) (_tree->access<double>(CYAN_NRNT_MB_PATH / "system/get_max_buffer_level").get());
    // The number to multiply get buffer level requests by to get the actual buffer level in number of samples
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "system/get_buffer_level_multiple", "system/get_buffer_level_multiple", double, double);
    buffer_level_multiple = (int64_t) (_tree->access<double>(CYAN_NRNT_MB_PATH / "system/get_buffer_level_multiple").get());

    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "system/nsamps_multiple_rx", "system/nsamps_multiple_rx", int, int);
    nsamps_multiple_rx = (_tree->access<int>(CYAN_NRNT_MB_PATH / "system/nsamps_multiple_rx").get());

    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "system/nsamps_multiple_tx", "system/nsamps_multiple_tx", int, int);

    if (not device_addr.has_key("send_buff_size")){
        //The buffer should be the size of the SRAM on the device,
        //because we will never commit more than the SRAM can hold.
        device_addr["send_buff_size"] = boost::lexical_cast<std::string>( (size_t) (max_buffer_level * sizeof( std::complex<int16_t> ) * ((double)(MAX_ETHERNET_MTU+1)/(CYAN_NRNT_MAX_MTU-CYAN_NRNT_UDP_OVERHEAD))) );
    }

    TREE_CREATE_RO(CYAN_NRNT_MB_PATH / "system/num_rx", "system/num_rx", int, int);
    TREE_CREATE_RO(CYAN_NRNT_MB_PATH / "system/num_tx", "system/num_tx", int, int);
    num_rx_channels = (size_t) (_tree->access<int>(CYAN_NRNT_MB_PATH / "system/num_rx").get());
    is_num_rx_channels_set = true;
    num_tx_channels = (size_t) (_tree->access<int>(CYAN_NRNT_MB_PATH / "system/num_tx").get());
    is_num_tx_channels_set = true;

    _mbc.rx_streamers.resize( num_rx_channels );
    _mbc.tx_streamers.resize( num_tx_channels );

    rx_gain_is_set.resize(num_rx_channels, false);
    last_set_rx_band.resize(num_rx_channels, -1);
    rx_rfe_rate_cache.resize(num_rx_channels, 0);
    tx_gain_is_set.resize(num_tx_channels, false);
    last_set_tx_band.resize(num_tx_channels, -1);
    
    // Checks if the tx channel is baseband only
    is_tx_baseband_only.resize(num_tx_channels, false);
    for(size_t ch = 0; ch < num_tx_channels; ch++) {
        TREE_CREATE_RO(tx_path / ch / "variant/is_baseband_only", "tx_"+ std::string(1, (char)(ch + 'a')) +"/about/variant/is_baseband_only", int, int);
        try {
            is_tx_baseband_only[ch] = _tree->access<int>(tx_path / ch / "variant/is_baseband_only").get();
        } catch(...) {
            // If unable to check assume the server from before baseband only was created
            is_tx_baseband_only[ch] = false;
        }
    }

    TREE_CREATE_RO(CYAN_NRNT_MB_PATH / "system/max_rate", "system/max_rate", double, double);
    max_sample_rate = (_tree->access<double>(CYAN_NRNT_MB_PATH / "system/max_rate").get());

    // NOTE: otw_rx is ever made changable during runtime ensure rx_stream_cmd_issuer is updated
    TREE_CREATE_RO(CYAN_NRNT_MB_PATH / "system/otw_rx", "system/otw_rx", int, int);
    otw_rx = (_tree->access<int>(CYAN_NRNT_MB_PATH / "system/otw_rx").get());
    otw_rx_s = "sc" + std::to_string(otw_rx);

    TREE_CREATE_RO(CYAN_NRNT_MB_PATH / "system/otw_tx", "system/otw_tx", int, int);
    otw_tx = (_tree->access<int>(CYAN_NRNT_MB_PATH / "system/otw_tx").get());
    otw_tx_s = "sc" + std::to_string(otw_tx);

    TREE_CREATE_RO(CYAN_NRNT_MB_PATH / "system/flags/USE_3G_AS_1G", "system/flags/USE_3G_AS_1G", int, int);
    flag_use_3g_as_1g = (_tree->access<int>(CYAN_NRNT_MB_PATH / "system/flags/USE_3G_AS_1G").get());

    //Initializes the vectors contain caches of constant data
    is_tx_sfp_cached.resize(num_tx_channels, false);
    tx_sfp_cache.resize(num_tx_channels);
    is_tx_ip_cached.resize(num_tx_channels,false);
    tx_ip_cache.resize(num_tx_channels);
    is_tx_fc_cached.resize(num_tx_channels, false);
    tx_fc_cache.resize(num_tx_channels);
    is_tx_udp_port_cached.resize(num_tx_channels, false);
    tx_udp_port_cache.resize(num_tx_channels);
    tx_sfp_throughput_used.resize(num_tx_channels, 0);
    tx_channel_in_use = std::shared_ptr<std::vector<bool>>(new std::vector<bool>(num_tx_channels, false));

    is_rx_sfp_cached.resize(num_rx_channels, false);
    rx_sfp_cache.resize(num_rx_channels);
    rx_sfp_throughput_used.resize(num_rx_channels, 0);
    rx_channel_in_use = std::shared_ptr<std::vector<bool>>(new std::vector<bool>(num_rx_channels, false));

    static const std::vector<std::string> time_sources = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(CYAN_NRNT_MB_PATH / "time_source" / "options").set(time_sources);

    static const std::vector<double> external_freq_options = boost::assign::list_of(10e6);
    _tree->create<std::vector<double> >(CYAN_NRNT_MB_PATH / "clock_source" / "external" / "freq" / "options");
    static const std::vector<std::string> clock_source_options = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(CYAN_NRNT_MB_PATH / "clock_source" / "options").set(clock_source_options);

    TREE_CREATE_ST("/name", std::string, CYAN_NRNT_DEBUG_NAME_S " Device");

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////

    std::vector<size_t> default_rx_map(num_rx_channels);
    std::iota(default_rx_map.begin(), default_rx_map.end(), 0);
    std::vector<size_t> default_tx_map(num_tx_channels);
    std::iota(default_tx_map.begin(), default_tx_map.end(), 0);


    _tree->create<std::vector<size_t> >(CYAN_NRNT_MB_PATH / "rx_chan_dsp_mapping").set(default_rx_map);
    _tree->create<std::vector<size_t> >(CYAN_NRNT_MB_PATH / "tx_chan_dsp_mapping").set(default_tx_map);
    // TODO: make rx_subdev_spec and tx_subdev_spec read only and remove their respective update functions
    _tree->create<subdev_spec_t>(CYAN_NRNT_MB_PATH / "rx_subdev_spec").add_coerced_subscriber(std::bind(&cyan_nrnt_impl::update_rx_subdev_spec, this, ph::_1));
    _tree->create<subdev_spec_t>(CYAN_NRNT_MB_PATH / "tx_subdev_spec").add_coerced_subscriber(std::bind(&cyan_nrnt_impl::update_tx_subdev_spec, this, ph::_1));

    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "vendor", std::string, "Per Vices");
    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "name",   std::string, "FPGA Board");
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "id",         "fpga/about/id",     std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "serial",     "fpga/about/serial", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "server_version", "fpga/about/server_ver", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "fw_version", "fpga/about/fw_ver", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "hw_version", "fpga/about/hw_ver", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "sw_version", "fpga/about/sw_ver", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/backplane_pinout", "fpga/about/imgparam/backplane_pinout", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/ddr_used", "fpga/about/imgparam/ddr_used", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/hps_only", "fpga/about/imgparam/hps_only", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/num_rx", "fpga/about/imgparam/num_rx", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/num_tx", "fpga/about/imgparam/num_tx", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/rate", "fpga/about/imgparam/rate", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "imgparam/rtm", "fpga/about/imgparam/rtm", int, int);
    TREE_CREATE_WO(CYAN_NRNT_MB_PATH / "blink", "fpga/board/led", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "temp", "fpga/board/temp", std::string, string);
    // TODO: investigate/add comments to explain what user/regs does
    // Unlike the other properties, this access functions belong to this class
    // NOTE: ensure this property is not accessed after this is destructed
    _tree->create<user_reg_t> (CYAN_NRNT_MB_PATH / "user/regs")
        .add_desired_subscriber(std::bind(&cyan_nrnt_impl::set_user_reg, this, ("fpga/user/regs"), ph::_1));

    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "trigger/sma_dir", "fpga/trigger/sma_dir",  std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "trigger/sma_pol", "fpga/trigger/sma_pol",  std::string, string);

    // String is used because this is a 64 bit number and won't fit in int
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "gps_time", "fpga/board/gps_time", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "gps_frac_time", "fpga/board/gps_frac_time", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "gps_sync_time", "fpga/board/gps_sync_time", int, int);

    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "fpga/board/flow_control/sfpa_port", "fpga/board/flow_control/sfpa_port", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "fpga/board/flow_control/sfpb_port", "fpga/board/flow_control/sfpb_port", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "fpga/board/flow_control/sfpc_port", "fpga/board/flow_control/sfpc_port", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "fpga/board/flow_control/sfpd_port", "fpga/board/flow_control/sfpd_port", int, int);

    TREE_CREATE_ST(CYAN_NRNT_TIME_PATH / "name", std::string, "Time Board");
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "id",         "time/about/id",     std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "serial",     "time/about/serial", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "fw_version", "time/about/fw_ver", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "sw_version", "time/about/sw_ver", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "eeprom", "time/about/eeprom", std::string, string);

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
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link_max_rate", "fpga/link/rate", double, double);

    // SFP settings
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpa" / "ip_addr",  "fpga/link/sfpa/ip_addr", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpa" / "pay_len", "fpga/link/sfpa/pay_len", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpb" / "ip_addr",     "fpga/link/sfpb/ip_addr", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpb" / "pay_len", "fpga/link/sfpb/pay_len", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpc" / "ip_addr",  "fpga/link/sfpc/ip_addr", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpc" / "pay_len", "fpga/link/sfpc/pay_len", int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpd" / "ip_addr",     "fpga/link/sfpd/ip_addr", std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "link" / "sfpd" / "pay_len", "fpga/link/sfpd/pay_len", int, int);

    _which_time_diff_iface = -1;
    for (int i = 0; i < NUMBER_OF_XG_CONTROL_INTF; i++) {
        std::string xg_intf = std::string(1, char('a' + i));
        int sfp_port = _tree->access<int>( CYAN_NRNT_MB_PATH / "fpga/board/flow_control/sfp" + xg_intf + "_port" ).get();
        std::string time_diff_ip = _tree->access<std::string>( CYAN_NRNT_MB_PATH / "link" / "sfp" + xg_intf / "ip_addr" ).get();
        std::string time_diff_port = std::to_string( sfp_port );
        _time_diff_iface.push_back(udp_simple::make_connected( time_diff_ip, time_diff_port ));

        // Checks if this iface is working
        bool iface_good = ping_check("sfp" + xg_intf, time_diff_ip);
        // Set the iface used by time diffs to the first working one
        if(iface_good && _which_time_diff_iface < 0) {
            _which_time_diff_iface = i;
        }
    }

    if(_which_time_diff_iface < 0) {
        // TODO: only print this warning when using regular streaming
        UHD_LOG_WARNING(CYAN_NRNT_DEBUG_NAME_C, "Unable to ping any SFP ports. Only force stream will work. Normal streaming will not");
        // Attempt to use SFPA for time diff if unable to reach any
        // TODO: Don't run time diffs if unable to ping any SFP ports
        _which_time_diff_iface = 0;
    }

    // This is the master clock rate
    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "tick_rate", double, CYAN_NRNT_TICK_RATE);

    // TODO: see if time/clk/cmd is used anywhere, and if not remove it
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "cmd", "time/clk/cmd",      time_spec_t, time_spec);
    // This line will get time spec, the time diff port must be initialized first
    // Call request_resync_time_diff anytime this is set
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "now", "time/clk/cur_time", time_spec_t, time_spec);
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "pps", "time/clk/pps",    time_spec_t, time_spec);
    TREE_CREATE_RW(CYAN_NRNT_TIME_PATH / "pps_detected", "time/clk/pps_detected",    int,         int);
    _pps_thread_needed = true;
    try {
        // Attempt to read pps_detected
        // If success the the pps monitoring loop should be run
        // If failure then pps monitoring is not implemented on the server and the loop should not be run
        _tree->access<int>(CYAN_NRNT_TIME_PATH / "pps_detected").get();
    } catch(uhd::lookup_error &e) {
        _pps_thread_needed = false;
    }
    _pps_thread_running = false;
    _pps_thread_should_exit = false;

    // if the "serial" property is not added, then multi_usrp->get_rx_info() crashes libuhd
    // unfortunately, we cannot yet call get_mboard_eeprom().
    mboard_eeprom_t temp;
    temp["name"]     = "FPGA Board";
    temp["vendor"]   = "Per Vices";
    temp["serial"]   = "";
    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "eeprom", mboard_eeprom_t, temp);

    // This property chooses internal or external time (usually pps) source
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "time_source"  / "value",  	"time/source/set_time_source",  	std::string, string);
    // Sets whether to use internal or external clock source
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "time_source"  / "freq",  	    "time/source/freq_mhz",  	int, int);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "clock_source" / "value",      "time/source/ref",	std::string, string);
    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "clock_source" / "external",	"time/source/ref",	std::string, string);
    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "clock_source" / "external" / "value", double, CYAN_NRNT_EXT_CLK_RATE);
    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "clock_source" / "output", bool, true);
    TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "time_source"  / "output", bool, true);

    TREE_CREATE_RW(CYAN_NRNT_MB_PATH / "sensors" / "ref_locked", "time/status/lmk_lockdetect", sensor_value_t, sensor_value );

    // No GPSDO support on Crimson
    // TREE_CREATE_ST(CYAN_NRNT_MB_PATH / "sensors" / "ref_locked", sensor_value_t, sensor_value_t("NA", "0", "NA"));

    // loop for all RX chains
    for( size_t dspno = 0; dspno < num_rx_channels; dspno++ ) {
		std::string lc_num  = boost::lexical_cast<std::string>((char)(dspno + 'a'));
		std::string num     = boost::lexical_cast<std::string>((char)(dspno + 'A'));
		std::string chan    = "Channel_" + num;

		const fs_path rx_codec_path = CYAN_NRNT_MB_PATH / "rx_codecs" / num;
		const fs_path rx_fe_path    = CYAN_NRNT_MB_PATH / "dboards" / num / "rx_frontends" / chan;
		const fs_path db_path       = CYAN_NRNT_MB_PATH / "dboards" / num;
		const fs_path rx_dsp_path   = CYAN_NRNT_MB_PATH / "rx_dsps" / dspno;
		const fs_path rx_link_path  = CYAN_NRNT_MB_PATH / "rx_link" / dspno;

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

        // About information
		TREE_CREATE_RW(rx_path / dspno / "fw_version", "rx_"+lc_num+"/about/fw_ver", std::string, string);
		TREE_CREATE_RW(rx_path / dspno / "hw_version", "rx_"+lc_num+"/about/hw_ver", std::string, string);
		TREE_CREATE_RW(rx_path / dspno / "sw_version", "rx_"+lc_num+"/about/sw_ver", std::string, string);
		TREE_CREATE_RW(rx_path / dspno / "eeprom", "rx_"+lc_num+"/about/eeprom", std::string, string);

        TREE_CREATE_RW(rx_path / dspno / "about/rfe_rate", "rx_"+lc_num+"/about/rfe_rate", double, double);
        try {
            rx_rfe_rate_cache[dspno] = _tree->access<double>(rx_path / dspno / "about/rfe_rate").get();
        } catch(uhd::lookup_error &e) {
            // rx_rfe_rate_cache is only used when using 3G backplane and either 1G or 3G rfe boards
            // If using an old server assume that 3G backplane means 3G rfe boards
            rx_rfe_rate_cache[dspno] = 3000000000;
        }

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
		TREE_CREATE_ST(rx_fe_path / "bandwidth" / "value", double, (double) CYAN_NRNT_BW_FULL );
		TREE_CREATE_ST(rx_fe_path / "bandwidth" / "range", meta_range_t, meta_range_t( (double) CYAN_NRNT_BW_FULL, (double) CYAN_NRNT_BW_FULL ) );

		TREE_CREATE_ST(rx_fe_path / "freq", meta_range_t,
			meta_range_t((double) CYAN_NRNT_FREQ_RANGE_START, (double) _freq_range_stop, (double) CYAN_NRNT_FREQ_RANGE_STEP));

		TREE_CREATE_ST(rx_fe_path / "dc_offset" / "enable", bool, false);
		TREE_CREATE_ST(rx_fe_path / "dc_offset" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));
		TREE_CREATE_ST(rx_fe_path / "iq_balance" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));

		TREE_CREATE_RW(rx_fe_path / "connection",  "rx_"+lc_num+"/link/iface", std::string, string);

		TREE_CREATE_ST(rx_fe_path / "use_lo_offset", bool, true );

		TREE_CREATE_ST(rx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t((double) CYAN_NRNT_FREQ_RANGE_START, (double) _freq_range_stop, (double) CYAN_NRNT_FREQ_RANGE_STEP));

		TREE_CREATE_RW(rx_fe_path / "freq"  / "value", "rx_"+lc_num+"/rf/freq/val" , double, double);
		TREE_CREATE_ST(rx_fe_path / "gains", std::string, "gain" );
		TREE_CREATE_RW(rx_fe_path / "gain"  / "value", "rx_"+lc_num+"/rf/gain/val" , double, double);
        // Gain range for all elements in the current band
        TREE_CREATE_RW(rx_fe_path / "gain" / "range", "rx_"+lc_num+"/rf/gain/range",  meta_range_t, meta_range);
        TREE_CREATE_RW(rx_fe_path / "gain"  / "adc_digital", "rx_"+lc_num+"/rf/gain/adc_digital" , double, double);
		TREE_CREATE_RW(rx_fe_path / "atten" / "value", "rx_"+lc_num+"/rf/atten/val", double, double);

		// RF band
		TREE_CREATE_RW(rx_fe_path / "freq" / "band", "rx_"+lc_num+"/rf/freq/band", int, int);

		// RF receiver LNA
		TREE_CREATE_RW(rx_fe_path / "freq"  / "lna", "rx_"+lc_num+"/rf/freq/lna" , int, int);

        // Rx rf pll lock detect
        TREE_CREATE_RW(rx_fe_path / "sensors" / "rfpll_lock", "rx_"+lc_num+"/status/rfpll_lock", sensor_value_t, sensor_value );


		// these are phony properties for Crimson
		TREE_CREATE_ST(db_path / "rx_eeprom",  dboard_eeprom_t, dboard_eeprom_t());
		TREE_CREATE_ST(db_path / "gdb_eeprom", dboard_eeprom_t, dboard_eeprom_t());

		// DSPs
		TREE_CREATE_ST(rx_dsp_path / "rate" / "range", meta_range_t,
			meta_range_t((double) CYAN_NRNT_RATE_RANGE_START, (double) CYAN_NRNT_RATE_RANGE_STOP_FULL, (double) CYAN_NRNT_RATE_RANGE_STEP));
		TREE_CREATE_ST(rx_dsp_path / "freq" / "range", meta_range_t,
			meta_range_t((double) CYAN_NRNT_DSP_FREQ_RANGE_START_FULL, (double) CYAN_NRNT_DSP_FREQ_RANGE_STOP_FULL, (double) CYAN_NRNT_DSP_FREQ_RANGE_STEP));
		TREE_CREATE_ST(rx_dsp_path / "bw" / "range",   meta_range_t,
			meta_range_t((double) CYAN_NRNT_DSP_BW_START, (double) CYAN_NRNT_DSP_BW_STOP_FULL, (double) CYAN_NRNT_DSP_BW_STEPSIZE));

        TREE_CREATE_RW(rx_dsp_path / "rate" / "value", "rx_"+lc_num+"/dsp/rate", double, double);

		TREE_CREATE_RW(rx_dsp_path / "freq" / "value", "rx_"+lc_num+"/dsp/nco_adj", double, double);
		TREE_CREATE_RW(rx_dsp_path / "bw" / "value",   "rx_"+lc_num+"/dsp/rate",    double, double);

        // Used to issue an rx stream command
        // WO property
        _tree->create<uhd::stream_cmd_t> ( rx_dsp_path / "stream_cmd" )
            .add_desired_subscriber(std::bind(&cyan_nrnt_impl::set_stream_cmd, this, ("rx_"+lc_num+"/stream_cmd"), ph::_1));

		TREE_CREATE_RW(rx_dsp_path / "nco", "rx_"+lc_num+"/dsp/nco_adj", double, double);

        TREE_CREATE_RW(rx_path / dspno / "jesd" / "status", "rx_"+lc_num+"/jesd/status", std::string, string);

        TREE_CREATE_RW(rx_path / dspno / "status" / "lna", "rx_"+lc_num+"/status/lna", std::string, string);

		// Link settings
		TREE_CREATE_RW(rx_link_path / "vita_en", "rx_"+lc_num+"/link/vita_en", std::string, string);
        TREE_CREATE_RW(rx_link_path / "endian_swap", "rx_"+lc_num+"/link/endian_swap", int, int);
		TREE_CREATE_RW(rx_link_path / "ip_dest", "rx_"+lc_num+"/link/ip_dest", std::string, string);
		TREE_CREATE_RW(rx_link_path / "port",    "rx_"+lc_num+"/link/port",    std::string, string);
		TREE_CREATE_RW(rx_link_path / "iface",   "rx_"+lc_num+"/link/iface",   std::string, string);

        TREE_CREATE_RW(rx_link_path / "jesd_num",   "rx_"+lc_num+"/link/jesd_num",   int, int);

        TREE_CREATE_RW(rx_dsp_path / "delay_iq",   "rx_"+lc_num+"/jesd/delay_iq",   std::string, string);
    }

    // initializes all TX chains
    for( size_t dspno = 0; dspno < num_tx_channels; dspno++ ) {
		std::string lc_num  = boost::lexical_cast<std::string>((char)(dspno + 'a'));
		std::string num     = boost::lexical_cast<std::string>((char)(dspno + 'A'));
		std::string chan    = "Channel_" + num;

		const fs_path tx_codec_path = CYAN_NRNT_MB_PATH / "tx_codecs" / num;
		const fs_path tx_fe_path    = CYAN_NRNT_MB_PATH / "dboards" / num / "tx_frontends" / chan;
		const fs_path db_path       = CYAN_NRNT_MB_PATH / "dboards" / num;
		const fs_path tx_dsp_path   = CYAN_NRNT_MB_PATH / "tx_dsps" / dspno;
		const fs_path tx_link_path  = CYAN_NRNT_MB_PATH / "tx_link" / dspno;

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

        // About information
		TREE_CREATE_RW(tx_path / dspno / "fw_version", "tx_"+lc_num+"/about/fw_ver", std::string, string);
		TREE_CREATE_RW(tx_path / dspno / "hw_version", "tx_"+lc_num+"/about/hw_ver", std::string, string);
		TREE_CREATE_RW(tx_path / dspno / "sw_version", "tx_"+lc_num+"/about/sw_ver", std::string, string);
		TREE_CREATE_RW(tx_path / dspno / "eeprom", "tx_"+lc_num+"/about/eeprom", std::string, string);

		// Actual frequency values
		TREE_CREATE_RW(tx_path / chan / "freq" / "value", "tx_"+lc_num+"/rf/lo_freq", double, double);

		// Power status
		TREE_CREATE_RW(tx_path / dspno / "pwr", "tx_"+lc_num+"/pwr", std::string, string);

		// Codecs, phony properties for Crimson
		TREE_CREATE_RW(tx_codec_path / "gains", "tx_"+lc_num+"/dsp/gain", int, int);
		TREE_CREATE_ST(tx_codec_path / "name", std::string, "TX Codec");

		// Daughter Boards' Frontend Settings
		TREE_CREATE_ST(tx_fe_path / "name",   std::string, "TX Board");

	    // TX bandwidth
		TREE_CREATE_ST(tx_fe_path / "bandwidth" / "value", double, (double) CYAN_NRNT_BW_FULL );
		TREE_CREATE_ST(tx_fe_path / "bandwidth" / "range", meta_range_t, meta_range_t( (double) CYAN_NRNT_BW_FULL, (double) CYAN_NRNT_BW_FULL ) );

		TREE_CREATE_ST(tx_fe_path / "freq", meta_range_t,
			meta_range_t((double) CYAN_NRNT_FREQ_RANGE_START, (double) _freq_range_stop, (double) CYAN_NRNT_FREQ_RANGE_STEP));

		TREE_CREATE_ST(tx_fe_path / "dc_offset" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));
		TREE_CREATE_ST(tx_fe_path / "iq_balance" / "value", std::complex<double>, std::complex<double>(0.0, 0.0));

		TREE_CREATE_RW(tx_fe_path / "connection",  "tx_"+lc_num+"/link/iface", std::string, string);

		TREE_CREATE_ST(tx_fe_path / "use_lo_offset", bool, false);

		TREE_CREATE_ST(tx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t((double) CYAN_NRNT_FREQ_RANGE_START, (double) _freq_range_stop, (double) CYAN_NRNT_FREQ_RANGE_STEP));

		TREE_CREATE_RW(tx_fe_path / "freq"  / "value", "tx_"+lc_num+"/rf/lo_freq" , double, double);
		TREE_CREATE_ST(tx_fe_path / "gains", std::string, "gain" );
		TREE_CREATE_RW(tx_fe_path / "gain"  / "value", "tx_"+lc_num+"/rf/gain/val" , double, double);
        // Gain range for all elements in the current band
        TREE_CREATE_RW(tx_fe_path / "gain" / "range", "tx_"+lc_num+"/rf/gain/range",  meta_range_t, meta_range);

		// RF band
		TREE_CREATE_RW(tx_fe_path / "freq" / "band", "tx_"+lc_num+"/rf/band", int, int);

        // Tx rf pll lock detect
        TREE_CREATE_RW(tx_fe_path / "sensors" / "rfpll_lock", "tx_"+lc_num+"/status/rfpll_lock", sensor_value_t, sensor_value );

		// these are phony properties for Crimson
		TREE_CREATE_ST(db_path / "tx_eeprom",  dboard_eeprom_t, dboard_eeprom_t());

		// DSPs

		TREE_CREATE_ST(tx_dsp_path / "rate" / "range", meta_range_t,
			meta_range_t((double) CYAN_NRNT_RATE_RANGE_START, (double) CYAN_NRNT_RATE_RANGE_STOP_FULL, (double) CYAN_NRNT_RATE_RANGE_STEP));
		TREE_CREATE_ST(tx_dsp_path / "freq" / "range", meta_range_t,
			meta_range_t((double) CYAN_NRNT_DSP_FREQ_RANGE_START_FULL, (double) CYAN_NRNT_DSP_FREQ_RANGE_STOP_FULL, (double) CYAN_NRNT_DSP_FREQ_RANGE_STEP));
		TREE_CREATE_ST(tx_dsp_path / "bw" / "range",   meta_range_t,
			meta_range_t((double) CYAN_NRNT_DSP_BW_START, (double) CYAN_NRNT_DSP_BW_STOP_FULL, (double) CYAN_NRNT_DSP_BW_STEPSIZE));

        TREE_CREATE_RW(tx_dsp_path / "rate" / "value", "tx_"+lc_num+"/dsp/rate", double, double);

		TREE_CREATE_RW(tx_dsp_path / "bw" / "value",   "tx_"+lc_num+"/dsp/rate",    double, double);

        //interface for setting all ncos
		TREE_CREATE_RW(tx_dsp_path / "freq" / "value", "tx_"+lc_num+ "/dsp/all_nco", double, double);

		TREE_CREATE_WO(tx_dsp_path / "rstreq", "tx_"+lc_num+"/dsp/rstreq", double, double);
		TREE_CREATE_RW(tx_dsp_path / "nco", "tx_"+lc_num+"/dsp/fpga_nco", double, double);

        //accesses the interface function for all DAC ncos
		TREE_CREATE_RW(tx_fe_path / "nco", "tx_"+lc_num+"/rf/dac/nco/dacfreq", double, double);

        TREE_CREATE_RW(tx_path / dspno / "jesd" / "status", "tx_"+lc_num+"/jesd/status", std::string, string);

		// Link settings
		TREE_CREATE_RW(tx_link_path / "vita_en", "tx_"+lc_num+"/link/vita_en", std::string, string);
        TREE_CREATE_RW(tx_link_path / "endian_swap", "tx_"+lc_num+"/link/endian_swap", int, int);
		TREE_CREATE_RW(tx_link_path / "port",    "tx_"+lc_num+"/link/port",    std::string, string);
		TREE_CREATE_RW(tx_link_path / "iface",   "tx_"+lc_num+"/link/iface",   std::string, string);

        TREE_CREATE_RW(tx_dsp_path / "delay_iq", "tx_"+lc_num+"/jesd/delay_iq",   std::string, string);

		std::string ip_addr;
		uint16_t udp_port;
		std::string sfp;
		get_tx_endpoint( dspno, ip_addr, udp_port, sfp );

        // Creates socket for getting buffer level
		_mbc.fifo_ctrl_xports.push_back(
			udp_simple::make_connected(
				_tree->access<std::string>( CYAN_NRNT_MB_PATH / "link" / sfp / "ip_addr" ).get(),
				std::to_string( _tree->access<int>( CYAN_NRNT_MB_PATH / "fpga" / "board" / "flow_control" / ( sfp + "_port" ) ).get() )
			)
		);
    }

	const fs_path cm_path  = CYAN_NRNT_MB_PATH / "cm";

	// Common Mode
	TREE_CREATE_RW(cm_path / "chanmask-rx", "cm/chanmask-rx", int, int);
	TREE_CREATE_RW(cm_path / "chanmask-tx", "cm/chanmask-tx", int, int);
	TREE_CREATE_WO(cm_path / "rx/atten/val", "cm/rx/atten/val", double, double);
	TREE_CREATE_WO(cm_path / "rx/gain/val", "cm/rx/gain/val", int, int);
    TREE_CREATE_RW(cm_path / "rx/force_stream", "cm/rx/force_stream", int, int);
	TREE_CREATE_WO(cm_path / "tx/gain/val", "cm/tx/gain/val", double, double);
    TREE_CREATE_RW(cm_path / "tx/force_stream", "cm/tx/force_stream", int, int);
	TREE_CREATE_WO(cm_path / "trx/freq/val", "cm/trx/freq/val", double, double);
	TREE_CREATE_WO(cm_path / "trx/nco_adj", "cm/trx/fpga_nco", double, double);

    //do some post-init tasks
    this->update_rates();

    fs_path root = "/mboards/0";

    std::string sub_spec_rx;
    for(size_t n =0; n < num_rx_channels; n++) {
        sub_spec_rx.push_back(n+'A');
        sub_spec_rx+= ":Channel_";
        sub_spec_rx.push_back(n+'A');
        if(n+1 !=num_rx_channels) {
            sub_spec_rx+=" ";
        }
    }
    _tree->access<subdev_spec_t>(root / "rx_subdev_spec").set(subdev_spec_t( sub_spec_rx ));
    std::string sub_spec_tx;
    for(size_t n = 0; n < num_tx_channels; n++) {
        sub_spec_tx.push_back(n+'A');
        sub_spec_tx+= ":Channel_";
        sub_spec_tx.push_back(n+'A');
        if(n+1 !=num_tx_channels) {
            sub_spec_tx+=" ";
        }
    }
    _tree->access<subdev_spec_t>(root / "tx_subdev_spec").set(subdev_spec_t( sub_spec_tx ));

	if ( _pps_thread_needed ) {
		start_pps_dtc();
	}

    //Initialize "Time Diff" mechanism before starting flow control thread
    time_spec_t ts = uhd::get_system_time();
    _streamer_start_time = ts.get_real_secs();

    // The problem is that this class does not hold a multi_crimson instance
    //Dont set time. Crimson can compensate from 0. Set time will only be used for GPS

    // Tyreus-Luyben tuned PID controller
    // Create using placement new to avoid false sharing
    new (_time_diff_pidc) uhd::pidc_tl(
        0.0, // desired set point is 0.0s error
        1.0, // measured K-ultimate occurs with Kp = 1.0, Ki = 0.0, Kd = 0.0
        // measured P-ultimate is inverse of 1/2 the flow-control sample rate
        2.0 / (double)CYAN_NRNT_UPDATE_PER_SEC
    );

    _time_diff_pidc->set_error_filter_length( CYAN_NRNT_UPDATE_PER_SEC );

    _time_diff_pidc->set_max_error_for_convergence( 10e-6 );

    device_clock_sync_info = clock_sync_shared_info::make();

    // Start the clock sync thread
    // _time_diff_pidc and device_clock_sync_info sync info are created even when the the clock synce thread is not needed to make it easier to enable/disable the thread for debugging purposes
    if ( _bm_thread_needed ) {
        start_bm();
    }

	rx_stream_cmd_issuer.reserve(num_rx_channels);
    for(size_t ch = 0; ch < num_rx_channels; ch++) {
        //The channel argument in the packet is actually the jesd number relative to the sfp port on Cyan
        //i.e. If there are two channels per sfp port one channel on each port would be 0, the other 1
        size_t jesd_num = cyan_nrnt_impl::get_rx_jesd_num(ch);

        // Gets which sfp port is used by this channel
        int xg_intf = cyan_nrnt_impl::get_rx_xg_intf(ch);

        rx_stream_cmd_issuer.emplace_back(_time_diff_iface[xg_intf], device_clock_sync_info, jesd_num, otw_rx, (size_t) nsamps_multiple_rx);
    }
}

cyan_nrnt_impl::~cyan_nrnt_impl(void)
{
    stop_bm();
    stop_pps_dtc();

    // Manually calling destructor when using placement new is required
    _time_diff_pidc->~pidc();
    free(_time_diff_pidc);
}

//gets the jesd number to be used in creating stream command packets
//Note: these are relative to the sfp port
//i.e. 0 for all channels in 9r7t since it has 1 channel per sfp port, 0 or 1 for 8r since it has 2 channels per sfp
int cyan_nrnt_impl::get_rx_jesd_num(int channel) {
    const fs_path rx_link_path  = CYAN_NRNT_MB_PATH / "rx_link" / channel;
    int jesd_num = _tree->access<int>( rx_link_path / "jesd_num" ).get();
    return jesd_num;
}

//the number corresponding to each sfp port
//i.e. sfpa==0, sfpb==1...
int cyan_nrnt_impl::get_rx_xg_intf(int channel) {
    std::string sfp = get_rx_sfp(channel);
    int xg_intf = sfp.back() - 'a';
    return xg_intf;
}

std::string cyan_nrnt_impl::get_tx_sfp( size_t chan ) {
    if ( chan >= num_tx_channels ) {
        std::string error_msg = CYAN_NRNT_DEBUG_NAME_S " requested sfp port of non-existant channel: " + std::to_string(chan);
        throw uhd::value_error(error_msg);
    }
    if( is_tx_sfp_cached[chan] ) {
        return tx_sfp_cache[chan];
    } else {
        const fs_path tx_link_path  = CYAN_NRNT_MB_PATH / "tx_link" / chan;
        tx_sfp_cache[chan] = _tree->access<std::string>( tx_link_path / "iface" ).get();
        is_tx_sfp_cached[chan] = true;
        return tx_sfp_cache[chan];
    }
}

std::string cyan_nrnt_impl::get_rx_sfp( size_t chan ) {
    if ( chan >= num_rx_channels ) {
        std::string error_msg = CYAN_NRNT_DEBUG_NAME_S " requested sfp port of non-existant channel: " + std::to_string(chan);
        throw uhd::value_error(error_msg);
    }
    if( is_rx_sfp_cached[chan] ) {
        return rx_sfp_cache[chan];
    } else {
        rx_sfp_cache[chan] = _tree->access<std::string>( rx_link_root(chan) / "iface" ).get();
        is_rx_sfp_cached[chan] = true;
        return rx_sfp_cache[chan];
    }
}

std::string cyan_nrnt_impl::get_tx_ip( size_t chan ) {
    if ( chan >= num_tx_channels ) {
        std::string error_msg = CYAN_NRNT_DEBUG_NAME_S " requested ip of non-existant channel: " + std::to_string(chan);
        throw uhd::value_error(error_msg);
    }
    if( is_tx_ip_cached[chan] ) {
        return tx_ip_cache[chan];
    } else {
        std::string sfp = get_tx_sfp(chan);

        tx_ip_cache[chan] = _tree->access<std::string>( CYAN_NRNT_MB_PATH / "link" / sfp / "ip_addr").get();
        is_tx_ip_cached[chan] = true;
        return tx_ip_cache[chan];
    }
}

uint16_t cyan_nrnt_impl::get_tx_fc_port( size_t chan ) {
    if ( chan >= num_tx_channels ) {
        std::string error_msg = CYAN_NRNT_DEBUG_NAME_S " requested fc port of non-existant channel: " + std::to_string(chan);
        throw uhd::value_error(error_msg);
    }
    if( is_tx_fc_cached[chan] ) {
        return tx_fc_cache[chan];
    } else {
        const fs_path fc_port_path = CYAN_NRNT_MB_PATH / ("fpga/board/flow_control/" + get_tx_sfp(chan) + "_port");
    
        tx_fc_cache[chan] = (uint16_t) _tree->access<int>( fc_port_path ).get();
        is_tx_fc_cached[chan] = true;
        return tx_fc_cache[chan];
    }
}

uint16_t cyan_nrnt_impl::get_tx_udp_port( size_t chan ) {
    if ( chan >= num_tx_channels ) {
        std::string error_msg = CYAN_NRNT_DEBUG_NAME_S " requested udp port of non-existant channel: " + std::to_string(chan);
        throw uhd::value_error(error_msg);
    }
    if( is_tx_udp_port_cached[chan] ) {
        return tx_udp_port_cache[chan];
    } else {
        const fs_path prop_path = CYAN_NRNT_MB_PATH / "tx_link";

        const std::string udp_port_str = _tree->access<std::string>(prop_path / std::to_string( chan ) / "port").get();

        std::stringstream udp_port_ss( udp_port_str );
        uint16_t udp_port;
        udp_port_ss >> udp_port;
        tx_udp_port_cache[chan] = udp_port;
        is_tx_udp_port_cached[chan] = true;
        return tx_udp_port_cache[chan];
    }
}

//figures out which ip address, udp port, and sfp port to use
void cyan_nrnt_impl::get_tx_endpoint( const size_t & chan, std::string & ip_addr, uint16_t & udp_port, std::string & sfp ) {
    sfp = get_tx_sfp(chan);

    udp_port = get_tx_udp_port(chan);

    ip_addr = get_tx_ip(chan);
}

constexpr double RX_SIGN = +1.0;
constexpr double TX_SIGN = -1.0;

// XXX: @CF: 20180418: stop-gap until moved to server
static int select_band( const double freq ) {
	if( freq >= CYAN_NRNT_MID_HIGH_BARRIER )
        return HIGH_BAND;
    else if( freq >= CYAN_NRNT_LOW_MID_BARRIER )
        return MID_BAND;
    else
        return LOW_BAND;
}

// return true if b is a (not necessarily strict) subset of a
static bool range_contains( const meta_range_t & a, const meta_range_t & b ) {
	return b.start() >= a.start() && b.stop() <= a.stop();
}

double cyan_nrnt_impl::choose_lo_shift( double target_freq, int band, property_tree::sptr dsp_subtree, int xx_sign, size_t chan ) {
    //lo is unused in low band
    if(band == LOW_BAND) return 0;

    double band_max_lo;
    double band_min_lo;
    if(band == MID_BAND) {
        band_max_lo = CYAN_NRNT_MID_MAX_LO;
        band_min_lo = CYAN_NRNT_LOW_MID_BARRIER;
    } else if (band == HIGH_BAND) {
        band_min_lo = CYAN_NRNT_MID_HIGH_BARRIER;
        band_max_lo = _freq_range_stop;
    } else {
        throw uhd::runtime_error("invalid band");
    }

    // Preferences:
    // Preferences:
    // 1. have entire relevant range below the lo with CYAN_NRNT_LO_TARGET_SEPERATION of seperation
    // 2. have entire relevant range above the lo with CYAN_NRNT_LO_TARGET_SEPERATION of seperation
    // 3/4. have entire relevant range below/above the lo with the largest seperation possible
    // 5. have lo as close to the middle of the relevant range as possible
    // candidate_x corresponds to the above numbers

    const double sample_rate = dsp_subtree->access<double>("/rate/value").get();
    const double user_bw = sample_rate;

    double dsp_bw;
    if(sample_rate > max_sample_rate / CYAN_NRNT_MAX_DSP_RATE_FACTOR) {
        // DSP is bypassed at higher rates
        dsp_bw = 0;
    } else {
        freq_range_t dsp_range = dsp_subtree->access<meta_range_t>("freq/range").get();
        dsp_bw = dsp_range.stop() - dsp_range.start();
    }
    
    // 3G rx has a fixed 250MHz NCO built into it in 1G mode
    // if dsp NCO is not bypassed, try to keep LO 250MHz above the target 
    double compensatory_dsp_shift;
    if(flag_use_3g_as_1g && RX_SIGN == xx_sign && dsp_bw != 0 && rx_rfe_rate_cache[chan]) {
        compensatory_dsp_shift = CYAN_NRNT_RX_NCO_SHIFT_3G_TO_1G;
    } else {
        compensatory_dsp_shift = 0;
    }

    const freq_range_t relevant_range( target_freq - (user_bw / 2.0), target_freq + (user_bw / 2.0), 0 );

    double a = std::ceil( (relevant_range.stop() + compensatory_dsp_shift + CYAN_NRNT_LO_TARGET_SEPERATION) / CYAN_NRNT_LO_STEPSIZE );
    double candidate_1 = a * CYAN_NRNT_LO_STEPSIZE;
    // Frequence range observable below the candidate lo; candidate_1
    const freq_range_t below_lo(candidate_1 - (dsp_bw / 2.0) + std::min(compensatory_dsp_shift, 0.0), candidate_1, 0);

    // The relevant range can be fit between a viable lo and the dsp's lower limit
    // Due to issues with rx in the 3G to 1G conversion the negative side band cannot be used with it
    if(range_contains(below_lo, relevant_range) && candidate_1 >= band_min_lo && candidate_1 <= band_max_lo && !flag_use_3g_as_1g) return candidate_1;

    double b = std::floor( (relevant_range.start() - compensatory_dsp_shift - CYAN_NRNT_LO_TARGET_SEPERATION) / CYAN_NRNT_LO_STEPSIZE );
    double candidate_2 = b * CYAN_NRNT_LO_STEPSIZE;
    // compensatory_dsp_shift of the dsp bw is needed to complensate for the ADC NCO shift
    const freq_range_t above_lo(candidate_2, candidate_2 + (dsp_bw / 2.0) - std::max(compensatory_dsp_shift, 0.0), 0);

    // The relevant range can be fit between a viable lo and the dsp's upper limit
    if(range_contains(above_lo, relevant_range) && candidate_2 >= band_min_lo && candidate_2 <= band_max_lo) return candidate_2;

    // Test los that are closer to the target band than CYAN_NRNT_LO_TARGET_SEPERATION, but still outside the band
    while(a * CYAN_NRNT_LO_STEPSIZE + ((user_bw / 2.0)) > relevant_range.stop() && b * CYAN_NRNT_LO_STEPSIZE - (user_bw / 2.0) < relevant_range.start()) {
        a--;
        b++;

        // Test a new lo that is above the target range and slightly closer than the last check
        double candidate_3 = a * CYAN_NRNT_LO_STEPSIZE;
        const freq_range_t below_lo_low_seperation(candidate_3 - (dsp_bw / 2.0) + std::min(compensatory_dsp_shift, 0.0), candidate_3, 0);
        // Due to issues with rx in the 3G to 1G conversion the negative side band cannot be used with it
        if(range_contains(below_lo_low_seperation, relevant_range) && candidate_3 >= band_min_lo && candidate_3 <= band_max_lo && !flag_use_3g_as_1g) return candidate_3;

        // Test a new lo that is above the target range and slightly closer than the last check
        double candidate_4 = b * CYAN_NRNT_LO_STEPSIZE;
        const freq_range_t above_lo_low_seperation(candidate_4, candidate_4 + (dsp_bw / 2.0) - std::max(compensatory_dsp_shift, 0.0), 0);
        if(range_contains(above_lo_low_seperation, relevant_range) && candidate_4 >= band_min_lo && candidate_4 <= band_max_lo) return candidate_4;
    }

    // Fallback to having the lo centered
    if(flag_use_3g_as_1g && RX_SIGN == xx_sign) {
        return std::max(std::min(::round( (target_freq + CYAN_NRNT_RX_NCO_SHIFT_3G_TO_1G) / CYAN_NRNT_LO_STEPSIZE ) * CYAN_NRNT_LO_STEPSIZE, band_max_lo), (double) band_min_lo);
    } else {
        return std::max(std::min(::round( (target_freq) / CYAN_NRNT_LO_STEPSIZE ) * CYAN_NRNT_LO_STEPSIZE, band_max_lo), (double) band_min_lo);
    }
}

// XXX: @CF: 20180418: stop-gap until moved to server
//calculates and sets the band, nco, and lo shift
tune_result_t cyan_nrnt_impl::tune_xx_subdev_and_dsp( const double xx_sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, const tune_request_t &tune_request, int* gain_is_set, int* last_set_band, size_t chan ) {

	freq_range_t dsp_range = dsp_subtree->access<meta_range_t>("freq/range").get();
	freq_range_t rf_range = rf_fe_subtree->access<meta_range_t>("freq/range").get();
	freq_range_t adc_range( dsp_range.start(), dsp_range.stop(), 0.0001 );

	double clipped_requested_freq = rf_range.clip( tune_request.target_freq );
    
    int band;
    // Is tx and tx channel is low band only
    if(TX_SIGN == xx_sign && is_tx_baseband_only[chan]) {
        band = LOW_BAND;
    // Is a normal board, use normal band selection
    } else {
        band = select_band( clipped_requested_freq );
    }

	//------------------------------------------------------------------
	//-- set the RF frequency depending upon the policy
	//------------------------------------------------------------------
    // Desired LO frequency
	double target_rf_freq = 0.0;

	if ( TX_SIGN == xx_sign ) {
		rf_fe_subtree->access<double>("nco").set( 0.0 );
	}

	switch (tune_request.rf_freq_policy){
		case tune_request_t::POLICY_AUTO:
			switch( band ) {
			case LOW_BAND:
				// in low band, we only use the DSP to tune
				target_rf_freq = 0;
				break;
            //The differences between mid and high band are handled on the server
			case MID_BAND:
            case HIGH_BAND:
                target_rf_freq = choose_lo_shift( clipped_requested_freq, band, dsp_subtree, xx_sign, chan );
				break;
			}
		break;

		case tune_request_t::POLICY_MANUAL:
            // prevent use of low band when a specific lo is requested
            if(band == LOW_BAND && tune_request.lo_freq !=0) band = MID_BAND;
			target_rf_freq = tune_request.lo_freq;
			break;

		case tune_request_t::POLICY_NONE:
			break; //does not set
	}

    rf_fe_subtree->access<int>( "freq/band" ).set( band );

    if(!gain_reset_warning_printed) {
        if(*gain_is_set) {
            if(*last_set_band != band) {
                UHD_LOG_INFO("GAIN", "Band changed after setting gain old gain value will be ignored. Remember to set gain again");
                *gain_is_set = false;
                gain_reset_warning_printed = true;
            }
        }
        *last_set_band = band;
    }


	//------------------------------------------------------------------
	//-- Tune the RF frontend
	//------------------------------------------------------------------
	rf_fe_subtree->access<double>("freq/value").set( target_rf_freq );
	const double actual_rf_freq = rf_fe_subtree->access<double>("freq/value").get();

    if(actual_rf_freq == 0 && target_rf_freq != 0) {
        UHD_LOG_ERROR(CYAN_NRNT_DEBUG_NAME_C, "Error when attempting to set lo on channel " + std::string(1, chan + 'A') + ". If this error persists contact support");
    }

	//------------------------------------------------------------------
	//-- Set the DSP frequency depending upon the DSP frequency policy.
	//------------------------------------------------------------------
	double target_dsp_freq = 0.0;
	switch (tune_request.dsp_freq_policy) {
		case tune_request_t::POLICY_AUTO:
			target_dsp_freq = actual_rf_freq - clipped_requested_freq;

			//invert the sign on the dsp freq for transmit (spinning up vs down)
			target_dsp_freq *= xx_sign;

			break;

		case tune_request_t::POLICY_MANUAL:
			target_dsp_freq = tune_request.dsp_freq;
			break;

		case tune_request_t::POLICY_NONE:
			break; //does not set
	}

	//------------------------------------------------------------------
	//-- Tune the DSP
	//------------------------------------------------------------------
	dsp_subtree->access<double>("freq/value").set(target_dsp_freq);
	const double actual_dsp_freq = dsp_subtree->access<double>("freq/value").get();

    // Warn user if they attempted manual tuning, and the resulting NCO is 0 and the intended NCO was not 0 (which occurs at high sample rates where the DSP is bypassed)
    if( (tune_request.dsp_freq_policy == tune_request_t::POLICY_MANUAL || tune_request.rf_freq_policy == tune_request_t::POLICY_MANUAL) && target_dsp_freq != 0 && actual_dsp_freq == 0) {
        const double sample_rate = dsp_subtree->access<double>("/rate/value").get();
        if(sample_rate > max_sample_rate / CYAN_NRNT_MAX_DSP_RATE_FACTOR) {
            UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "Manual tune request required use of the NCO, but the NCO is disabled at above " << (max_sample_rate / CYAN_NRNT_MAX_DSP_RATE_FACTOR) / 1e6 << "Msps" ;
        }
    }

	//------------------------------------------------------------------
	//-- Load and return the tune result
	//------------------------------------------------------------------
	tune_result_t tune_result;
	tune_result.clipped_rf_freq = clipped_requested_freq;
	tune_result.target_rf_freq = target_rf_freq;
	tune_result.actual_rf_freq = actual_rf_freq;
	tune_result.target_dsp_freq = target_dsp_freq;
	tune_result.actual_dsp_freq = actual_dsp_freq;
	return tune_result;
}

uhd::tune_result_t cyan_nrnt_impl::set_rx_freq(
	const uhd::tune_request_t &tune_request, size_t chan
) {

	tune_result_t result = tune_xx_subdev_and_dsp(RX_SIGN,
			_tree->subtree(rx_dsp_root(chan)),
			_tree->subtree(rx_rf_fe_root(chan)),
			tune_request,
            &rx_gain_is_set[chan], &last_set_rx_band[chan],
            chan);
	return result;

}

double cyan_nrnt_impl::get_rx_freq(size_t chan) {

        double cur_dsp_nco = _tree->access<double>(rx_dsp_root(chan) / "nco").get();
        double cur_lo_freq = 0;
        if (_tree->access<int>(rx_rf_fe_root(chan) / "freq" / "band").get() > 0) {
            cur_lo_freq = _tree->access<double>(rx_rf_fe_root(chan) / "freq" / "value").get();
        }
        return cur_lo_freq - cur_dsp_nco;
}

uhd::tune_result_t cyan_nrnt_impl::set_tx_freq(
	const uhd::tune_request_t &tune_request, size_t chan
) {

	tune_result_t result = tune_xx_subdev_and_dsp(TX_SIGN,
			_tree->subtree(tx_dsp_root(chan)),
			_tree->subtree(tx_rf_fe_root(chan)),
			tune_request,
            &tx_gain_is_set[chan], &last_set_tx_band[chan],
            chan);
	return result;

}

double cyan_nrnt_impl::get_tx_freq(size_t chan) {

        //gets FPGA nco
        double cur_nco = _tree->access<double>(tx_dsp_root(chan) / "freq" / "value").get();
        //The system does not currently use then channelizer nco are DAC, but if a future version begins using this it will need to be added
        double cur_lo_freq = 0;
        if (_tree->access<int>(tx_rf_fe_root(chan) / "freq" / "band").get() != LOW_BAND) {
                cur_lo_freq = _tree->access<double>(tx_rf_fe_root(chan) / "freq" / "value").get();
        }
        return cur_lo_freq + cur_nco;
}
void cyan_nrnt_impl::set_tx_gain(double gain, const std::string &name, size_t chan){

    if ( multi_usrp::ALL_CHANS != chan ) {
        (void) name;

        // Used to decide if a warning should be printed when changing bands
        if(gain != 0) {
            tx_gain_is_set[chan] = true;
        } else {
            // Set to false to avoid spurious warning when changing band
            tx_gain_is_set[chan] = false;
        }

        _tree->access<double>(tx_rf_fe_root(chan) / "gain" / "value").set(gain);
        return;
    }
    for (size_t c = 0; c < num_tx_channels; c++){
        set_tx_gain(gain, name, c);
    }
}

double cyan_nrnt_impl::get_tx_gain(const std::string &name, size_t chan) {
    (void) name;

    return _tree->access<double>(tx_rf_fe_root(chan) / "gain" / "value").get();
}

int64_t cyan_nrnt_impl::get_tx_buff_scale() {
    return buffer_level_multiple;
}

void cyan_nrnt_impl::set_rx_gain(double gain, const std::string &name, size_t chan) {

    //If only one channel is selected sets the values in the state tree for gain, otherwise calls this function for each channel individually
    if ( multi_usrp::ALL_CHANS != chan ) {

        (void) name;

        // Used to decide if a warning should be printed when changing bands
        if(gain != 0) {
            rx_gain_is_set[chan] = true;
        } else {
            // Set to false to avoid spurious warning when changing band
            rx_gain_is_set[chan] = false;
        }

        //sets gain in the rf chain (variable amplifier, variable attenuator, bypassable amplifier
        //currently deciding how to combine them is calculated on the server. On older versions UHD determined how to adjust them
        _tree->access<double>( rx_rf_fe_root(chan) / "gain" / "value" ).set( gain );

        return;
    }
    
    //calls this function for each channel individually
    for (size_t c = 0; c < num_rx_channels; c++){
        set_rx_gain( gain, name, c );
    }
}

double cyan_nrnt_impl::get_rx_gain(const std::string &name, size_t chan) {
    (void) name;

    return _tree->access<double>(rx_rf_fe_root(chan) / "gain" / "value").get();
}

void cyan_nrnt_impl::set_rx_rate(double rate, size_t chan) {
    if (chan != multi_usrp::ALL_CHANS) {
        _tree->access<double>(rx_dsp_root(chan) / "rate" / "value").set(rate);

        double actual_rate = get_rx_rate(chan);
        do_samp_rate_warning_message(rate, actual_rate, "RX", chan);

        rx_rate_check(chan, rate);

        update_rx_samp_rate(chan, actual_rate);

        return;
    }
    for (size_t c = 0; c < num_rx_channels; c++) {
        set_rx_rate(rate, c);
    }
}

double cyan_nrnt_impl::get_rx_rate(size_t chan) {
    return _tree->access<double>(rx_dsp_root(chan) / "rate" / "value").get();
}

void cyan_nrnt_impl::set_tx_rate(double rate, size_t chan) {
    if (chan != multi_usrp::ALL_CHANS) {
        _tree->access<double>(tx_dsp_root(chan) / "rate" / "value").set(rate);

        double actual_rate = get_tx_rate(chan);
        do_samp_rate_warning_message(rate, actual_rate, "TX", chan);

        tx_rate_check(chan, rate);

        update_tx_samp_rate(chan, actual_rate);

        return;
    }
    for (size_t c = 0; c < num_tx_channels; c++) {
        set_tx_rate(rate, c);
    }
}

double cyan_nrnt_impl::get_tx_rate(size_t chan) {
    return _tree->access<double>(tx_dsp_root(chan) / "rate" / "value").get();
}

inline void cyan_nrnt_impl::request_resync_time_diff() {
    device_clock_sync_info->request_resync();
}

bool cyan_nrnt_impl::ping_check(std::string sfp, std::string ip) {
    std::lock_guard<std::mutex> ping_lock(ping_mutex);

    size_t sfp_num = sfp.back() - 'a';
    if(sfp_num > NUMBER_OF_XG_CONTROL_INTF) {
        UHD_LOG_ERROR(CYAN_NRNT_DEBUG_NAME_C, "Ping check requested for sfp port that does not exist: " + sfp);
        return false;
    }
    // This sfp port has already been pinged, do not check again
    if(ping_check_completed[sfp_num]) {
        return sfp_working[sfp_num];
    }

    char cmd[128];
    snprintf(cmd, 128, "ping -c 1 -W 1 %s  > /dev/null 2>&1", ip.c_str());
    int check = system(cmd);

    // Mark this sfp port as having been checked
    ping_check_completed[sfp_num] = true;
    sfp_working[sfp_num] = !check;

    return sfp_working[sfp_num];
}

double cyan_nrnt_impl::get_link_rate() {
    if(link_rate_cache == 0) {
        link_rate_cache = _tree->access<double>(CYAN_NRNT_MB_PATH / "link_max_rate").get();
        return link_rate_cache;
    } else {
        return link_rate_cache;
    }
}
