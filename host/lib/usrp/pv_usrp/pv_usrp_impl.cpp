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
#include <functional>
#include <boost/foreach.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/endian/conversion.hpp>

#include "pv_usrp_impl.hpp"
#include "pv_usrp_fw_common.h"

#include "uhd/transport/if_addrs.hpp"
#include "uhd/transport/udp_simple.hpp"
#include "uhd/types/stream_cmd.hpp"
#include "uhd/utils/static.hpp"

#include <uhdlib/transport/udp_common.hpp>

namespace link_pv_usrp {
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

std::string pv_usrp_impl::rx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/rx_link/" + std::to_string(channel);
}

std::string pv_usrp_impl::tx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/tx_link/" + std::to_string(channel);
}

std::string pv_usrp_impl::tx_dsp_root(const size_t channel, const size_t mboard) {
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
void pv_usrp_impl::set_stream_cmd( const std::string pre, stream_cmd_t stream_cmd ) {
    const size_t ch = pre_to_ch( pre );

    rx_stream_cmd_issuer[ch].issue_stream_command(stream_cmd);
}

// Loop that polls the device to verify the PPS is working
void pv_usrp_impl::detect_pps( pv_usrp_impl *dev ) {
    dev->_pps_thread_running = true;
    int pps_detected;

    while (! dev->_pps_thread_should_exit) {
        dev->get_tree()->access<int>(PV_USRP_TIME_PATH / "pps_detected").set(1);
        pps_detected = dev->get_tree()->access<int>(PV_USRP_TIME_PATH / "pps_detected").get();

        if (pps_detected == 0) {
            std::cout << "WARNING: PPS has not been detected in the past two seconds" << std::endl;
            // Stop PPS monitoring after one failure to avoid spamming the user with the same warning message
            dev->_pps_thread_should_exit = true;
        }
        #ifdef DEBUG_COUT
        std::cout << "PPS flag: " << pps_detected << std::endl;
        #endif

        sleep(1);
    }
    dev->_pps_thread_running = false;
}

void pv_usrp_impl::set_command_time( const std::string key, time_spec_t value ) {
    (void) key;

    _command_time = value;
}

void pv_usrp_impl::send_gpio_burst_req(const gpio_burst_req& req) {
    // TODO: figure out if this can be sent on any SFP port and if so use _which_time_diff_iface instead of 0
    _time_diff_iface[0]->send(boost::asio::const_buffer(&req, sizeof(req)));
}

//TODO: validate if this works
// It is okay to leave set/get user reg here because it requires use of the SFP ports (which iface doesn't have) and will never be accessed by a streamer
void pv_usrp_impl::set_user_reg(const std::string key, user_reg_t value) {
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

void pv_usrp_impl::set_time_now(const time_spec_t& time_spec, size_t mboard) {
    _tree->access<time_spec_t>(mb_root(mboard) / "time/now").set(time_spec);
    request_resync_time_diff();
}

// TODO: change the rx functions that bind to this to use their own clock_sync_shared_info
uhd::time_spec_t pv_usrp_impl::get_time_now() {
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

void pv_usrp_impl::set_properties_from_addr() {
    // TODO: Use name of whatever device inherits this
    static const std::string pv_prop_prefix( "pv:" );
    static const std::vector<std::string> blacklist { "pv:sob" };

    for( auto & prop: device_addr.keys() ) {
        if ( 0 == prop.compare( 0, pv_prop_prefix.length(), pv_prop_prefix ) ) {

            bool is_blacklisted = false;
            for( auto & e: blacklist ) {
                if ( e == prop ) {
                    is_blacklisted = true;
                }
            }
            if ( is_blacklisted ) {
                continue;
            }

            std::string key = prop.substr( pv_prop_prefix.length() );
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
device_addrs_t pv_usrp_impl::pv_usrp_find_with_addr(const device_addr_t &hint)
{
    // temporarily make a UDP device only to look for devices
    // loop for all the available ports, if none are available, that means all 8 are open already
    udp_simple::sptr comm = udp_simple::make_broadcast(
        hint["addr"], BOOST_STRINGIZE(PV_USRP_FW_COMMS_UDP_PORT));

    //send request for echo
    comm->send(asio::buffer("1,get,fpga/about/name", sizeof("1,get,fpga/about/name")));

    // List is Crimsons connected
    device_addrs_t pv_usrp_addrs;
    char buff[PV_USRP_FW_COMMS_MTU] = {};

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
        pv_usrp_addrs.push_back(new_addr);
    }

    // List of devices that match the filter
    device_addrs_t matching_addr;

    // Gets the Serial number for all connected Cyans found in the previous loop, and adds them to the return list if all required parameters match filters
    for(auto& addr : pv_usrp_addrs) {
        udp_simple::sptr comm = udp_simple::make_connected(
            addr["addr"], BOOST_STRINGIZE(PV_USRP_FW_COMMS_UDP_PORT));


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

// TODO: Add common <device_name>_find() function

/**
 * Buffer Management / Time Diff
 */

// SoB: Time Diff (Time Diff mechanism is used to get an accurate estimate of Crimson's absolute time)
inline int64_t pv_usrp_impl::ticks_to_nsecs( int64_t tv_tick ) {
    return (int64_t)( (double) tv_tick * _tick_period_ns ) /* [tick] * [ns/tick] = [ns] */;
}
inline int64_t pv_usrp_impl::nsecs_to_ticks( int64_t tv_nsec ) {
    return (int64_t)( (double) tv_nsec / _tick_period_ns )  /* [ns] / [ns/tick] = [tick] */;
}

inline void pv_usrp_impl::make_time_diff_packet( time_diff_req & pkt, time_spec_t ts = uhd::get_system_time() ) {
    pkt.header = (uint64_t)0x20002 << 16;
    pkt.tv_sec = ts.get_full_secs();
    pkt.tv_tick = nsecs_to_ticks( (int64_t) ( ts.get_frac_secs() * 1e9 ) );

    boost::endian::native_to_big_inplace( pkt.header );
    boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_sec );
    boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_tick );
}

// TODO: Common time_diff_send and recv?

/// SoB Time Diff: feed the time diff error back into out control system
void pv_usrp_impl::time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now ) {

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

//stops clock synchronization
void pv_usrp_impl::stop_bm() {

    if ( _bm_thread_running ) {

        _bm_thread_should_exit = true;
        _bm_thread.join();

    }
}

void pv_usrp_impl::start_pps_dtc() {

    if ( ! _pps_thread_needed ) {
        return;
    }

    if ( ! _pps_thread_running ) {
        _pps_thread_should_exit = false;
        _pps_thread = std::thread( detect_pps, this );
    }
}

void pv_usrp_impl::stop_pps_dtc() {

    if ( ! _pps_thread_needed ) {
        return;
    }

    if(_pps_thread.joinable()) {
        _pps_thread_should_exit = true;
        _pps_thread.join();
    }
}

// TODO: Common bm_thread_fn

/***********************************************************************
 * Make
 **********************************************************************/

// This is the core function that registers itself with uhd::device base class. The base device class
// will have a reference to all the registered devices and upon device find/make it will loop through
// all the registered devices' find and make functions.
UHD_STATIC_BLOCK(register_pv_usrp_device)
{
    set_log_level( uhd::log::severity_level::info );
    device::register_device(&pv_usrp_impl::pv_usrp_find, &pv_usrp_make, device::USRP);
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

pv_usrp_impl::pv_usrp_impl(const device_addr_t &_device_addr)
:
    device_addr(_device_addr),
    _pv_usrp_debug_name(debug_name),
    _time_diff_pidc((uhd::pidc*) aligned_alloc(CACHE_LINE_SIZE, padded_pidc_tcl_size)),
    _bm_thread_needed( true ),
    _bm_thread_running( false ),
    _bm_thread_should_exit( false ),
    _pps_thread_running( false ),
    _pps_thread_should_exit( false ),
    _command_time( 0.0 )
{
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
            BOOST_STRINGIZE( PV_USRP_FW_COMMS_UDP_PORT )
        )
    );

    // Create the file tree of properties.
    // Crimson/Cyan only has support for one mother board, and the RF chains will show up individually as daughter boards.
    // All the initial settings are read from the current status of the board.
    _tree = uhd::property_tree::make();

    // TODO check if locked already
    // TODO lock the Crimson device to this process, this will prevent the Crimson device being used by another program

}

pv_usrp_impl::~pv_usrp_impl(void)
{
    stop_bm();
    stop_pps_dtc();

    // Manually calling destructor when using placement new is required
    _time_diff_pidc->~pidc();
    free(_time_diff_pidc);
}

constexpr double RX_SIGN = +1.0;
constexpr double TX_SIGN = -1.0;

// return true if b is a (not necessarily strict) subset of a
static bool range_contains( const meta_range_t & a, const meta_range_t & b ) {
    return b.start() >= a.start() && b.stop() <= a.stop();
}

uhd::tune_result_t pv_usrp_impl::set_rx_freq(
    const uhd::tune_request_t &tune_request, size_t chan
) {
    tune_result_t result = tune_xx_subdev_and_dsp(RX_SIGN,
            _tree->subtree(rx_dsp_root(chan)),
            _tree->subtree(rx_rf_fe_root(chan)),
            tune_request,
            &rx_gain_is_set[chan],
            &last_set_rx_band[chan],
            chan);
    return result;

}

double pv_usrp_impl::get_rx_freq(size_t chan) {
    double cur_dsp_nco = _tree->access<double>(rx_dsp_root(chan) / "nco").get();
    double cur_lo_freq = 0;
    if (_tree->access<int>(rx_rf_fe_root(chan) / "freq" / "band").get() > 0) {
        cur_lo_freq = _tree->access<double>(rx_rf_fe_root(chan) / "freq" / "value").get();
    }
    return cur_lo_freq - cur_dsp_nco;
}

uhd::tune_result_t pv_usrp_impl::set_tx_freq(
    const uhd::tune_request_t &tune_request, size_t chan
) {
    tune_result_t result = tune_xx_subdev_and_dsp(TX_SIGN,
            _tree->subtree(tx_dsp_root(chan)),
            _tree->subtree(tx_rf_fe_root(chan)),
            tune_request,
            &tx_gain_is_set[chan],
            &last_set_tx_band[chan],
            chan);
    return result;
}

void pv_usrp_impl::set_rx_rate(double rate, size_t chan) {
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

double pv_usrp_impl::get_rx_rate(size_t chan) {
    return _tree->access<double>(rx_dsp_root(chan) / "rate" / "value").get();
}

void pv_usrp_impl::set_tx_rate(double rate, size_t chan) {
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

double pv_usrp_impl::get_tx_rate(size_t chan) {
    return _tree->access<double>(tx_dsp_root(chan) / "rate" / "value").get();
}

inline void pv_usrp_impl::request_resync_time_diff() {
    device_clock_sync_info->request_resync();
}

double pv_usrp_impl::get_link_rate() {
    if(link_rate_cache == 0) {
        link_rate_cache = _tree->access<double>(PV_USRP_MB_PATH / "link_max_rate").get();
        return link_rate_cache;
    } else {
        return link_rate_cache;
    }
}
