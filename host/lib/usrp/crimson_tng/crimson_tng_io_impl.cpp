//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Per Vices Corporation
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

#include <stdlib.h>

#include <cctype>
#include <iomanip>
#include <mutex>
#include <sys/file.h>

#include "crimson_tng_io_impl.hpp"
#include "crimson_tng_impl.hpp"
#include "crimson_tng_fw_common.h"
#include <uhd/utils/log.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread.hpp>
#include <format>
#include <functional>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <boost/endian/buffers.hpp>
#include <boost/endian/conversion.hpp>

#include <uhdlib/utils/system_time.hpp>
#include <uhd/utils/log.hpp>

#if 0
#ifndef UHD_TXRX_DEBUG_PRINTS
#define UHD_TXRX_DEBUG_PRINTS
#endif
#endif

#if 0
#ifndef UHD_TXRX_SEND_DEBUG_PRINTS
#define UHD_TXRX_SEND_DEBUG_PRINTS
#endif
#endif

#if 0
#ifndef DEBUG_FC
#define DEBUG_FC
#endif
#endif

#if 0
#ifndef BUFFER_LVL_DEBUG
#define BUFFER_LVL_DEBUG
#endif
#endif

#if 0
#ifndef UHD_TXRX_DEBUG_TIME
#define UHD_TXRX_DEBUG_TIME
#endif
#endif

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace ph = std::placeholders;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

/***********************************************************************
* helpers
**********************************************************************/

std::ostream & operator<<( std::ostream & os, const uhd::time_spec_t & ts ) {
    os << std::fixed << std::setprecision( 6 ) << ts.get_real_secs();
    return os;
}

crimson_tng_recv_packet_streamer::crimson_tng_recv_packet_streamer(const std::string product_name_c, const std::vector<size_t> channels, const std::vector<int>& recv_sockets, const std::vector<std::string>& dsp_ip, const size_t max_sample_bytes_per_packet, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian,  std::shared_ptr<std::vector<bool>> rx_channel_in_use, size_t device_total_rx_channels, pv_iface::sptr iface, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuer, std::vector<int> channel_locks, std::vector<int> streaming_locks)
: pv_device_recv_packet_streamer(product_name_c, channels, recv_sockets, dsp_ip, max_sample_bytes_per_packet, CRIMSON_TNG_HEADER_SIZE, CRIMSON_TNG_TRAILER_SIZE, cpu_format, wire_format, wire_little_endian, rx_channel_in_use, device_total_rx_channels, iface, cmd_issuer, channel_locks, streaming_locks)
{}

crimson_tng_recv_packet_streamer::~crimson_tng_recv_packet_streamer() = default;

crimson_tng_send_packet_streamer::crimson_tng_send_packet_streamer(const std::string product_name_c, const std::vector<size_t>& channels, const size_t max_num_samps, const size_t max_bl, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, double tick_rate, const std::shared_ptr<uhd::pv_tx_async_msg_queue> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<std::vector<bool>> tx_channel_in_use, pv_iface::sptr iface, std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info, std::vector<int> channel_locks, std::vector<int> streaming_locks)
: pv_device_send_packet_streamer(product_name_c, channels, max_num_samps, max_bl, dst_ips, dst_ports, device_target_nsamps, CRIMSON_TNG_PACKET_NSAMP_MULTIPLE, tick_rate, CRIMSON_TNG_MIN_TX_DELAY, CRIMSON_TNG_UPDATE_PER_SEC, async_msg_fifo, cpu_format, wire_format, wire_little_endian, tx_channel_in_use, iface, clock_sync_info, channel_locks, streaming_locks)
{}

crimson_tng_send_packet_streamer::~crimson_tng_send_packet_streamer() = default;

void crimson_tng_send_packet_streamer::set_xport_chan_fifo_lvl_abs( size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs ) {
    _eprops.at(chan).xport_chan_fifo_lvl_abs = get_fifo_lvl_abs;
}
void crimson_tng_send_packet_streamer::set_channel_name( size_t chan, std::string name ) {
    _eprops.at(chan).name = name;
}
// Stores the channel sample rate in eprops variable
void crimson_tng_send_packet_streamer::sync_channel_rate( size_t chan, double rate ) {
    size_t channel_index = std::distance(_channels, std::find(_channels, _channels + _NUM_CHANNELS, chan));
    _eprops.at(channel_index).sample_rate = rate;
}
void crimson_tng_send_packet_streamer::resize(const size_t size){
    _eprops.resize( size );
}

// Check that all channels on a streamer have the same sample rate.
void crimson_tng_send_packet_streamer::check_matching_rates() {
    const std::string mismatch_message = "Multiple sample rates are detected, but a streamer can only handle one.\n"
        "Make sure the specified sample rate is valid and identical for all channels or use multiple streamers instead.\n";
    double prev_rate = _eprops[0].sample_rate;
    for (auto &e : _eprops) {
        if (e.sample_rate != prev_rate) {
            UHD_LOG_ERROR(_product_name_c, mismatch_message);
            throw uhd::runtime_error(mismatch_message);
        }
        prev_rate = e.sample_rate;
    }
}


/***********************************************************************
* constants
**********************************************************************/
static const size_t vrt_send_header_offset_words32 = 0;

/***********************************************************************
* Helper Functions
**********************************************************************/

void crimson_tng_impl::rx_rate_check(size_t ch, double rate_samples) {
    rx_sfp_throughput_used[ch] = rate_samples;
    // Only print the warning once
    if(rx_rate_warning_printed) {
        return;
    }
    double rate_used = 0;
    for(size_t n = 0; n < num_rx_channels; n++) {
        if(get_rx_sfp(n) == get_rx_sfp(ch) && rx_channel_in_use->at(n)) {
            rate_used += rx_sfp_throughput_used[n];
        }
    }

    if(rate_used * CRIMSON_TNG_RX_SAMPLE_BITS * 2 > get_link_rate()) {

        UHD_LOGGER_WARNING(product_name_c)
                << std::format("The total sum of rates ({} MSps on SFP used by channel {})"
                                "exceeds the maximum capacity of the connection.\n"
                                "This can cause overflows.",
                    (rate_used / 1e6), ch);

        rx_rate_warning_printed = true;
    }
}

void crimson_tng_impl::update_rx_samp_rate(const size_t chan, const double rate ){

    // Get the streamer corresponding to the channel
    std::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer = _mbc.rx_streamers[chan].lock();
    // if shared_ptr is false then no streamer is using this ch
    if (!my_streamer) return;

    // Inform the streamer of the sample rate change
    my_streamer->set_sample_rate(rate);
}

void crimson_tng_impl::tx_rate_check(size_t ch, double rate_samples) {
    tx_sfp_throughput_used[ch] = rate_samples;
    double rate_used = 0;
    for(size_t n = 0; n < num_tx_channels; n++) {
        if(get_tx_sfp(n) == get_tx_sfp(ch) && tx_channel_in_use->at(n)) {
            rate_used += tx_sfp_throughput_used[n];
        }
    }

    if(rate_used * CRIMSON_TNG_TX_SAMPLE_BITS * 2 > get_link_rate() && !tx_rate_warning_printed) {

        UHD_LOGGER_WARNING(product_name_c)
                << std::format("The total sum of rates ({} MSps on SFP used by channel {})"
                                "exceeds the maximum capacity of the connection.\n"
                                "This can cause underruns.",
                    (rate_used / 1e6), ch);

        // Only print this warning once
        tx_rate_warning_printed = true;
    }
}

void crimson_tng_impl::update_tx_samp_rate(const size_t chan, const double rate ){

    // Get the streamer corresponding to the channel
    std::shared_ptr<crimson_tng_send_packet_streamer> my_streamer = _mbc.tx_streamers[chan].lock();
    // if shared_ptr is false then no streamer is using this ch
    if (!my_streamer) return;
    // Store specified channels sample rate
    my_streamer->sync_channel_rate(chan, rate);
    // Inform the streamer of the sample rate change
    my_streamer->set_samp_rate(rate);
}

void crimson_tng_impl::update_rates(void){

    for(size_t ch = 0; ch < num_tx_channels; ch++) {
        double rate = _mbc.iface->get_double( "tx_" + std::string( 1, 'a' + ch ) + "/dsp/rate" );
        update_tx_samp_rate(ch, rate);
    }

    for(size_t ch = 0; ch < num_rx_channels; ch++) {
        double rate = _mbc.iface->get_double( "rx_" + std::string( 1, 'a' + ch ) + "/dsp/rate" );
        update_rx_samp_rate(ch, rate);
    }
}

void crimson_tng_impl::update_rx_subdev_spec(const subdev_spec_t &spec){
    (void) spec;

    //fs_path root = "/mboards/0/dboards";

    //sanity checking
    //validate_subdev_spec(_tree, spec, "rx", "0");

    //setup mux for this spec
    //bool fe_swapped = false;
    //for (size_t i = 0; i < spec.size(); i++){
    //    const std::string conn = _tree->access<std::string>(root / spec[i].db_name / "rx_frontends" / spec[i].sd_name / "connection").get();
    //    if (i == 0 and (conn == "QI" or conn == "Q")) fe_swapped = true;
    //    _mbc.rx_dsps[i]->set_mux(conn, fe_swapped);
    //}
    //_mbc.rx_fe->set_mux(fe_swapped);
}

void crimson_tng_impl::update_tx_subdev_spec(const subdev_spec_t &spec){
    (void) spec;

    //fs_path root = "/mboards/0/dboards";

    //sanity checking
    //validate_subdev_spec(_tree, spec, "tx", "0");

    //set the mux for this spec
    //const std::string conn = _tree->access<std::string>(root / spec[0].db_name / "tx_frontends" / spec[0].sd_name / "connection").get();
    //_mbc.tx_fe->set_mux(conn);
}

/***********************************************************************
* Async Data
**********************************************************************/
bool crimson_tng_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    if(!recv_async_msg_deprecated_warning) {
        std::cout << "device recv_async_msg function is deprecated. Stream to tx_streamer.recv_async_msg\n";
        recv_async_msg_deprecated_warning = true;
    }
    // The fifo is created during get_tx_stream, as part of changes to better handle stream specific get async messages
    // The means calling the device get async msg (this function) before creating a stream can be done before the fifo is created
    if(_async_msg_fifo.get() != NULL) {
        return _async_msg_fifo->pop(&async_metadata, timeout);
    } else {
        return false;
    }
}

/***********************************************************************
* Receive streamer
**********************************************************************/
rx_streamer::sptr crimson_tng_impl::get_rx_stream(const uhd::stream_args_t &args_){
    // Set flag to indicate clock sync is desired so that clock sync warnings are displayed
    device_clock_sync_info->set_clock_sync_desired(true);

    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        if(chan > num_rx_channels) {
            std::string message = "Requested rx streamer with channel " + std::to_string(chan) + " but only " + std::to_string(num_rx_channels) + " channels exist";
            UHD_LOG_ERROR(product_name_c, message);
            throw uhd::index_error(message);
        } else if(chan > 1 && is_full_tx) {
            std::string message = "Requested rx streamer with channel " + std::to_string(chan) + " but only rx channels A and B work on a full tx device. Change the rx channel requests or reconfigure Crimson as a non full tx Crimson";
            UHD_LOG_ERROR(product_name_c, message);
            throw uhd::index_error(message);
        }
    }

    if (args.otw_format != "sc16"){
        throw uhd::value_error(product_name_c + " RX cannot handle requested wire format: " + args.otw_format);
    }

    std::vector<std::string> dst_ip(args.channels.size());
    for(size_t n = 0; n < dst_ip.size(); n++) {
        dst_ip[n] = _tree->access<std::string>( rx_link_root(args.channels[n]) + "/ip_dest" ).get();
    }

    std::vector<int> dst_port(args.channels.size());
    for(size_t n = 0; n < dst_port.size(); n++) {
        dst_port[n] = std::stoi(_tree->access<std::string>( rx_link_root(args.channels[n]) + "/port" ).get());
    }

    int data_len = 0;
    // Get vita payload length length (header + data, not including triler)
    for(size_t n = 0; n < args.channels.size(); n++) {
        std::string sfp = _tree->access<std::string>( rx_link_root(args.channels[n]) + "/iface" ).get();
        int payload_len = _tree->access<int>( "/mboards/0/link/" + sfp + "/pay_len" ).get();
        if(data_len == 0) {
            data_len = payload_len - CRIMSON_TNG_HEADER_SIZE;
        }
        // If unable to get length, fallback to hard coded version for variant
        if(data_len + CRIMSON_TNG_HEADER_SIZE != payload_len && data_len !=0) {
            throw uhd::value_error("Payload length mismatch between channels");
        }

        // Verify if the source of rx packets can pinged
        std::string src_ip = _tree->access<std::string>( CRIMSON_TNG_MB_PATH / "link" / sfp / "ip_addr").get();

        if(!ping_check(sfp, src_ip)) {
            UHD_LOG_ERROR(product_name_c, "Unable to ping " + src_ip + " on " + sfp + ". RX channel " + std::to_string(args.channels[n]) + " will not work");
        }
    }

    // Fallback to hard coded values if attempt to get payload fails
    if(data_len == 0) {
        throw uhd::runtime_error("Unable to get payload size from device");
    }

    bool little_endian_supported = true;

    // Create and bind sockets
    // Done here instead of the packet handler class due to the need to auto change ports if the default is busy
    std::vector<int> recv_sockets(args.channels.size());
    for(size_t n = 0; n < dst_port.size(); n++) {
        struct sockaddr_in dst_address;
        memset(&dst_address, 0, sizeof(sockaddr_in));
        int recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(recv_socket_fd < 0) {
            throw uhd::runtime_error( "Failed to create recv socket. Error code:" + std::string(strerror(errno)));
        }

        int bind_r;
        int bind_attempts = 0;
        const int max_band_attempts = 3;
        do {
            dst_address.sin_family = AF_INET;
            dst_address.sin_addr.s_addr = inet_addr(dst_ip[n].c_str());
            dst_address.sin_port = htons(dst_port[n]);

            bind_r = bind(recv_socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address));

            // If a bind error occured pick a new
            if(bind_r < 0 && errno == EADDRINUSE && bind_attempts < max_band_attempts) {
                std::string error_message = "Channel " + std::to_string(args.channels[n]) + " IP address " + dst_ip[n] + " and port " + std::to_string(dst_port[n]) + " is already in use. UHD will change the channel's port and trying again. The most likely causes are either there are multiple instances of UHD running or the OS has not cleaned up a previous UHD program's binds";
                UHD_LOG_INFO(product_name_c, error_message);

                // Pick new port to attempt that shouldn't interfere with other channels
                int desired_new_port = dst_port[n] + 32;
                _tree->access<std::string>( rx_link_root(args.channels[n]) + "/port" ).set(std::to_string(desired_new_port));
                dst_port[n] = std::stoi(_tree->access<std::string>( rx_link_root(args.channels[n]) + "/port" ).get());

            } else if (bind_r < 0) {
                std::string error_message = "Channel " + std::to_string(args.channels[n]) + " bind to IP address " + dst_ip[n] + " and port " + std::to_string(dst_port[n]) + " failed with error code: " + std::string(strerror(errno));
                UHD_LOG_ERROR(product_name_c, error_message);
                throw uhd::io_error( error_message );
            }

            bind_attempts++;
        } while (bind_attempts < max_band_attempts && bind_r < 0);

        recv_sockets[n] = recv_socket_fd;
    }

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];

        const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
        std::string num     = std::string(1, (char)(chan + 'A'));
        const fs_path rx_path   = CRIMSON_TNG_MB_PATH / "rx";
        const fs_path rx_fe_path    = CRIMSON_TNG_MB_PATH / "dboards" / num / "rx_frontends" / ch;
        const fs_path rx_link_path  = CRIMSON_TNG_MB_PATH / "rx_link" / chan;
        const fs_path rx_dsp_path   = CRIMSON_TNG_MB_PATH / "rx_dsps" / chan;

        // stop streaming
        _tree->access<std::string>(rx_path / chan / "stream").set("0");
        if(little_endian_supported) {
            // enables endian swap (by default the packets are big endian, x86 CPUs are little endian)
            _tree->access<int>(rx_link_path / "endian_swap").set(1);
            // Checks if the server accepted the endian swap request
            // If 0 then the device does not support endian swap
            int endian_status = _tree->access<int>(rx_link_path / "endian_swap").get();
            if(endian_status == 0) {
                little_endian_supported = false;
            }
        } else {
            // Don't need to attempt to enable little endian for other channels if one has already failed, since they will all fail
        }
        // vita enable
        _tree->access<std::string>(rx_link_path / "vita_en").set("1");
    }

    // Gets the issuers used by the channels used by this server
    std::vector<uhd::usrp::stream_cmd_issuer> issuers;
    issuers.reserve(args.channels.size());
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];

        issuers.emplace_back(rx_stream_cmd_issuer[chan]);
    }

    // Creates streamer
    // must be done after setting stream to 0 in the state tree so flush works correctly
    std::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer = std::shared_ptr<crimson_tng_recv_packet_streamer>(new crimson_tng_recv_packet_streamer(product_name_c, args.channels, recv_sockets, dst_ip, data_len, args.cpu_format, args.otw_format, little_endian_supported, rx_channel_in_use, num_rx_channels, _mbc.iface, issuers, rx_channel_lock_fd, rx_streaming_lock_fd));

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];

        _mbc.rx_streamers[chan] = my_streamer; //store weak pointer
    }

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];

        const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
        std::string num     = std::string(1, (char)(chan + 'A'));
        const fs_path rx_path   = CRIMSON_TNG_MB_PATH / "rx";
        const fs_path rx_fe_path    = CRIMSON_TNG_MB_PATH / "dboards" / num / "rx_frontends" / ch;
        const fs_path rx_link_path  = CRIMSON_TNG_MB_PATH / "rx_link" / chan;
        const fs_path rx_dsp_path   = CRIMSON_TNG_MB_PATH / "rx_dsps" / chan;

        _tree->access<std::string>(rx_path / chan / "stream").set("0");
        // vita enable
        _tree->access<std::string>(rx_link_path / "vita_en").set("1");

        // power on the channel
        _tree->access<std::string>(rx_path / chan / "pwr").set("1");
        _tree->access<std::string>(rx_path / chan / "stream").set("1");

// TODO: see if this is still required, it probably sets the band since pwr(0) mutes it
#define _update( t, p ) \
_tree->access<t>( p ).set( _tree->access<t>( p ).get() )

        _update( int, rx_fe_path / "freq" / "band" );
    }

    //sets all tick and samp rates on this streamer
    // TODO: only update relevant channels
    this->update_rates();

    return my_streamer;
}

/***********************************************************************
* Transmit streamer
**********************************************************************/

static void get_fifo_lvl_udp_abs( const size_t channel, const int64_t bl_multiple, uhd::transport::udp_simple::sptr xport, double tick_rate, uint64_t & lvl, uint64_t & uflow, uint64_t & oflow, uhd::time_spec_t & now ) {

    double tick_period_ps = 1.0 / tick_rate;

    #pragma pack(push,1)
    struct fifo_lvl_req {
        uint64_t header; // 000000010001CCCC (C := channel bits, x := WZ,RAZ)
        //uint64_t cookie;
    };
    #pragma pack(pop)

    #pragma pack(push,1)
    struct fifo_lvl_rsp {
        uint64_t header; // CCCC00000000FFFF (C := channel bits, F := fifo bits)
        uint64_t oflow;
        uint64_t uflow;
        uint64_t tv_sec;
        uint64_t tv_tick;
        //uint64_t cookie;
    };
    #pragma pack(pop)

    fifo_lvl_req req;
    fifo_lvl_rsp rsp;

    req.header = (uint64_t)0x10001 << 16;
    req.header |= (channel & 0xffff);

    boost::endian::big_to_native_inplace( req.header );

    size_t r = 0;

    for( size_t tries = 0; tries < 100; tries++ ) {
        r = xport->send( boost::asio::mutable_buffer( & req, sizeof( req ) ) );
        if ( sizeof( req ) != r ) {
            continue;
        }

        r = xport->recv( boost::asio::mutable_buffer( & rsp, sizeof( rsp ) ) );
        if ( sizeof( rsp ) != r ) {
            continue;
        }

        boost::endian::big_to_native_inplace( rsp.header );
        if ( channel != ( ( rsp.header >> 48 ) & 0xffff ) ) {
            r = 0;
            continue;
        }

        break;
    }

    if ( 0 == r ) {
        UHD_LOGGER_ERROR("SEND_PACKET_STREAMER") << "Failed to retrieve buffer level for channel " + std::string( 1, 'A' + channel ) + "\nCheck SFP port connections and cofiguration" << std::endl;
        throw new io_error( "Failed to retrieve buffer level for channel " + std::string( 1, 'A' + channel ) );
    }

    boost::endian::big_to_native_inplace( rsp.oflow );
    boost::endian::big_to_native_inplace( rsp.uflow );
    boost::endian::big_to_native_inplace( rsp.tv_sec );
    boost::endian::big_to_native_inplace( rsp.tv_tick );

    //fifo level provided by FPGA
    lvl = rsp.header & 0xffff;

    lvl = lvl * bl_multiple;

#ifdef BUFFER_LVL_DEBUG
    static uint32_t last[4];
    static uint32_t curr[4];
    last[channel] = curr[channel];
    curr[channel] = lvl;

    std::printf("%10u\t", lvl);
    if(channel == 3)
    {
        std::printf("%10u\t", last[0] - curr[0]);
        std::printf("%10u\t", last[1] - curr[1]);
        std::printf("%10u\t", last[2] - curr[2]);
        std::printf("%10u\t", last[3] - curr[3]);

        const uint32_t min = std::min(curr[0], std::min(curr[1], std::min(curr[2], curr[3])));
        const uint32_t max = std::max(curr[0], std::max(curr[1], std::max(curr[2], curr[3])));
        std::printf("%10u\t", max - min);
        std::printf("\n");
    }
#endif

    uflow = rsp.uflow & uint64_t( 0x0fffffffffffffff );
    oflow = rsp.oflow & uint64_t( 0x0fffffffffffffff );

    now = uhd::time_spec_t( rsp.tv_sec, rsp.tv_tick * tick_period_ps );

#ifdef UHD_TXRX_DEBUG_PRINTS
    std::stringstream ss;
    ss
            << now << ": "
            << (char)('A' + channel) << ": "
            << '%' << std::dec << std::setw( 2 ) << std::setfill( ' ' ) << (unsigned)( pcnt * 100 )  << " "
            << std::hex << std::setw( 4 ) << std::setfill( '0' ) << lvl << " "
            << std::hex << std::setw( 16 ) << std::setfill( '0' ) << uflow << " "
            << std::hex << std::setw( 16 ) << std::setfill( '0' ) << oflow << " "
            << std::endl << std::flush;
    std::cout << ss.str();
#endif
}

tx_streamer::sptr crimson_tng_impl::get_tx_stream(const uhd::stream_args_t &args_){
    // Set flag to indicate clock sync is desired so that clock sync warnings are displayed
    device_clock_sync_info->set_clock_sync_desired(true);

    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    if (args.otw_format != "sc16"){
        throw uhd::value_error(product_name_c + " TX cannot handle requested wire format: " + args.otw_format);
    }

    const size_t spp = CRIMSON_TNG_MAX_SEND_SAMPLE_BYTES/convert::get_bytes_per_item(args.otw_format);

    std::vector<std::string> dst_ips(args.channels.size());
    std::vector<int> dst_ports(args.channels.size());
    std::vector<std::string> sfps(args.channels.size());
    for(size_t n = 0; n < args.channels.size(); n++) {
        uint16_t dst_port = 0;
        get_tx_endpoint( _tree, args.channels[n], dst_ips[n], dst_port, sfps[n] );
        dst_ports[n] = dst_port;

        // Verify the destination of tx packets can be pinged
        if(!ping_check(sfps[n], dst_ips[n])) {
            UHD_LOG_ERROR(product_name_c, "Unable to ping " + dst_ips[n] + " on " + sfps[n] + ". TX channel " + std::to_string(args.channels[n]) + " will not work");
        }
    }

    bool little_endian_supported = true;

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        size_t chan = args.channels[ chan_i ];
        const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
        const fs_path tx_path   = CRIMSON_TNG_MB_PATH / "tx";
        const fs_path tx_link_path  = CRIMSON_TNG_MB_PATH / "tx_link" / chan;

        // power on the channel
        _tree->access<std::string>(tx_path / chan / "pwr").set("1");

        if(little_endian_supported) {
            // enables endian swap (by default the packets are big endian, x86 CPUs are little endian)
            _tree->access<int>(tx_link_path / "endian_swap").set(1);
            // Checks if the server accepted the endian swap request
            // If 0 then the device does not support endian swap
            int endian_status = _tree->access<int>(tx_link_path / "endian_swap").get();
            if(endian_status == 0) {
                little_endian_supported = false;
            }
        } else {
            // Don't need to attempt to enable little endian for other channels if one has already failed, since they will all fail
        }

        // vita enable
        _tree->access<std::string>(tx_link_path / "vita_en").set("1");

        // Issue reset request to clear anything from initialization
        _tree->access<double>(tx_dsp_root(chan) + "/rstreq").set(1);
    }

    // Each streamer has its own FIFO buffer that can operate independantly
    // However there is a deprecated function in device for reading async message
    // To handle it, each streamer will have its own buffer and the device recv_async_msg will access the buffer from the most recently created streamer
    _async_msg_fifo = std::shared_ptr<pv_tx_async_msg_queue>(new pv_tx_async_msg_queue(1000)/*Buffer contains 1000 messages*/);
    std::shared_ptr<crimson_tng_send_packet_streamer> my_streamer = std::shared_ptr<crimson_tng_send_packet_streamer>(new crimson_tng_send_packet_streamer( product_name_c, args.channels, spp, CRIMSON_TNG_BUFF_SIZE , dst_ips, dst_ports, (int64_t) (CRIMSON_TNG_BUFF_PERCENT * CRIMSON_TNG_BUFF_SIZE), _master_tick_rate, _async_msg_fifo, args.cpu_format, args.otw_format, little_endian_supported, tx_channel_in_use, _mbc.iface, device_clock_sync_info, tx_channel_lock_fd, tx_streaming_lock_fd ));

    //init some streamer stuff
    my_streamer->resize(args.channels.size());

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];

        my_streamer->set_channel_name(chan_i,std::string( 1, 'A' + chan ));

        // Sets the function used to get the buffer level, overflow, and underflow counts
        // NOTE: when passing pointer to this function make sure they are smark pointers
        my_streamer->set_xport_chan_fifo_lvl_abs(chan_i, std::bind(
            &get_fifo_lvl_udp_abs, chan, CRIMSON_TNG_BUFF_SCALE, _mbc.fifo_ctrl_xports[chan], _master_tick_rate, ph::_1, ph::_2, ph::_3, ph::_4
        ));

        _mbc.tx_streamers[chan] = my_streamer; //store weak pointer
    }

    //sets all tick and samp rates on this streamer
    // TODO: only update relevant channels
    this->update_rates();

    // Clock sync takes time and some programs assume send will work quickly instead of having to wait for clock sync to finish, to avoid causing issues with those programs wait for lock sync before returning
    device_clock_sync_info->wait_for_sync();

    return my_streamer;
}
