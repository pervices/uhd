//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/common/stream_cmd_issuer.hpp>

// For sending formatted error and warning message
#include <uhd/utils/log.hpp>
// uhd::get_system_time()
#include <uhdlib/utils/system_time.hpp>

// Helper includes used by make_rx_stream_cmd_packet
#include "boost/tuple/tuple.hpp"
#include "boost/assign/list_of.hpp"
#include <uhd/types/dict.hpp>
#include <boost/endian/conversion.hpp>

using namespace uhd;
using namespace uhd::usrp;

void stream_cmd_issuer::make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, uhd::usrp::rx_stream_cmd & pkt ) {

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
    pkt.header = ( 0x1 << channel_bits ) | ( ch_jesd_number & channel_mask );

    //setup the instruction flag values
    bool inst_reload, inst_chain, inst_samps, inst_stop;
    boost::tie(inst_reload, inst_chain, inst_samps, inst_stop) = mode_to_inst[cmd.stream_mode];

    pkt.header |= inst_reload ? ( 0b1000LL << 36 ) : 0;
    pkt.header |= inst_chain  ? ( 0b0100LL << 36 ) : 0;
    pkt.header |= inst_samps  ? ( 0b0010LL << 36 ) : 0;
    pkt.header |= inst_stop   ? ( 0b0001LL << 36 ) : 0;

    uhd::time_spec_t ts = cmd.stream_now ? 0.0 : cmd.time_spec;

    pkt.tv_sec = ts.get_full_secs();
    pkt.tv_psec = ts.get_frac_secs() * 1e12;

    pkt.nsamples = inst_samps ? cmd.num_samps : 0;

    // std::cout << "header: " << std::hex << std::setw( 16 ) << std::setfill('0') << pkt.header << std::endl;
    // std::cout << "tv_sec: " << std::dec << pkt.tv_sec << std::endl;
    // std::cout << "tv_psec: " << std::dec << pkt.tv_psec << std::endl;
    // std::cout << "nsampls: " << std::dec << pkt.nsamples << std::endl;

    boost::endian::native_to_big_inplace( pkt.header );
    boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_sec );
    boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_psec );
    boost::endian::native_to_big_inplace( (uint64_t &) pkt.nsamples );
}

void stream_cmd_issuer::issue_stream_command( stream_cmd_t stream_cmd ) {
    // The number of samples requested must be a multiple of a certain number, depending on the variant
    uint64_t original_nsamps_req = stream_cmd.num_samps;
    stream_cmd.num_samps = (original_nsamps_req / nsamps_multiple_rx) * nsamps_multiple_rx;
    if(original_nsamps_req != stream_cmd.num_samps) {
        // Effectively always round up
        stream_cmd.num_samps+=nsamps_multiple_rx;
        if(stream_cmd.stream_mode != uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS) {
            UHD_LOGGER_WARNING("STREAM_CMD_ISSUER") << "Number of samples requested must be multiple of " << nsamps_multiple_rx << ". The number of samples requested has been modified to " << stream_cmd.num_samps << std::endl;
        }
    }

    // The part of the FPGA that tracks how many samples are sent is hard coded to assume sc16
    // Therefore, we need to actually request a number of samples with the same amount of data if it were sc16 as what we actually want
    // i.e. sc12 contains 3/4 the amount of data as sc16, so multiply by 3/4
    stream_cmd.num_samps = stream_cmd.num_samps * num_rx_bits / 16;

    if(!clock_sync_info->is_synced()) [[unlikely]] {
        // TODO: get time from server instead of waiting for sync if not already synced
        clock_sync_info->wait_for_sync();
    }
    double current_time = (uhd::get_system_time() + clock_sync_info->get_time_diff()).get_real_secs();

#ifdef DEBUG_COUT
    std::cout
        << std::fixed << std::setprecision(6)
        << current_time
        << ": "
        << stream_cmd.stream_mode
        << ": "
        << pre
        << ": SETTING STREAM COMMAND: "
        << stream_cmd.num_samps << ": "
        << stream_cmd.stream_now << ": "
        << stream_cmd.time_spec.get_real_secs() << std::endl;
#endif

    uint8_t packet_buffer2[256];
    memset(packet_buffer2, 0, 256);
	uhd::usrp::rx_stream_cmd* rx_stream_cmd = (uhd::usrp::rx_stream_cmd*) packet_buffer2;

    if (stream_cmd.time_spec.get_real_secs() < current_time + 0.01 && stream_cmd.stream_mode != uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS && !stream_cmd.stream_now) {
        UHD_LOGGER_WARNING("STREAM_CMD_ISSUER") << "Requested rx start time of " + std::to_string(stream_cmd.time_spec.get_real_secs()) + " close to current device time of " + std::to_string(current_time) + ". Ignoring start time and enabling stream_now";
        stream_cmd.stream_now = true;
    }

    // Due to FPGA issues send a stop command before the actual command
    // The issue is most likely to occur with 1Gsps JESD and 100G SFP
    // See issue 14110 note 27
    uhd::stream_cmd_t clear_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    clear_stream_cmd.num_samps  = 0;
    clear_stream_cmd.stream_now = stream_cmd.stream_now;
    clear_stream_cmd.time_spec  = stream_cmd.time_spec;
    uint8_t packet_buffer1[256];
    memset(packet_buffer1, 0, 256);
    uhd::usrp::rx_stream_cmd* clear_rx_stream_cmd_packet = (uhd::usrp::rx_stream_cmd*) packet_buffer1;
    uhd::usrp::stream_cmd_issuer::make_rx_stream_cmd_packet( clear_stream_cmd, *clear_rx_stream_cmd_packet );

    // Conver the user provided struct to a packet
    uhd::usrp::stream_cmd_issuer::make_rx_stream_cmd_packet( stream_cmd, *rx_stream_cmd );

    // command_socket->send( packet_buffer1, 512 );
    command_socket->send( packet_buffer2, 256 );
}

stream_cmd_issuer::stream_cmd_issuer(std::shared_ptr<uhd::transport::udp_simple> command_socket, std::shared_ptr<uhd::usrp::clock_sync_shared_info> clock_sync_info, size_t ch_jesd_number, size_t num_rx_bits, size_t nsamps_multiple_rx)
: command_socket(command_socket),
clock_sync_info(clock_sync_info),
ch_jesd_number(ch_jesd_number),
num_rx_bits(num_rx_bits),
nsamps_multiple_rx(nsamps_multiple_rx)
{

}

stream_cmd_issuer::stream_cmd_issuer()
:
ch_jesd_number(0),
num_rx_bits(0),
nsamps_multiple_rx(0)
{

}

stream_cmd_issuer::stream_cmd_issuer(const stream_cmd_issuer& other)
:
ch_jesd_number(other.ch_jesd_number),
num_rx_bits(other.num_rx_bits),
nsamps_multiple_rx(other.nsamps_multiple_rx)
{
    if(other.command_socket) {
        command_socket = other.command_socket;
    }
    if(other.clock_sync_info) {
        clock_sync_info = other.clock_sync_info;
    }
}

stream_cmd_issuer& stream_cmd_issuer::operator=(stream_cmd_issuer&& other) {
    if(other.command_socket) {
        command_socket = other.command_socket;
    }
    if(other.clock_sync_info) {
        clock_sync_info = other.clock_sync_info;
    }
    ch_jesd_number = other.ch_jesd_number;
    num_rx_bits = other.num_rx_bits;
    nsamps_multiple_rx = other.nsamps_multiple_rx;

    return *this;
}

stream_cmd_issuer& stream_cmd_issuer::operator=(const stream_cmd_issuer& other) {
    if(other.command_socket) {
        command_socket = other.command_socket;
    }
    if(other.clock_sync_info) {
        clock_sync_info = other.clock_sync_info;
    }
    ch_jesd_number = other.ch_jesd_number;
    num_rx_bits = other.num_rx_bits;
    nsamps_multiple_rx = other.nsamps_multiple_rx;

    return *this;
}
