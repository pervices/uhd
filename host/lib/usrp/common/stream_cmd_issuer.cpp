//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/common/stream_cmd_issuer.hpp>

// For sending formatted error and warning message
#include <uhd/utils/log.hpp>

// Helper includes used by make_rx_stream_cmd_packet
#include "boost/tuple/tuple.hpp"
#include "boost/assign/list_of.hpp"
#include <uhd/types/dict.hpp>
#include <boost/endian/conversion.hpp>

using namespace uhd;
using namespace uhd::usrp;

void stream_cmd_issuer::make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, const size_t channel, uhd::usrp::rx_stream_cmd & pkt ) {

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

void stream_cmd_issuer::send_command_packet( const rx_stream_cmd & req, const std::shared_ptr<uhd::transport::udp_simple> command_socket) {
    command_socket->send( &req, sizeof( req ) );
}

stream_cmd_issuer::stream_cmd_issuer(std::shared_ptr<uhd::transport::udp_simple> command_socket, size_t ch_jesd_number, size_t num_rx_bits)
: command_socket(command_socket),
ch_jesd_number(ch_jesd_number),
num_rx_bits(num_rx_bits)
{

}

stream_cmd_issuer::stream_cmd_issuer(stream_cmd_issuer& from)
: command_socket(from.command_socket),
ch_jesd_number(from.ch_jesd_number),
num_rx_bits(from.num_rx_bits)
{

}
