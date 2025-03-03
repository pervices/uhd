//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

// Header file for classes related to clock sync between the host and device
// WIP: the clock sync loop itself is currently done by bm_thread_fn in the device's respective impl files

#pragma once

// Fixed width numbers
#include <stdint.h>
// Smart pointers
#include <memory>
// UDP socket to send commands to
#include <uhd/transport/udp_simple.hpp>

// UHD types
#include <uhd/types/stream_cmd.hpp>

namespace uhd { namespace usrp {

#pragma pack(push,1)
// Struct corresponding to the packet to be sent to the device
struct rx_stream_cmd {
    uint64_t header;   // 0x10000 for RX SoB
    int64_t tv_sec;    // when the SoB should take place
    int64_t tv_psec;   // when the SoB should take place (ps)
    uint64_t nsamples;
};
#pragma pack(pop)

// This class is responsible for sending rx stream commands
// Purpose of this class:
// -Consolidate identical code between Cyan and Crimson
// -Be callable from both the device itself and the streamer
// -Each issuer is responsible for 1 channel
class stream_cmd_issuer
{
private:
    std::shared_ptr<uhd::transport::udp_simple> command_socket;

    // Channel/JESD number this instance corresponds to
    // Channel on Crimson, JESD on Cyan
    size_t ch_jesd_number;

    // Number of bits in half a complex pair
    size_t num_rx_bits;

public:
    // TODO: make make_rx_stream_cmd_packet private once the functions that depend on it have been moved to this class
    /**
     * Creates the rx command packet.
     * @param cmd The command to be sent. (Read this)
     * @param channel Channel number for Crimson, JESD number for Cyan
     * @param pkt The packet to be send. (Write to this)
     */
    static void make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, const size_t channel, uhd::usrp::rx_stream_cmd & pkt );

    // Issues the packet
    // TODO: make this non static and use the class's command socket isntance
    static void send_command_packet( const rx_stream_cmd & req, const std::shared_ptr<uhd::transport::udp_simple> command_socket);

    // Regular constructor
    stream_cmd_issuer(std::shared_ptr<uhd::transport::udp_simple> command_socket, size_t ch_jesd_number, size_t num_rx_bits);

    // Empty constructor
    stream_cmd_issuer()
    : command_socket(nullptr),
    ch_jesd_number(0),
    num_rx_bits(0)
    {

    }

    // Copy constructor
    stream_cmd_issuer(stream_cmd_issuer& from);

};


}} // namespace uhd::usrp
