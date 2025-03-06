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
// Used to check the time to decide whether or not to display a start time warning
#include <uhdlib/usrp/common/clock_sync.hpp>

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
    // UDP socket manager to send commands over
    std::shared_ptr<uhd::transport::udp_simple> command_socket;

    // Clock sync info used to check the time
    // std::shared_ptr<uhd::usrp::clock_sync_shared_info> clock_sync_info;

    // Channel/JESD number this instance corresponds to
    // Channel on Crimson, JESD on Cyan
    size_t ch_jesd_number;

    // Number of bits in half a complex pair
    size_t num_rx_bits;

    // Depending on the device, the requested number of samples might need to be a multiple of this
    size_t nsamps_multiple_rx;

    /**
     * Helper function to convert the command struct used by UHD internally the packet format
     * @param cmd The command to be sent. (Read this)
     * @param pkt The packet to be send. (Write to this)
     */
    void make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, uhd::usrp::rx_stream_cmd & pkt );

public:

    /**
     * Sends the stream command
     * @param stream_cmd The struct containing the command to send
     */
    void issue_stream_command( stream_cmd_t stream_cmd );

    // Regular constructor
    stream_cmd_issuer(std::shared_ptr<uhd::transport::udp_simple> command_socket_, std::shared_ptr<uhd::usrp::clock_sync_shared_info>& clock_sync_info_, size_t ch_jesd_number, size_t num_rx_bits, size_t nsamps_multiple_rx);

    // Empty constructor
    stream_cmd_issuer();

    // Copy constructor
    stream_cmd_issuer(const stream_cmd_issuer& from);

    // Move operator
    stream_cmd_issuer& operator=(stream_cmd_issuer&& other);

    // Copy operator
    stream_cmd_issuer& operator=(const stream_cmd_issuer& other);

};


}} // namespace uhd::usrp
