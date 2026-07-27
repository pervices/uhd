//
// Copyright 2024-2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhdlib/transport/super_recv_packet_handler_mmsg.hpp>
#include <uhdlib/usrp/common/pv_iface.hpp>

namespace uhd {
namespace usrp {

/**
 * @brief Receive packet streamer shared by Per Vices devices.
 *
 * Used by Crimson TNG, Cyan NRNT, Chestnut, and other Per Vices devices.
 */
class pv_device_recv_packet_streamer : public uhd::transport::sph::recv_packet_streamer_mmsg
{
public:

    /**
     * @brief Construct a receive packet streamer for Per Vices devices.
     *
     * @param product_name_c Product name in capital letters, used for log messages only.
     * @param channels Channel indices included in this streamer.
     * @param recv_sockets UDP socket file descriptors used to receive packets.
     * @param dst_ip Destination IP addresses for each channel.
     * @param max_sample_bytes_per_packet Maximum sample payload size per packet, in bytes.
     * @param header_size VRT header size, in bytes.
     * @param trailer_size VRT trailer size, in bytes.
     * @param cpu_format Format string specifying the format of samples to output (host).
     * @param wire_format Format string specifying the format of incoming samples (over the write).
     * @param wire_little_endian Whether wire-format samples are little-endian.
     * @param rx_channel_in_use Shared channel-in-use flags for rate/overrun checks.
     * @param device_total_rx_channels Total number of RX channels on the device.
     * @param iface Interface used to access the device server.
     * @param cmd_issuer Stream command issuer for each channel.
     * @param channel_locks Advisory lock file descriptors for if a channels is owned by a streamer.
     * @param streaming_locks Advisory lock file descriptors for if a channel is actively streaming.
     */
    pv_device_recv_packet_streamer(
        const std::string product_name_c,
        const std::vector<size_t> channels,
        const std::vector<int>& recv_sockets,
        const std::vector<std::string>& dst_ip,
        const size_t max_sample_bytes_per_packet,
        const size_t header_size,
        const size_t trailer_size,
        const std::string& cpu_format,
        const std::string& wire_format,
        bool wire_little_endian,
        std::shared_ptr<std::vector<bool>> rx_channel_in_use,
        size_t device_total_rx_channels,
        pv_iface::sptr iface,
        std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuer,
        std::vector<int> channel_locks,
        std::vector<int> streaming_locks
    );

    /**
     * @brief Deactivates rx channels and free advisory locks
     */
    virtual ~pv_device_recv_packet_streamer();

    /**
     * @brief Unpack a received VRT IF packet header.
     *
     * @param packet_buff Raw packet buffer containing the header.
     * @param if_packet_info The struct to unpack the header into
     */
    void if_hdr_unpack(const uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info) override;

protected:

    /** @brief Product name in all capitals, used for user-facing messages only. */
    const std::string _product_name_c;

    /**
     * @brief Channel indices owned by this streamer.
     *
     * Be very careful to keep track of whether you should be using a channel index with respect to the device
     * Or a channel index with respect to the streamer. This maps the index within the streamer to the device
     */
    std::vector<size_t> _channels;

    /**
     * @brief Shared flags indicating which RX channels are currently in use
     *
     * It relies on the device channel index, not the streamer channel index
     */
    std::shared_ptr<std::vector<bool>> _rx_streamer_channel_in_use;

    /**
     * @brief Advisory lock file descriptors indicating a streamer exists for a channel.
     *
     * It relies on the device channel index, not the streamer channel index
     */
    std::vector<int> _channel_locks;

    /**
     * @brief Advisory lock file descriptors indicating a channel is actively streaming.
     *
     * It relies on the device channel index, not the streamer channel index
     */
    std::vector<int> _streaming_locks;

    /**
     * @brief Shared pointer to the interface used to access the device server.
     *
     * Use server-side property paths with pv_iface get/set functions rather than
     * property-tree mappings.
     */
    pv_iface::sptr _iface;
};

}
}
