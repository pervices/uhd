//
// Copyright 2024 Per Vices Corporation
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

#ifndef INCLUDED_PV_DEVICE_RECV_PACKET_STREAMER_HPP
#define INCLUDED_PV_DEVICE_RECV_PACKET_STREAMER_HPP

#include <uhdlib/transport/super_recv_packet_handler_mmsg.hpp>
#include <uhdlib/usrp/common/pv_iface.hpp>

namespace uhd {
namespace usrp {

/**
 * Base class containing the state common to all Per Vices device receive
 * packet streamers (e.g. crimson_tng_recv_packet_streamer and
 * cyan_nrnt_recv_packet_streamer).
 */
class pv_device_recv_packet_streamer : public uhd::transport::sph::recv_packet_streamer_mmsg
{
public:

    pv_device_recv_packet_streamer(
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
    )
    : uhd::transport::sph::recv_packet_streamer_mmsg(channels, recv_sockets, dst_ip, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian, device_total_rx_channels, cmd_issuer, streaming_locks),
    _channels(channels),
    _rx_streamer_channel_in_use(rx_channel_in_use),
    _channel_locks(channel_locks),
    _streaming_locks(streaming_locks),
    _iface(iface)
    {}

    virtual ~pv_device_recv_packet_streamer() = default;

protected:

    std::vector<size_t> _channels;
    std::shared_ptr<std::vector<bool>> _rx_streamer_channel_in_use;
    // Indicates a streamer has already been created for a channel
    std::vector<int> _channel_locks;
    // Indicates a channel is actively streaming
    std::vector<int> _streaming_locks;

    /**
    * A shared pointer to the interface used to access the server.
    * When using this to access properties use the actual path on the server and use the get function in pv_iface instead of the mapping and access command from the property tree
    */
    pv_iface::sptr _iface;
};

}
}

#endif /* INCLUDED_PV_DEVICE_RECV_PACKET_STREAMER_HPP */
