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

#ifndef INCLUDED_CRIMSON_TNG_IO_IMPL_HPP
#define INCLUDED_CRIMSON_TNG_IO_IMPL_HPP

#include "../pv_device/pv_device_io_impl.hpp"

namespace uhd {
namespace usrp {

class crimson_tng_recv_packet_streamer : public pv_device_recv_packet_streamer
{
public:

    /**
    * @param iface The interface to access thes server
    */
    crimson_tng_recv_packet_streamer(const std::string product_name_c, const std::vector<size_t> channels, const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian,  std::shared_ptr<std::vector<bool>> rx_channel_in_use, size_t device_total_rx_channels, pv_iface::sptr iface, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuer, std::vector<int> channel_locks, std::vector<int> streaming_locks);

    ~crimson_tng_recv_packet_streamer();

    void if_hdr_unpack(const uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info);

    void teardown();

};

class crimson_tng_send_packet_streamer : public pv_device_send_packet_streamer {
public:

    typedef std::function<uhd::time_spec_t(void)> timenow_type;
    typedef std::function<void(uint64_t&,uint64_t&,uint64_t&,uhd::time_spec_t&)> xport_chan_fifo_lvl_abs_type;

    /**
    * @param iface The interface to access thes server
    */
    crimson_tng_send_packet_streamer(const std::string product_name_c, const std::vector<size_t>& channels, const size_t max_num_samps, const size_t max_bl, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, double tick_rate, const std::shared_ptr<uhd::pv_tx_async_msg_queue> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<std::vector<bool>> tx_channel_in_use, pv_iface::sptr iface, std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info, std::vector<int> channel_locks, std::vector<int> streaming_locks);

    ~crimson_tng_send_packet_streamer();

    void teardown();

    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    );

    // Calls the function from the device to get the time on the device if it has been set, otherwise get's the host's system time
    uhd::time_spec_t get_time_now();

    void set_xport_chan_fifo_lvl_abs( size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs );

    void set_channel_name( size_t chan, std::string name );
    void sync_channel_rate ( size_t chan, double rate);

    void resize(const size_t size);

    void check_matching_rates() override;
};

}
}

#endif /* INCLUDED_CRIMSON_TNG_IO_IMPL_HPP */
