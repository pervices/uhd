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

#ifndef INCLUDED_CYAN_NRNT_IO_IMPL_HPP
#define INCLUDED_CYAN_NRNT_IO_IMPL_HPP

#include "../../transport/super_recv_packet_handler_mmsg.cpp"
#include "../../transport/super_send_packet_handler_mmsg.cpp"

#include <uhdlib/usrp/common/pv_iface.hpp>

namespace uhd {
namespace usrp {

class cyan_nrnt_recv_packet_streamer : public uhd::transport::sph::recv_packet_streamer_mmsg
{
public:

    /**
     * @param iface The interface to access thes server
     */
    cyan_nrnt_recv_packet_streamer(const std::vector<size_t> channels, const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian,  std::shared_ptr<std::vector<bool>> rx_channel_in_use, size_t device_total_rx_channels, pv_iface::sptr iface, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuer);

    ~cyan_nrnt_recv_packet_streamer();

    void resize(const size_t size);

    void if_hdr_unpack(const uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info);

    void teardown();

private:

    std::vector<size_t> _channels;
    std::shared_ptr<std::vector<bool>> _rx_streamer_channel_in_use;

    /**
     * A shared pointer to the interface used to access the server.
     * When using this to access properties use the actual path on the server and use the get function in pv_iface instead of the mapping and access command from the property tree
     */
    pv_iface::sptr _iface;
};

class cyan_nrnt_send_packet_streamer : public uhd::transport::sph::send_packet_streamer_mmsg {
public:

    typedef std::function<uhd::time_spec_t(void)> timenow_type;
    typedef std::function<void(uint64_t&,uint64_t&,uint64_t&,uhd::time_spec_t&)> xport_chan_fifo_lvl_abs_type;

    /**
     * @param iface The interface to access thes server
     */
    cyan_nrnt_send_packet_streamer(const std::vector<size_t>& channels, const size_t max_num_samps, const size_t max_bl, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, const std::shared_ptr<uhd::transport::bounded_buffer<async_metadata_t>> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<std::vector<bool>> tx_channel_in_use, pv_iface::sptr iface, std::shared_ptr<uhd::usrp::clock_sync_shared_info> clock_sync_info);

    ~cyan_nrnt_send_packet_streamer();

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

    void resize(const size_t size);

    // Starts buffer monitor thread if it is not already running
	inline void start_buffer_monitor_thread() {
        _stop_buffer_monitor = false;

        //spawn a thread to monitor the buffer level
        _buffer_monitor_thread = std::thread( cyan_nrnt_send_packet_streamer::buffer_monitor_loop, this );
        _buffer_monitor_running = true;
	}

	void stop_buffer_monitor_thread();

protected:
    void if_hdr_pack(uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info);

    int64_t get_buffer_level_from_device(const size_t ch_i);

private:
	bool _first_call_to_send;
    bool _buffer_monitor_running;
    bool _stop_buffer_monitor;
    std::thread _buffer_monitor_thread;
    timenow_type _time_now;

    // extended per-channel properties, beyond what is available in sphc::send_packet_handler::xport_chan_props_type
    struct eprops_type{
		uhd::transport::zero_copy_if::sptr xport_chan;
        xport_chan_fifo_lvl_abs_type xport_chan_fifo_lvl_abs;
		uint64_t oflow;
		uint64_t uflow;
        /**
         * Upper case channel letter.
         */
        std::string name;
        eprops_type() : oflow( -1 ), uflow( -1 ) {}
        eprops_type( const eprops_type & other )
        :
            xport_chan( other.xport_chan ),
            oflow( other.oflow ),
            uflow( other.uflow )
        {}
    };
    std::vector<eprops_type> _eprops;

    std::shared_ptr<std::vector<bool>> _tx_streamer_channel_in_use;

    bool _performance_warning_printed = false;

    /**
     * A shared pointer to the interface used to access the server.
     * When using this to access properties use the actual path on the server and use the get function in pv_iface instead of the mapping and access command from the property tree
     */
    pv_iface::sptr _iface;

    /***********************************************************************
     * buffer_monitor_loop
     * - DOES NOT update predicted buffer levels: predicted buffer level is based entirely on time. Having timestamps on every packet fixes dropped packets, and time diffs calculates the latency of sending data over the SFP. The benefit of having this update the buffer level is non-existent, the penalty of the inter-thread communication updating the bias is very significant when using no DDR mode
     * - update over / underflow counters
     * - put async message packets into queue
     **********************************************************************/
	static void buffer_monitor_loop( cyan_nrnt_send_packet_streamer *self );
};

}
}

#endif /* INCLUDED_CYAN_NRNT_IO_IMPL_HPP */
