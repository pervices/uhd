#ifndef INCLUDED_PV_DEVICE_IO_IMPL_HPP
#define INCLUDED_PV_DEVICE_IO_IMPL_HPP

#include <uhdlib/transport/super_recv_packet_handler_mmsg.hpp>
#include <uhdlib/transport/super_send_packet_handler_mmsg.hpp>

#include <uhdlib/usrp/common/pv_iface.hpp>
#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>

namespace uhd {
namespace usrp {

class pv_device_recv_packet_streamer : public uhd::transport::sph::recv_packet_streamer_mmsg
{
public:
    pv_device_recv_packet_streamer(const std::string& log_id,
        const std::vector<size_t>& channels,
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
        std::vector<int> streaming_locks);

    virtual ~pv_device_recv_packet_streamer();
    void if_hdr_unpack(const uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info);
    virtual void teardown();

protected:
    std::string _log_id;
    std::shared_ptr<std::vector<bool>> _rx_streamer_channel_in_use;
    std::vector<int> _channel_locks;
    std::vector<int> _streaming_locks;
    pv_iface::sptr _iface;
};

class pv_device_send_packet_streamer : public uhd::transport::sph::send_packet_streamer_mmsg {
public:
    typedef std::function<uhd::time_spec_t(void)> timenow_type;
    typedef std::function<void(uint64_t&, uint64_t&, uint64_t&, uhd::time_spec_t&)> xport_chan_fifo_lvl_abs_type;

    struct eprops_type {
        uhd::transport::zero_copy_if::sptr xport_chan;
        xport_chan_fifo_lvl_abs_type xport_chan_fifo_lvl_abs;
        uint64_t oflow;
        uint64_t uflow;
        std::string name;
        double sample_rate;
        eprops_type() : oflow(-1), uflow(-1), sample_rate(0.0) {}
        eprops_type(const eprops_type& other)
            : xport_chan(other.xport_chan)
            , xport_chan_fifo_lvl_abs(other.xport_chan_fifo_lvl_abs)
            , oflow(other.oflow)
            , uflow(other.uflow)
            , name(other.name)
            , sample_rate(other.sample_rate)
        {}
    };

    pv_device_send_packet_streamer(const std::string& log_id,
        const std::vector<size_t>& channels,
        const size_t max_num_samps,
        const size_t max_bl,
        std::vector<std::string>& dst_ips,
        std::vector<int>& dst_ports,
        int64_t device_target_nsamps,
        const size_t nsamp_multiple,
        double tick_rate,
        double min_tx_delay,
        const size_t update_per_sec,
        const std::shared_ptr<uhd::pv_tx_async_msg_queue> async_msg_fifo,
        const std::string& cpu_format,
        const std::string& wire_format,
        bool wire_little_endian,
        std::shared_ptr<std::vector<bool>> tx_channel_in_use,
        pv_iface::sptr iface,
        std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info,
        std::vector<int> channel_locks,
        std::vector<int> streaming_locks);

    virtual ~pv_device_send_packet_streamer();
    virtual void teardown();
    size_t send(const tx_streamer::buffs_type& buffs, const size_t nsamps_per_buff, const uhd::tx_metadata_t& metadata_, const double timeout);
    void set_xport_chan_fifo_lvl_abs(size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs);
    void set_channel_name(size_t chan, std::string name);
    virtual void sync_channel_rate(size_t chan, double rate);
    void resize(const size_t size);
    void stop_buffer_monitor_thread();

protected:
    void if_hdr_pack(uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info);
    int64_t get_buffer_level_from_device(const size_t ch_i);
    virtual void check_matching_rates();
    static void buffer_monitor_loop(pv_device_send_packet_streamer* self);

    double _min_tx_delay;
    size_t _update_per_sec;
    bool _first_call_to_send;
    std::atomic<bool> _buffer_monitor_running;
    std::atomic<bool> _stop_buffer_monitor;
    std::thread _buffer_monitor_thread;
    timenow_type _time_now;
    uint16_t _sfp_oflow_start;
    uint16_t _max_sfp_oflow_count;
    std::vector<eprops_type> _eprops;
    std::shared_ptr<std::vector<bool>> _tx_streamer_channel_in_use;
    std::vector<int> _channel_locks;
    bool _performance_warning_printed = false;
    std::string _log_id;
    pv_iface::sptr _iface;
};

}
}

#endif
