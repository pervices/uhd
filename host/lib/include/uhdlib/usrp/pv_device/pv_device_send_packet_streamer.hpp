//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Per Vices Corporation
// Copyright 2022-2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhdlib/transport/super_send_packet_handler_mmsg.hpp>

#include <uhdlib/usrp/common/pv_iface.hpp>
#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>

namespace uhd {
namespace usrp {

/**
 * @brief Transmit packet streamer shared by Per Vices devices.
 *
 * Used by Crimson TNG, Cyan NRNT, Chestnut, and other Per Vices devices.
 */
class pv_device_send_packet_streamer : public uhd::transport::sph::send_packet_streamer_mmsg {
public:

    /** @brief Callback that queries absolute FIFO level and overflow counters for a channel. */
    typedef std::function<void(uint64_t&, uint64_t&, uint64_t&, uhd::time_spec_t&)> xport_chan_fifo_lvl_abs_type;

    /**
     * @brief Construct a Per Vices device transmit packet streamer.
     *
     * @param product_name_c Product name in capital letters, used for log messages only.
     * @param channels Channel indices included in this streamer.
     * @param max_num_samps Maximum number of samples per packet.
     * @param max_bl Maximum device buffer level, in samples.
     * @param dst_ips Destination IP address for each channel.
     * @param dst_ports Destination UDP port for each channel.
     * @param device_target_nsamps Target buffer level used by flow control, in samples.
     * @param nsamp_multiple Packet sample count must be a multiple of this value.
     * @param tick_rate Device tick rate used to convert time specs to/from ticks.
     * @param min_tx_delay Minimum lead time required for a timed burst start, in seconds.
     * @param update_per_sec Buffer monitor poll rate, in updates per second.
     * @param async_msg_fifo Queue used to deliver TX async metadata messages.
     * @param cpu_format Format string specifying the format of samples of samples from the user (host).
     * @param wire_format Format string specifying the format of outgoing samples (over the write).
     * @param wire_little_endian Whether wire-format samples are little-endian.
     * @param tx_channel_in_use Shared channel-in-use flags for rate/overrun checks.
     * @param iface Interface used to access the device server.
     * @param clock_sync_info Shared clock synchronization state for the device.
     * @param channel_locks Advisory lock file descriptors for if a channels is owned by a streamer.
     * @param streaming_locks Advisory lock file descriptors for if a channel is actively streaming.
     */
    pv_device_send_packet_streamer(
        const std::string product_name_c,
        const std::vector<size_t>& channels,
        const size_t max_num_samps,
        const size_t max_bl,
        std::vector<std::string>& dst_ips,
        std::vector<int>& dst_ports,
        int64_t device_target_nsamps,
        const size_t nsamp_multiple,
        double tick_rate,
        double min_tx_delay,
        double update_per_sec,
        const std::shared_ptr<uhd::pv_tx_async_msg_queue> async_msg_fifo,
        const std::string& cpu_format,
        const std::string& wire_format,
        bool wire_little_endian,
        std::shared_ptr<std::vector<bool>> tx_channel_in_use,
        pv_iface::sptr iface,
        std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info,
        std::vector<int> channel_locks,
        std::vector<int> streaming_locks
    );

    /** @brief Stop streaming and release resources for all channels in this streamer. */
    ~pv_device_send_packet_streamer();

    /**
     * @brief Wrapper for the destructor
     *
     * Stop the buffer monitor thread and shut down all TX channels.
     * Stops the buffer monitor thread, waits for device buffers to drain,
     * reports overflow/underflow counts, and releases advisory channel locks.
     */
    void teardown();

    /**
     * @brief Send samples on all channels in this streamer.
     *
     * @param buffs Sample buffers for each channel.
     * @param nsamps_per_buff Number of samples to send from each buffer.
     * @param metadata_ A struct containing metadata about the stream.
     * @param timeout Timeout for the send operation, in seconds.
     * @return Number of samples sent per channel.
     */
    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    );

    /**
     * @brief Register a FIFO level query callback for a streamer channel.
     *
     * @param chan Streamer-local channel index.
     * @param get_fifo_lvl_abs Callback used to read buffer level and overflow state.
     */
    void set_xport_chan_fifo_lvl_abs(size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs);

    /**
     * @brief Set the display name for a streamer channel.
     *
     * @param chan Streamer-local channel index.
     * @param name Upper-case channel letter used in messages.
     */
    void set_channel_name(size_t chan, std::string name);

    /**
     * @brief Store the sample rate for a device channel in this streamer.
     *
     * @param chan Device channel index.
     * @param rate Sample rate in samples per second.
     */
    void sync_channel_rate(size_t chan, double rate);

    /**
     * @brief Resize the per-channel extended property table.
     *
     * @param size Number of channels in this streamer.
     */
    void resize(const size_t size);

    /** @brief Start the background buffer monitor thread if it is not already running. */
    inline void start_buffer_monitor_thread() {
        _stop_buffer_monitor.store(false, std::memory_order_relaxed);

        //spawn a thread to monitor the buffer level
        _buffer_monitor_thread = std::thread( pv_device_send_packet_streamer::buffer_monitor_loop, this );
        _buffer_monitor_running.store(true, std::memory_order_relaxed);
    }

    /** @brief Stop and join the background buffer monitor thread. */
    void stop_buffer_monitor_thread();

    /**
     * @brief Verify that all channels in this streamer share the same sample rate.
     *
     * Throws if a mismatch is detected.
     */
    void check_matching_rates();

protected:
    /**
     * @brief Pack a VRT IF packet header into a transmit buffer.
     *
     * @param packet_buff Destination buffer for the packed header.
     * @param if_packet_info Header fields to pack.
     */
    void if_hdr_pack(uint32_t* packet_buff, uhd::transport::vrt::if_packet_info_t& if_packet_info);

    /**
     * @brief Query the current buffer level for a streamer channel.
     *
     * @param ch_i Streamer-local channel index.
     * @return Current buffer level reported by the device.
     */
    int64_t get_buffer_level_from_device(const size_t ch_i);

private:
    /** @brief Product name in all capitals, used for user-facing messages only. */
    const std::string _product_name_c;

    /** @brief Minimum lead time required for a timed burst start, in seconds. */
    const double _min_tx_delay;

    /** @brief Buffer monitor poll rate, in updates per second. */
    const double _update_per_sec;

    /**
     * @brief True until the first call to send() completes.
     *
     * Used for checks that should be run only for the first call
     */
    bool _first_call_to_send;

    /** @brief True while the buffer monitor thread is running. */
    std::atomic<bool> _buffer_monitor_running;

    /** @brief Flag used to request buffer monitor thread shutdown. */
    std::atomic<bool> _stop_buffer_monitor;

    /** @brief Background thread that monitors buffer level and overflow state. */
    std::thread _buffer_monitor_thread;

    /** @brief SFP overflow counter value captured at streamer initialization. */
    uint16_t _sfp_oflow_start;

    /** @brief Maximum value the SFP overflow counter can report before wrapping. */
    uint16_t _max_sfp_oflow_count;

    /** @brief Extended per-channel properties for this streamer. */
    struct eprops_type {
        /** @brief Transport channel used to query FIFO level. */
        uhd::transport::zero_copy_if::sptr xport_chan;

        /** @brief Callback that reads absolute FIFO level and overflow counters. */
        xport_chan_fifo_lvl_abs_type xport_chan_fifo_lvl_abs;

        /** @brief Overflow counter value last reported for this channel. */
        uint64_t oflow;

        /** @brief Underflow counter value last reported for this channel. */
        uint64_t uflow;

        /** @brief Upper-case channel letter used in messages. */
        std::string name;

        /**
         * @brief Sample rate configured for this channel.
         *
         * This is only used for error checking.
         * The rate used to decide when to send packets is located elsewhere
         */
        double sample_rate;

        eprops_type() : oflow( -1 ), uflow( -1 ) {}
        eprops_type( const eprops_type & other )
        :
            xport_chan( other.xport_chan ),
            oflow( other.oflow ),
            uflow( other.uflow )
        {}
    };
    std::vector<eprops_type> _eprops;

    /**
     * @brief Shared flags indicating which TX channels are currently in use.
     *
     * Indexed by device channel, not streamer channel index.
     */
    std::shared_ptr<std::vector<bool>> _tx_streamer_channel_in_use;

    /**
     * @brief Advisory lock file descriptors indicating a streamer exists for a channel.
     *
     * Indexed by device channel, not streamer channel index.
     */
    std::vector<int> _channel_locks;

    /** @brief True after a performance warning has been printed for this streamer. */
    bool _performance_warning_printed = false;

    /**
     * @brief Shared pointer to the interface used to access the device server.
     */
    pv_iface::sptr _iface;

    /**
     * @brief Background loop that polls buffer level and posts async overflow messages.
     *
     * @param self Streamer instance to monitor.
     */
    static void buffer_monitor_loop(pv_device_send_packet_streamer *self);
};

}
}