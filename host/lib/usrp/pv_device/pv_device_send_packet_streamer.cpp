//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Per Vices Corporation
// Copyright 2022-2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/pv_device/pv_device_send_packet_streamer.hpp>

#include <uhd/utils/log.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread.hpp>

#include <uhdlib/utils/system_time.hpp>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sys/file.h>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

pv_device_send_packet_streamer::pv_device_send_packet_streamer(
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
)
:
sph::send_packet_streamer_mmsg( channels, max_num_samps, max_bl, dst_ips, dst_ports, device_target_nsamps, nsamp_multiple, tick_rate, async_msg_fifo, cpu_format, wire_format, wire_little_endian, clock_sync_info, streaming_locks ),
_product_name_c(product_name_c),
_min_tx_delay(min_tx_delay),
_update_per_sec(update_per_sec),
_first_call_to_send( true ),
_buffer_monitor_running( false ),
_stop_buffer_monitor( false ),
_channel_locks(channel_locks),
_iface(iface)
{
    // Attempt to lock each channel used by the streamer. If the channel is already locked, an error will be printed but the program will continue incase this is intentional behaviour by the user
    for (size_t n = 0; n < channels.size(); n++) {
        int lock_fd = _channel_locks[channels[n]];
        int r = flock(lock_fd, LOCK_EX | LOCK_NB);
        if (r == -1) {
            int err = errno;
            // Since flock was run with the nonblocking flag, errno will be EWOULDBLOCK if this channel was already locked
            if (err == EWOULDBLOCK) {
                std::string err_msg = "Another UHD streamer has already been created using channel " + std::to_string(_channels[n]) + " which may cause unexpected behaviour.";
                UHD_LOG_ERROR(_product_name_c, err_msg);
            } else {
                UHD_LOG_ERROR(_product_name_c, "Failed to place lock on channel " + std::to_string(_channels[n]) + " lockfile.\nflock failed with error: " + std::string(strerror(err)));
            }
        }
    }

    _tx_streamer_channel_in_use = tx_channel_in_use;
    for(size_t n = 0; n < channels.size(); n++) {
        _tx_streamer_channel_in_use->at(channels[n]) = true;
    }

    // Track SFP overflows if supported by server
    try {
        // Get SFP overflow counter value at initialization to track increase from this streamer
        _iface->set_int("fpga/link/qa/sfp_oflow", 0);
        _sfp_oflow_start = _iface->get_int("fpga/link/qa/sfp_oflow");
        _max_sfp_oflow_count = _iface->get_int("fpga/link/max_sfp_oflow_count");

        // If overflow counter itself has overflowed, value will be -1 and overflows will not be tracked
        if (_sfp_oflow_start == uint16_t(-1)) {
            UHD_LOG_WARNING(_product_name_c,
                "SFP overflow counter has exceeded its max count and will not be reset until the unit reboots.\n    SFP overflows will not be tracked.");
        }
    } catch (uhd::lookup_error &e) {
        // If unable to get properties from the server, set _sfp_oflow_start to -1 to disable tracking
        _sfp_oflow_start = uint16_t(-1);
    }
}

pv_device_send_packet_streamer::~pv_device_send_packet_streamer() {
    // TODO: see if having teardown seperate from the destructor is still required
    teardown();
}

void pv_device_send_packet_streamer::teardown() {
    // Stop buffer monitor thread before polling for the buffer to be empty because they use the same socket
    stop_buffer_monitor_thread();

    // Waits for all samples sent to be consumed before destructing, times out after 30s
    uhd::time_spec_t timeout_time = uhd::get_system_time() + 30;
    while(timeout_time > uhd::get_system_time()) {
        int64_t buffer_with_samples_i = -1;
        // Checks if any buffers still have samples
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            if(get_buffer_level_from_device(n) != 0) {
                buffer_with_samples_i = n;
                break;
            }
        }
        // If none have samples exit loop
        if(buffer_with_samples_i == -1) {
            break;
        // If it is taking to long for the buffer to empty, continue anyway with an error message
        } else if(timeout_time < uhd::get_system_time()) {
            UHD_LOG_ERROR(_product_name_c, "Timeout while waiting for tx " + std::to_string(_channels[buffer_with_samples_i]) + " to finish");
            break;
        }
        usleep(10);
    }

    _eprops.clear();

    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        std::string channel_name = std::string(1, ('a' + _channels[n]));
        // qa properties won't update with values until they are first manually set
        _iface->set_string(("tx/" + channel_name + "/qa/oflow"), channel_name);
        _iface->set_string(("tx/" + channel_name + "/qa/uflow"), channel_name);

        std::string oflow = _iface->get_string("tx/" + channel_name + "/qa/oflow");
        std::string uflow = _iface->get_string("tx/" + channel_name + "/qa/uflow");
        std::cout << "CH " << std::string( 1, 'A' + _channels[n] ) << ": Overflow Count: " << oflow << ", Underflow Count: " << uflow << "\n";
    }

    // Check for SFP FIFO buffer overflows if tracking was enabled for this streamer
    if (_sfp_oflow_start != uint16_t(-1)) {
        // Write to property to force update, then get updated value
        _iface->set_int("fpga/link/qa/sfp_oflow", 0);
        uint16_t sfp_total_oflow = _iface->get_int("fpga/link/qa/sfp_oflow");
        uint16_t num_sfp_oflow;
        std::string sfp_oflow_message;

        if (sfp_total_oflow == uint16_t(-1)) {
            // If counter limit was exceeded during stream, warn user of number of overflows tracked until it was exceeded
            num_sfp_oflow = _max_sfp_oflow_count - _sfp_oflow_start;
            sfp_oflow_message = "SFP overflow counter exceeded limit during streaming.\n    Counted "
                + std::to_string(num_sfp_oflow) + " overflows before tracking stopped.";
        } else {
            // The SFP buffer overflow counter does not reset until reboot, so ignore oflows from before streamer
            num_sfp_oflow = sfp_total_oflow - _sfp_oflow_start;
            sfp_oflow_message = "SFP buffer overflowed during streaming.\n    SFP Overflow Count: " + std::to_string(num_sfp_oflow);
        }

        // Only print warning when the count has increased since streamer initialization
        if (num_sfp_oflow > 0) {
            UHD_LOG_WARNING(_product_name_c, sfp_oflow_message);
        }
    }

    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        // Deactivates the channel. Mutes rf, puts the dsp in reset, and turns on the outward facing LED on the board
        // Does not actually turn off board
        _iface->set_string("tx/" + std::string(1, (char) (_channels[n] + 'a')) + "/pwr", "0");
        _tx_streamer_channel_in_use->at(_channels[n]) = false;

        // Release channel locks
        flock(_channel_locks[_channels[n]], LOCK_UN);
    }
}

//send fucntion called by external programs
size_t pv_device_send_packet_streamer::send(
    const tx_streamer::buffs_type &buffs,
    const size_t nsamps_per_buff,
    const uhd::tx_metadata_t &metadata_,
    const double timeout
){

    size_t r = 0;

    uhd::tx_metadata_t metadata = metadata_;

    if ( _first_call_to_send || metadata.start_of_burst ) {
        metadata.start_of_burst = true;

        // Make sure all channel sample rates match for this streamer. No need if there is only one channel.
        if (_eprops.size() > 1) {
            check_matching_rates();
        }

        // Lock streaming locks at the start of a burst. It will remain locked until the end of the burst (in super_send_packet_handler_mmsg)
        for (size_t n = 0; n < _NUM_CHANNELS; n++) {
            lock_channel_streaming(_channels[n]);
        }

        if ( metadata.time_spec.get_real_secs() == 0 || !metadata.has_time_spec ) {
            uhd::time_spec_t now = _clock_sync->get_device_time();
            metadata.time_spec = now + _min_tx_delay;
            metadata.has_time_spec = true;
        } else {
            double current_time = _clock_sync->get_device_time().get_real_secs();
            if (metadata.time_spec.get_real_secs() < current_time + _min_tx_delay && _first_call_to_send) {
                UHD_LOGGER_WARNING(_product_name_c) << "Requested tx start time of " + std::to_string(metadata.time_spec.get_real_secs()) + " close to current device time of " + std::to_string(current_time) + ". Shifting start time to " + std::to_string(current_time + _min_tx_delay);
                metadata.time_spec = uhd::time_spec_t(current_time + _min_tx_delay);
            }
        }
    }

    _first_call_to_send = false;

    if( ! _buffer_monitor_running.load(std::memory_order_relaxed) && !use_blocking_fc ) {
        start_buffer_monitor_thread();
    }

    r = send_packet_handler_mmsg::send(buffs, nsamps_per_buff, metadata, timeout);

    return r;
}

void pv_device_send_packet_streamer::set_xport_chan_fifo_lvl_abs( size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs ) {
    _eprops.at(chan).xport_chan_fifo_lvl_abs = get_fifo_lvl_abs;
}
void pv_device_send_packet_streamer::set_channel_name( size_t chan, std::string name ) {
    _eprops.at(chan).name = name;
}
// Stores the channel sample rate in eprops variable
void pv_device_send_packet_streamer::sync_channel_rate( size_t chan, double rate ) {
    size_t channel_index = std::distance(_channels, std::find(_channels, _channels + _NUM_CHANNELS, chan));
    _eprops.at(channel_index).sample_rate = rate;
}
void pv_device_send_packet_streamer::resize(const size_t size){
    _eprops.resize( size );
}

void pv_device_send_packet_streamer::stop_buffer_monitor_thread() {
    if ( _buffer_monitor_running.load(std::memory_order_relaxed) ) {
        _stop_buffer_monitor.store(true, std::memory_order_relaxed);
        if ( _buffer_monitor_thread.joinable() ) {
            _buffer_monitor_thread.join();
            _buffer_monitor_running.store(false, std::memory_order_relaxed);
        }
    }
}

void pv_device_send_packet_streamer::if_hdr_pack(uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) {
    vrt::if_hdr_pack_be(packet_buff, if_packet_info);
}

// TODO: refactor this so that it does not rely on binding to a function in the device
int64_t pv_device_send_packet_streamer::get_buffer_level_from_device(const size_t ch_i) {

    uint64_t level;
    uint64_t uflow;
    uint64_t oflow;
    uhd::time_spec_t then;
    _eprops[ch_i].xport_chan_fifo_lvl_abs(level, uflow, oflow, then);
    return level;
}

// Check that all channels on a streamer have the same sample rate.
// A mismatch is currently only possible on Crimson TNG (each channel's sample rate is
// set independently there), but the check is safe and cheap to run for any device.
void pv_device_send_packet_streamer::check_matching_rates() {
    const std::string mismatch_message = "Multiple sample rates are detected, but a streamer can only handle one.\n"
        "Make sure the specified sample rate is valid and identical for all channels or use multiple streamers instead.\n";
    double prev_rate = _eprops[0].sample_rate;
    for (auto &e : _eprops) {
        if (e.sample_rate != prev_rate) {
            UHD_LOG_ERROR(_product_name_c, mismatch_message);
            throw uhd::runtime_error(mismatch_message);
        }
        prev_rate = e.sample_rate;
    }
}

/***********************************************************************
    * buffer_monitor_loop
    * - DOES NOT update predicted buffer levels: predicted buffer level is based entirely on time. Having timestamps on every packet fixes dropped packets, and time diffs calculates the latency of sending data over the SFP. The benefit of having this update the buffer level is non-existent, the penalty of the inter-thread communication updating the bias is very significant when using no DDR mode
    * - update over / underflow counters
    * - put async message packets (for overflows/underflows) into queue
    **********************************************************************/
void pv_device_send_packet_streamer::buffer_monitor_loop( pv_device_send_packet_streamer *self ) {
    // This is not time sensitive, remove thread priority
    uhd::set_thread_priority_safe(0, false);

    for( ; ! self->_stop_buffer_monitor.load(std::memory_order_relaxed); ) {

        const auto t0 = std::chrono::high_resolution_clock::now();

        for( size_t i = 0; i < self->_eprops.size(); i++ ) {
            // Check if the monitoring loop has been told to exit
            if ( self->_stop_buffer_monitor.load(std::memory_order_relaxed) ) {
                return;
            }

            // Object used to store the under/overflow counts for internal use elsewhere
            // TODO: see if we still need this elsewhere and consider replacing it with and array local to this function
            eprops_type & ep = self->_eprops[ i ];

            xport_chan_fifo_lvl_abs_type get_fifo_level;

            get_fifo_level = ep.xport_chan_fifo_lvl_abs;

            if ( !( get_fifo_level) ) {
                continue;
            }

            // Reported buffer level
            size_t level;
            // Number of underflows reported by this request
            uint64_t uflow;
            // Number of underflows reported by this request
            uint64_t oflow;
            // Time of the reply to the buffer level querry
            uhd::time_spec_t then;

            async_metadata_t metadata;

            // gets buffer level, we only care about the uflow and oflow counters
            try {
                get_fifo_level( level, uflow, oflow, then );
            } catch( ... ) {
                continue;
            }

            // Update underflow counter and send message if there are more underflows now than the previous check
            if ( uflow > ep.uflow ) {
                // XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
                // async_metadata_t metadata;
                // load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
                metadata.channel = i;
                metadata.has_time_spec = true;
                metadata.time_spec = then;
                metadata.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
                // assumes that underflow counter is monotonically increasing
                self->push_async_msg( metadata );

                if(!self->_performance_warning_printed) {
                    // Check if any core is not set to performance mode, used to decide if an info message should be printed if overflows occur
                    bool using_performance_governor = true;
                    std::vector<std::string> governors = uhd::get_performance_governors();
                    for(auto& g : governors) {
                        if(g.find("performance") == std::string::npos) {
                            using_performance_governor = false;
                            break;
                        }
                    }
                    if(!using_performance_governor) {
                        UHD_LOG_WARNING(self->_product_name_c, "\nSend underflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                    }
                    self->_performance_warning_printed = true;
                }
                ep.uflow = uflow;
            }
            // ep.uflow is initialized to -1, so it needs to be set if this is the first time
            if ( (uint64_t)-1 == ep.uflow ) {
                ep.uflow = uflow;
            }

            // Update overflow counter and send message if there are more overflows now than the previous check
            if ( oflow > ep.oflow ) {
                // XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
                // async_metadata_t metadata;
                // load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
                metadata.channel = i;
                metadata.has_time_spec = true;
                metadata.time_spec = then;
                metadata.event_code = uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR;
                // assumes that overflow counter is monotonically increasing
                self->push_async_msg( metadata );

                if(!self->_performance_warning_printed) {
                    // Check if any core is not set to performance mode, used to decide if an info message should be printed if overflows occur
                    bool using_performance_governor = true;
                    std::vector<std::string> governors = uhd::get_performance_governors();
                    for(auto& g : governors) {
                        if(g.find("performance") == std::string::npos) {
                            using_performance_governor = false;
                            break;
                        }
                    }
                    if(!using_performance_governor) {
                        UHD_LOG_WARNING(self->_product_name_c, "\nSend overflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                    }
                    self->_performance_warning_printed = true;
                }
                ep.oflow = oflow;
            }
            // ep.oflow is initialized to -1, so it needs to be set if this is the first time
            if ( (uint64_t)-1 == ep.oflow ) {
                ep.oflow = oflow;
            }
        }

        const auto t1 = std::chrono::high_resolution_clock::now();
        const long long us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        const long long usloop = 1.0 / self->_update_per_sec * 1e6;
        const long long usdelay = usloop - us;

        ::usleep( usdelay < 0 ? 0 : usdelay );
    }
}
