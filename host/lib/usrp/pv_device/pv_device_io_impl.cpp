#include <stdlib.h>

#include <cstdint>
#include <iomanip>
#include <mutex>
#include <sys/file.h>

#include "pv_device_io_impl.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread.hpp>
#include <functional>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <vector>

#include <boost/endian/buffers.hpp>
#include <boost/endian/conversion.hpp>

#include <uhdlib/utils/system_time.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace ph = std::placeholders;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

pv_device_recv_packet_streamer::pv_device_recv_packet_streamer(const std::string& log_id, const std::vector<size_t>& channels, const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<std::vector<bool>> rx_channel_in_use, size_t device_total_rx_channels, pv_iface::sptr iface, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuer, std::vector<int> channel_locks, std::vector<int> streaming_locks)
: sph::recv_packet_streamer_mmsg(channels, recv_sockets, dst_ip, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian, device_total_rx_channels, cmd_issuer, streaming_locks),
_log_id(log_id),
_rx_streamer_channel_in_use(rx_channel_in_use),
_channel_locks(channel_locks),
_streaming_locks(streaming_locks),
_iface(iface)
{
    for (size_t n = 0; n < channels.size(); n++) {
        int lock_fd = _channel_locks[channels[n]];
        int r = flock(lock_fd, LOCK_EX | LOCK_NB);
        if (r == -1) {
            int err = errno;
            if (err == EWOULDBLOCK) {
                std::string err_msg = "Another UHD streamer has already been created using channel " + std::to_string(_channels[n]) + " which may cause unexpected behaviour.";
                UHD_LOG_ERROR(_log_id, err_msg);
            } else {
                UHD_LOG_ERROR(_log_id, "Failed to place lock on channel " + std::to_string(_channels[n]) + " lockfile.\nflock failed with error: " + std::string(strerror(err)));
            }
        }
    }

    for(size_t n = 0; n < channels.size(); n++) {
        _rx_streamer_channel_in_use->at(channels[n]) = true;
    }
}

pv_device_recv_packet_streamer::~pv_device_recv_packet_streamer() {
    teardown();
}

void pv_device_recv_packet_streamer::if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) {
    vrt::if_hdr_unpack_be(packet_buff, if_packet_info);
}

void pv_device_recv_packet_streamer::teardown() {
    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        _iface->set_string("rx/" + std::string(1, (char) (_channels[n] + 'a')) + "/stream", "0");
        _iface->set_string("rx/" + std::string(1, (char) (_channels[n] + 'a')) + "/pwr", "0");
        _rx_streamer_channel_in_use->at(_channels[n]) = false;
        flock(_channel_locks[_channels[n]], LOCK_UN);
    }
}

pv_device_send_packet_streamer::pv_device_send_packet_streamer(const std::string& log_id, const std::vector<size_t>& channels, const size_t max_num_samps, const size_t max_bl, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, const size_t nsamp_multiple, double tick_rate, double min_tx_delay, const size_t update_per_sec, const std::shared_ptr<uhd::pv_tx_async_msg_queue> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<std::vector<bool>> tx_channel_in_use, pv_iface::sptr iface, std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info, std::vector<int> channel_locks, std::vector<int> streaming_locks)
: sph::send_packet_streamer_mmsg(channels, max_num_samps, max_bl, dst_ips, dst_ports, device_target_nsamps, nsamp_multiple, tick_rate, async_msg_fifo, cpu_format, wire_format, wire_little_endian, clock_sync_info, streaming_locks),
_min_tx_delay(min_tx_delay),
_update_per_sec(update_per_sec),
_first_call_to_send(true),
_buffer_monitor_running(false),
_stop_buffer_monitor(false),
_tx_streamer_channel_in_use(tx_channel_in_use),
_channel_locks(channel_locks),
_log_id(log_id),
_iface(iface)
{
    for (size_t n = 0; n < channels.size(); n++) {
        int lock_fd = _channel_locks[channels[n]];
        int r = flock(lock_fd, LOCK_EX | LOCK_NB);
        if (r == -1) {
            int err = errno;
            if (err == EWOULDBLOCK) {
                std::string err_msg = "Another UHD streamer has already been created using channel " + std::to_string(_channels[n]) + " which may cause unexpected behaviour.";
                UHD_LOG_ERROR(_log_id, err_msg);
            } else {
                UHD_LOG_ERROR(_log_id, "Failed to place lock on channel " + std::to_string(_channels[n]) + " lockfile.\nflock failed with error: " + std::string(strerror(err)));
            }
        }
    }

    for(size_t n = 0; n < channels.size(); n++) {
        _tx_streamer_channel_in_use->at(channels[n]) = true;
    }

    try {
        _iface->set_int("fpga/link/qa/sfp_oflow", 0);
        _sfp_oflow_start = _iface->get_int("fpga/link/qa/sfp_oflow");
        _max_sfp_oflow_count = _iface->get_int("fpga/link/max_sfp_oflow_count");
        if (_sfp_oflow_start == uint16_t(-1)) {
            UHD_LOG_WARNING(_log_id, "SFP overflow counter has exceeded its max count and will not be reset until the unit reboots.\n    SFP overflows will not be tracked.");
        }
    } catch (uhd::lookup_error&) {
        _sfp_oflow_start = uint16_t(-1);
    }
}

pv_device_send_packet_streamer::~pv_device_send_packet_streamer() {
    teardown();
}

void pv_device_send_packet_streamer::teardown() {
    stop_buffer_monitor_thread();

    uhd::time_spec_t timeout_time = uhd::get_system_time() + 30;
    while(timeout_time > uhd::get_system_time()) {
        int64_t buffer_with_samples_i = -1;
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            if(get_buffer_level_from_device(n) != 0) {
                buffer_with_samples_i = n;
                break;
            }
        }
        if(buffer_with_samples_i == -1) {
            break;
        } else if(timeout_time < uhd::get_system_time()) {
            UHD_LOG_ERROR(_log_id, "Timeout while waiting for tx " + std::to_string(_channels[buffer_with_samples_i]) + " to finish");
            break;
        }
        usleep(10);
    }

    _eprops.clear();

    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        std::string channel_name = std::string(1, ('a' + _channels[n]));
        _iface->set_string(("tx/" + channel_name + "/qa/oflow"), channel_name);
        _iface->set_string(("tx/" + channel_name + "/qa/uflow"), channel_name);

        std::string oflow = _iface->get_string("tx/" + channel_name + "/qa/oflow");
        std::string uflow = _iface->get_string("tx/" + channel_name + "/qa/uflow");
        std::cout << "CH " << std::string( 1, 'A' + _channels[n] ) << ": Overflow Count: " << oflow << ", Underflow Count: " << uflow << "\n";
    }

    if (_sfp_oflow_start != uint16_t(-1)) {
        _iface->set_int("fpga/link/qa/sfp_oflow", 0);
        uint16_t sfp_total_oflow = _iface->get_int("fpga/link/qa/sfp_oflow");
        uint16_t num_sfp_oflow;
        std::string sfp_oflow_message;

        if (sfp_total_oflow == uint16_t(-1)) {
            num_sfp_oflow = _max_sfp_oflow_count - _sfp_oflow_start;
            sfp_oflow_message = "SFP overflow counter exceeded limit during streaming.\n    Counted " + std::to_string(num_sfp_oflow) + " overflows before tracking stopped.";
        } else {
            num_sfp_oflow = sfp_total_oflow - _sfp_oflow_start;
            sfp_oflow_message = "SFP buffer overflowed during streaming.\n    SFP Overflow Count: " + std::to_string(num_sfp_oflow);
        }

        if (num_sfp_oflow > 0) {
            UHD_LOG_WARNING(_log_id, sfp_oflow_message);
        }
    }

    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        _iface->set_string("tx/" + std::string(1, (char) (_channels[n] + 'a')) + "/pwr", "0");
        _tx_streamer_channel_in_use->at(_channels[n]) = false;
        flock(_channel_locks[_channels[n]], LOCK_UN);
    }
}

size_t pv_device_send_packet_streamer::send(const tx_streamer::buffs_type &buffs, const size_t nsamps_per_buff, const uhd::tx_metadata_t &metadata_, const double timeout){
    size_t r = 0;
    uhd::tx_metadata_t metadata = metadata_;

    if ( _first_call_to_send || metadata.start_of_burst ) {
        metadata.start_of_burst = true;

        if (_eprops.size() > 1) {
            check_matching_rates();
        }

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
                UHD_LOGGER_WARNING(_log_id) << "Requested tx start time of " + std::to_string(metadata.time_spec.get_real_secs()) + " close to current device time of " + std::to_string(current_time) + ". Shifting start time to " + std::to_string(current_time + _min_tx_delay);
                metadata.time_spec = uhd::time_spec_t(current_time + _min_tx_delay);
            }
        }
    }

    _first_call_to_send = false;

    if( ! _buffer_monitor_running.load(std::memory_order_relaxed) && !use_blocking_fc ) {
        _stop_buffer_monitor.store(false, std::memory_order_relaxed);
        _buffer_monitor_thread = std::thread( pv_device_send_packet_streamer::buffer_monitor_loop, this );
        _buffer_monitor_running.store(true, std::memory_order_relaxed);
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
void pv_device_send_packet_streamer::sync_channel_rate( size_t chan, double rate ) {
    size_t channel_index = std::distance(_channels.data(), std::find(_channels.data(), _channels.data() + _NUM_CHANNELS, chan));
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

int64_t pv_device_send_packet_streamer::get_buffer_level_from_device(const size_t ch_i) {
    uint64_t level;
    uint64_t uflow;
    uint64_t oflow;
    uhd::time_spec_t then;
    _eprops[ch_i].xport_chan_fifo_lvl_abs(level, uflow, oflow, then);
    return level;
}

void pv_device_send_packet_streamer::check_matching_rates() {
    const std::string mismatch_message = "Multiple sample rates are detected, but a streamer can only handle one.\nMake sure the specified sample rate is valid and identical for all channels or use multiple streamers instead.\n";
    double prev_rate = _eprops[0].sample_rate;
    for (auto &e : _eprops) {
        if (e.sample_rate != prev_rate) {
            UHD_LOG_ERROR(_log_id, mismatch_message);
            throw uhd::runtime_error(mismatch_message);
        }
        prev_rate = e.sample_rate;
    }
}

void pv_device_send_packet_streamer::buffer_monitor_loop( pv_device_send_packet_streamer *self ) {
    uhd::set_thread_priority_safe(0, false);

    for( ; ! self->_stop_buffer_monitor.load(std::memory_order_relaxed); ) {
        const auto t0 = std::chrono::high_resolution_clock::now();

        for( size_t i = 0; i < self->_eprops.size(); i++ ) {
            if ( self->_stop_buffer_monitor.load(std::memory_order_relaxed) ) {
                return;
            }

            eprops_type & ep = self->_eprops[ i ];
            xport_chan_fifo_lvl_abs_type get_fifo_level = ep.xport_chan_fifo_lvl_abs;
            if ( !( get_fifo_level) ) {
                continue;
            }

            size_t level;
            uint64_t uflow;
            uint64_t oflow;
            uhd::time_spec_t then;
            async_metadata_t metadata;

            try {
                get_fifo_level( level, uflow, oflow, then );
            } catch( ... ) {
                continue;
            }

            if ( uflow > ep.uflow ) {
                metadata.channel = i;
                metadata.has_time_spec = true;
                metadata.time_spec = then;
                metadata.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
                self->push_async_msg( metadata );

                if(!self->_performance_warning_printed) {
                    bool using_performance_governor = true;
                    std::vector<std::string> governors = uhd::get_performance_governors();
                    for(auto& g : governors) {
                        if(g.find("performance") == std::string::npos) {
                            using_performance_governor = false;
                            break;
                        }
                    }
                    if(!using_performance_governor) {
                        UHD_LOG_WARNING(self->_log_id, "\nSend underflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                    }
                    self->_performance_warning_printed = true;
                }
                ep.uflow = uflow;
            }
            if ( (uint64_t)-1 == ep.uflow ) {
                ep.uflow = uflow;
            }

            if ( oflow > ep.oflow ) {
                metadata.channel = i;
                metadata.has_time_spec = true;
                metadata.time_spec = then;
                metadata.event_code = uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR;
                self->push_async_msg( metadata );

                if(!self->_performance_warning_printed) {
                    bool using_performance_governor = true;
                    std::vector<std::string> governors = uhd::get_performance_governors();
                    for(auto& g : governors) {
                        if(g.find("performance") == std::string::npos) {
                            using_performance_governor = false;
                            break;
                        }
                    }
                    if(!using_performance_governor) {
                        UHD_LOG_WARNING(self->_log_id, "\nSend overflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                    }
                    self->_performance_warning_printed = true;
                }
                ep.oflow = oflow;
            }
            if ( (uint64_t)-1 == ep.oflow ) {
                ep.oflow = oflow;
            }
        }

        const auto t1 = std::chrono::high_resolution_clock::now();
        const long long us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        const long long usloop = 1.0 / (double)self->_update_per_sec * 1e6;
        const long long usdelay = usloop - us;
        ::usleep( usdelay < 0 ? 0 : usdelay );
    }
}
