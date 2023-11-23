//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Per Vices Corporation
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

#include <stdlib.h>

#include <iomanip>
#include <mutex>

#include "../../transport/super_recv_packet_handler_mmsg.hpp"
#include "../../transport/super_send_packet_handler_mmsg.hpp"
#include "crimson_tng_impl.hpp"
#include "crimson_tng_fw_common.h"
#include <uhd/utils/log.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/format.hpp>
#include <functional>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <thread>
#include <vector>

#include <boost/endian/buffers.hpp>
#include <boost/endian/conversion.hpp>

#include <uhdlib/utils/system_time.hpp>

#if 0
  #ifndef UHD_TXRX_DEBUG_PRINTS
  #define UHD_TXRX_DEBUG_PRINTS
  #endif
#endif

#if 0
  #ifndef UHD_TXRX_SEND_DEBUG_PRINTS
  #define UHD_TXRX_SEND_DEBUG_PRINTS
  #endif
#endif

#if 0
  #ifndef DEBUG_FC
  #define DEBUG_FC
  #endif
#endif

#if 0
  #ifndef BUFFER_LVL_DEBUG
  #define BUFFER_LVL_DEBUG
  #endif
#endif

#if 0
  #ifndef UHD_TXRX_DEBUG_TIME
  #define UHD_TXRX_DEBUG_TIME
  #endif
#endif

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace ph = std::placeholders;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

/***********************************************************************
 * helpers
 **********************************************************************/

std::ostream & operator<<( std::ostream & os, const uhd::time_spec_t & ts ) {
	os << std::fixed << std::setprecision( 6 ) << ts.get_real_secs();
	return os;
}

class crimson_tng_recv_packet_streamer : public sph::recv_packet_streamer_mmsg {
public:
	typedef std::function<void(void)> onfini_type;

	crimson_tng_recv_packet_streamer(const std::vector<size_t> channels, const std::vector<std::string>& dsp_ip, std::vector<int>& dst_port, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian,  std::shared_ptr<std::vector<bool>> rx_channel_in_use)
	: sph::recv_packet_streamer_mmsg(dsp_ip, dst_port, CRIMSON_TNG_MAX_NBYTES, CRIMSON_TNG_HEADER_SIZE, CRIMSON_TNG_TRAILER_SIZE, cpu_format, wire_format, wire_little_endian),
	_channels(channels)
	{
        _rx_streamer_channel_in_use = rx_channel_in_use;
        for(size_t n = 0; n < channels.size(); n++) {
            _rx_streamer_channel_in_use->at(channels[n]) = true;
        }
    }

	virtual ~crimson_tng_recv_packet_streamer() {
		teardown();
	}

    void issue_stream_cmd(const stream_cmd_t &stream_cmd)
    {
        return recv_packet_handler::issue_stream_cmd(stream_cmd);
    }

    void set_on_fini( size_t chan, onfini_type on_fini ) {
        _eprops.at(chan).on_fini = on_fini;
    }

    void resize(const size_t size) {
        _eprops.resize( size );
        sph::recv_packet_streamer_mmsg::resize( size );
    }

    void if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) {
        vrt::if_hdr_unpack_be(packet_buff, if_packet_info);
    }

	void teardown() {
		for( auto & ep: _eprops ) {
			if ( ep.on_fini ) {
				ep.on_fini();
			}
		}
		_eprops.clear();

        for(size_t n = 0; n < _channels.size(); n++) {
            _rx_streamer_channel_in_use->at(_channels[n]) = false;
        }
	}

private:
    size_t _max_num_samps;

    struct eprops_type{
        onfini_type on_fini;
    };
    std::vector<eprops_type> _eprops;

    std::vector<size_t> _channels;
    std::shared_ptr<std::vector<bool>> _rx_streamer_channel_in_use;
};

static std::vector<std::weak_ptr<crimson_tng_recv_packet_streamer>> allocated_rx_streamers;
static void shutdown_lingering_rx_streamers() {
	// This is required as a workaround, because the relevent destructurs are not called
	// when you close the top block in gnu radio. Unsolved mystery for the time being.
	for( auto & rx: allocated_rx_streamers ) {
		if ( ! rx.expired() ) {
			std::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer = rx.lock();
			if ( my_streamer ) {
				my_streamer->teardown();
			}
		}
	}
	allocated_rx_streamers.clear();
}

class crimson_tng_send_packet_streamer : public sph::send_packet_streamer_mmsg {
public:

	typedef std::function<void(void)> onfini_type;
	typedef std::function<uhd::time_spec_t(void)> timenow_type;
    typedef std::function<void(uint64_t&,uint64_t&,uint64_t&,uhd::time_spec_t&)> xport_chan_fifo_lvl_abs_type;

	crimson_tng_send_packet_streamer(const std::vector<size_t>& channels, const size_t max_num_samps, const size_t max_bl, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, const std::shared_ptr<bounded_buffer<async_metadata_t>> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<std::vector<bool>> tx_channel_in_use)
	:
		sph::send_packet_streamer_mmsg( channels, max_num_samps, max_bl, dst_ips, dst_ports, device_target_nsamps, CRIMSON_TNG_PACKET_NSAMP_MULTIPLE, CRIMSON_TNG_MASTER_TICK_RATE, async_msg_fifo, cpu_format, wire_format, wire_little_endian ),
		_first_call_to_send( true ),
		_buffer_monitor_running( false ),
		_stop_buffer_monitor( false )

	{
        _tx_streamer_channel_in_use = tx_channel_in_use;
        for(size_t n = 0; n < channels.size(); n++) {
            _tx_streamer_channel_in_use->at(channels[n]) = true;
        }
	}

	virtual ~crimson_tng_send_packet_streamer() {
		teardown();
	}

	void teardown() {
		stop_buffer_monitor_thread();
		for( auto & ep: _eprops ) {
			if ( ep.on_fini ) {
				ep.on_fini();
			}
            // oflow/uflow counter is initialized to -1. If they are still -1 then the monitoring hasn't started yet
            // TODO: query the uflow/oflow count from the FPGA once it supports that
            if(ep.oflow != (uint64_t)-1 || ep.uflow != (uint64_t)-1) {
                std::cout << "CH " << ep.name << ": Overflow Count: " << ep.oflow << ", Underflow Count: " << ep.uflow << "\n";
            } else {
                std::cout << "CH " << ep.name << ": Overflow Count: 0, Underflow Count: 0\n";
            }
		}
		_eprops.clear();

        for(size_t n = 0; n < _channels.size(); n++) {
            _tx_streamer_channel_in_use->at(_channels[n]) = false;
        }
	}

    // For temporary workaround to reject underflows/overflows after an end of burst
    uhd::time_spec_t sob_time;
    bool streaming_active = false;
    
    //send fucntion called by external programs
    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    ){

        static const double default_sob = 1.0;

        size_t r = 0;

        uhd::tx_metadata_t metadata = metadata_;

        if ( _first_call_to_send || metadata.start_of_burst ) {
            metadata.start_of_burst = true;


            if ( metadata.time_spec.get_real_secs() == 0 || !metadata.has_time_spec ) {
                uhd::time_spec_t now = get_time_now();
                metadata.time_spec = now + default_sob;
                metadata.has_time_spec = true;
            } else {
                double current_time = get_time_now().get_real_secs();
                if (metadata.time_spec.get_real_secs() < current_time + CRIMSON_TNG_MIN_TX_DELAY) {
                    UHD_LOGGER_WARNING(CRIMSON_TNG_DEBUG_NAME_C) << "Requested tx start time of " + std::to_string(metadata.time_spec.get_real_secs()) + " close to current device time of " + std::to_string(current_time) + ". Shifting start time to " + std::to_string(current_time + CRIMSON_TNG_MIN_TX_DELAY);
                    metadata.time_spec = uhd::time_spec_t(current_time + CRIMSON_TNG_MIN_TX_DELAY);
                }
            }

            sob_time = metadata.time_spec;
        }

        streaming_active = (streaming_active || metadata.start_of_burst) && !metadata.end_of_burst;

        _first_call_to_send = false;

        if( ! _buffer_monitor_running && !use_blocking_fc ) {
            start_buffer_monitor_thread();
        }

        r = send_packet_handler_mmsg::send(buffs, nsamps_per_buff, metadata, timeout);

        if ( metadata.end_of_burst ) {
            stop_buffer_monitor_thread();
        }

        return r;
    }
    
    // Sets function to be run on close
    void set_on_fini( size_t chan, onfini_type on_fini ) {
		_eprops.at(chan).on_fini = on_fini;
    }

    // Sets the function from the device to be used to get the expected time on the device
    void set_time_now_function( timenow_type time_now ) {
        _time_now = time_now;
    }
    // Calls the function from the device to get the time on the device if it has been set, otherwise get's the host's system time
    uhd::time_spec_t get_time_now() {
        return _time_now ? _time_now() : get_system_time();
    }
    void set_xport_chan_fifo_lvl_abs( size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs ) {
		_eprops.at(chan).xport_chan_fifo_lvl_abs = get_fifo_lvl_abs;
    }
    void set_channel_name( size_t chan, std::string name ) {
        _eprops.at(chan).name = name;
    }

    void resize(const size_t size){
		_eprops.resize( size );
    }

    // Starts buffer monitor thread if it is not already running
	inline void start_buffer_monitor_thread() {
        _stop_buffer_monitor = false;

        //spawn a thread to monitor the buffer level
        _buffer_monitor_thread = std::thread( crimson_tng_send_packet_streamer::buffer_monitor_loop, this );
        _buffer_monitor_running = true;
	}

	void stop_buffer_monitor_thread() {
		if ( _buffer_monitor_running ) {
			_stop_buffer_monitor = true;
			if ( _buffer_monitor_thread.joinable() ) {
				_buffer_monitor_thread.join();
				_buffer_monitor_running = false;
			}
		}
	}

protected:
    void if_hdr_pack(uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) {
        vrt::if_hdr_pack_be(packet_buff, if_packet_info);
    }

    // TODO: refactor this so that it does not rely on binding to a function in the device
    int64_t get_buffer_level_from_device(const size_t ch_i) {

        uint64_t level;
        uint64_t uflow;
        uint64_t oflow;
        uhd::time_spec_t then;
        _eprops[ch_i].xport_chan_fifo_lvl_abs(level, uflow, oflow, then);
        return level;
    }

private:
	bool _first_call_to_send;
    bool _buffer_monitor_running;
    std::atomic<bool> _stop_buffer_monitor;
    std::thread _buffer_monitor_thread;
    timenow_type _time_now;

    // extended per-channel properties, beyond what is available in sphc::send_packet_handler::xport_chan_props_type
    struct eprops_type{
		onfini_type on_fini;
		uhd::transport::zero_copy_if::sptr xport_chan;
        xport_chan_fifo_lvl_abs_type xport_chan_fifo_lvl_abs;
		uint64_t oflow;
		uint64_t uflow;
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

    /***********************************************************************
     * buffer_monitor_loop
     * - update buffer levels
     * - update over / underflow counters
     * - put async message packets into queue
     **********************************************************************/
	static void buffer_monitor_loop( crimson_tng_send_packet_streamer *self ) {
        // This is not time sensitive, remove thread priority
        uhd::set_thread_priority_safe(0, false);

		for( ; ! self->_stop_buffer_monitor; ) {

			const auto t0 = std::chrono::high_resolution_clock::now();

			for( size_t i = 0; i < self->_eprops.size(); i++ ) {
                // Skip checking for overflows/underflows after an end of burst, and before the start time of the next burst
                if(!self->streaming_active || self->get_time_now() < self->sob_time) {
                    ::usleep( (int)(1.0 / (double)CRIMSON_TNG_UPDATE_PER_SEC * 1e6) );
                    continue;
                }

				eprops_type & ep = self->_eprops[ i ];

				xport_chan_fifo_lvl_abs_type get_fifo_level;

				get_fifo_level = ep.xport_chan_fifo_lvl_abs;

				if ( !( get_fifo_level) ) {
					continue;
				}

				uhd::time_spec_t then;
				uint64_t uflow;
				uint64_t oflow;
				async_metadata_t metadata;

                if ( self->_stop_buffer_monitor ) {
					return;
				}

				size_t level;
                //gets buffer level
				try {
					get_fifo_level( level, uflow, oflow, then );
				} catch( ... ) {
                    continue;
                }

                self->update_buffer_level(i, level, then);

				if ( (uint64_t)-1 != ep.uflow && uflow != ep.uflow ) {
					// XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
					// async_metadata_t metadata;
					// load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
					metadata.channel = i;
					metadata.has_time_spec = true;
					metadata.time_spec = then;
					metadata.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
					// assumes that underflow counter is monotonically increasing
					self->push_async_msg( metadata );
				}

				ep.uflow = uflow;

				if ( (uint64_t)-1 != ep.oflow && oflow != ep.oflow ) {
					// XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
					// async_metadata_t metadata;
					// load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
					metadata.channel = i;
					metadata.has_time_spec = true;
					metadata.time_spec = then;
					metadata.event_code = uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR;
					// assumes that overflow counter is monotonically increasing
					self->push_async_msg( metadata );
				}

				ep.oflow = oflow;
			}

			const auto t1 = std::chrono::high_resolution_clock::now();
			const long long us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
			const long long usloop = 1.0 / (double)CRIMSON_TNG_UPDATE_PER_SEC * 1e6;
			const long long usdelay = usloop - us;

			::usleep( usdelay < 0 ? 0 : usdelay );
		}
	}
};

static std::vector<std::weak_ptr<crimson_tng_send_packet_streamer>> allocated_tx_streamers;
static void shutdown_lingering_tx_streamers() {
	// This is required as a workaround, because the relevent destructurs are not called
	// when you close the top block in gnu radio. Unsolved mystery for the time being.
	for( auto & tx: allocated_tx_streamers ) {
		if ( ! tx.expired() ) {
			std::shared_ptr<crimson_tng_send_packet_streamer> my_streamer = tx.lock();
			if ( my_streamer ) {
				my_streamer->teardown();
			}
		}
	}
	allocated_tx_streamers.clear();
}

/***********************************************************************
 * constants
 **********************************************************************/
static const size_t vrt_send_header_offset_words32 = 0;

/***********************************************************************
 * Helper Functions
 **********************************************************************/
void crimson_tng_impl::io_init(void){

    //allocate streamer weak ptrs containers
    for (const std::string &mb : _mbc.keys()) {
        _mbc[mb].rx_streamers.resize( CRIMSON_TNG_RX_CHANNELS );
        _mbc[mb].tx_streamers.resize( CRIMSON_TNG_TX_CHANNELS );
    }
}

void crimson_tng_impl::rx_rate_check(size_t ch, double rate_samples) {
    rx_sfp_throughput_used[ch] = rate_samples;
    // Only print the warning once
    if(rx_rate_warning_printed) {
        return;
    }
    double rate_used = 0;
    for(size_t n = 0; n < num_rx_channels; n++) {
        if(get_rx_sfp(n) == get_rx_sfp(ch) && rx_channel_in_use->at(n)) {
            rate_used += rx_sfp_throughput_used[ch];
        }
    }

    if(rate_used * CRIMSON_TNG_RX_SAMPLE_BITS * 2 > get_link_rate()) {

        UHD_LOGGER_WARNING(CRIMSON_TNG_DEBUG_NAME_C)
                << boost::format("The total sum of rates (%f MSps on SFP used by channel %u)"
                                 "exceeds the maximum capacity of the connection.\n"
                                 "This can cause overflows.")
                       % (rate_used / 1e6) % ch;

        rx_rate_warning_printed = true;
    }
}

// Lets the streamer know what the rate is
void crimson_tng_impl::update_rx_samp_rate(const std::string &mb, const size_t dsp, const double rate_){

    set_double( "rx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate", rate_ );
    double rate = get_double( "rx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate" );

    rx_rate_check(dsp, rate);

    std::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer =
        std::dynamic_pointer_cast<crimson_tng_recv_packet_streamer>(_mbc[mb].rx_streamers[dsp].lock());
    if (my_streamer.get() == NULL) return;

    my_streamer->set_sample_rate(rate);
}

void crimson_tng_impl::tx_rate_check(size_t ch, double rate_samples) {
    tx_sfp_throughput_used[ch] = rate_samples;
    double rate_used = 0;
    for(size_t n = 0; n < num_tx_channels; n++) {
        if(get_tx_sfp(n) == get_tx_sfp(ch) && tx_channel_in_use->at(n)) {
            rate_used += tx_sfp_throughput_used[ch];
        }
    }

    if(rate_used * CRIMSON_TNG_TX_SAMPLE_BITS * 2 > get_link_rate() && !tx_rate_warning_printed) {

        UHD_LOGGER_WARNING(CRIMSON_TNG_DEBUG_NAME_C)
                << boost::format("The total sum of rates (%f MSps on SFP used by channel %u)"
                                 "exceeds the maximum capacity of the connection.\n"
                                 "This can cause underruns.")
                       % (rate_used / 1e6) % ch;

        // Only print this warning once
        tx_rate_warning_printed = true;
    }
}

// Lets the streamer know what the rate is
void crimson_tng_impl::update_tx_samp_rate(const std::string &mb, const size_t dsp, const double rate_ ){

    set_double( "tx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate", rate_ );
    double rate = get_double( "tx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate" );

    tx_rate_check(dsp, rate);

	std::shared_ptr<crimson_tng_send_packet_streamer> my_streamer =
        std::dynamic_pointer_cast<crimson_tng_send_packet_streamer>(_mbc[mb].tx_streamers[dsp].lock());
    if (my_streamer.get() == NULL) return;

    my_streamer->set_samp_rate(rate);
}

void crimson_tng_impl::update_all_rx_rates(void){
    for (const std::string &mb : _mbc.keys()) {
        fs_path root = "/mboards/" + mb;

        for(const std::string &name : _tree->list(root / "rx_dsps")) {
            if ( "1" == _tree->access<std::string>( root / "rx" / name / "pwr").get() ) {
                _tree->access<double>(root / "rx_dsps" / name / "rate" / "value").update();
            }
        }
    }
}

void crimson_tng_impl::update_all_tx_rates(void){
    for (const std::string &mb : _mbc.keys()) {
        fs_path root = "/mboards/" + mb;

        for(const std::string &name : _tree->list(root / "tx_dsps")) {
            if ( "1" == _tree->access<std::string>( root / "tx" / name / "pwr").get() ) {
                _tree->access<double>(root / "tx_dsps" / name / "rate" / "value").update();
            }
        }
    }
}

void crimson_tng_impl::update_rx_subdev_spec(const std::string &which_mb, const subdev_spec_t &spec){
    fs_path root = "/mboards/" + which_mb + "/dboards";

    //sanity checking
    //validate_subdev_spec(_tree, spec, "rx", which_mb);

    //setup mux for this spec
    //bool fe_swapped = false;
    //for (size_t i = 0; i < spec.size(); i++){
    //    const std::string conn = _tree->access<std::string>(root / spec[i].db_name / "rx_frontends" / spec[i].sd_name / "connection").get();
    //    if (i == 0 and (conn == "QI" or conn == "Q")) fe_swapped = true;
    //    _mbc[which_mb].rx_dsps[i]->set_mux(conn, fe_swapped);
    //}
    //_mbc[which_mb].rx_fe->set_mux(fe_swapped);

    //compute the new occupancy and resize
    _mbc[which_mb].rx_chan_occ = spec.size();
    size_t nchan = 0;
    for(const std::string &mb:  _mbc.keys()) nchan += _mbc[mb].rx_chan_occ;
}

void crimson_tng_impl::update_tx_subdev_spec(const std::string &which_mb, const subdev_spec_t &spec){
    fs_path root = "/mboards/" + which_mb + "/dboards";

    //sanity checking
    //validate_subdev_spec(_tree, spec, "tx", which_mb);

    //set the mux for this spec
    //const std::string conn = _tree->access<std::string>(root / spec[0].db_name / "tx_frontends" / spec[0].sd_name / "connection").get();
    //_mbc[which_mb].tx_fe->set_mux(conn);

    //compute the new occupancy and resize
    _mbc[which_mb].tx_chan_occ = spec.size();
    size_t nchan = 0;
    for(const std::string &mb:  _mbc.keys()) nchan += _mbc[mb].tx_chan_occ;
}

static void rx_pwr_off( std::weak_ptr<uhd::property_tree> tree, std::string path ) {
	tree.lock()->access<std::string>( path + "/stream" ).set( "0" );
	tree.lock()->access<std::string>( path + "/pwr" ).set( "0" );
}

static void tx_pwr_off( std::weak_ptr<uhd::property_tree> tree, std::string path ) {
	tree.lock()->access<std::string>( path + "/pwr" ).set( "0" );
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool crimson_tng_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    if(!recv_async_msg_deprecated_warning) {
        std::cout << "device recv_async_msg function is deprecated. Stream to tx_streamer.recv_async_msg\n";
        recv_async_msg_deprecated_warning = true;
    }
    // The fifo is created during get_tx_stream, as part of changes to better handle stream specific get async messages
    // The means calling the device get async msg (this function) before creating a stream can be done before the fifo is created
    if(_async_msg_fifo.get() != NULL) {
        boost::this_thread::disable_interruption di; //disable because the wait can throw
        return _async_msg_fifo->pop_with_timed_wait(async_metadata, timeout);
    } else {
        return false;
    }
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr crimson_tng_impl::get_rx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    if (args.otw_format != "sc16"){
        throw uhd::value_error("Crimson TNG RX cannot handle requested wire format: " + args.otw_format);
    }

    std::vector<std::string> dst_ip(args.channels.size());
    for(size_t n = 0; n < dst_ip.size(); n++) {
        dst_ip[n] = _tree->access<std::string>( rx_link_root(args.channels[n]) + "/ip_dest" ).get();
    }

    std::vector<int> dst_port(args.channels.size());
    for(size_t n = 0; n < dst_port.size(); n++) {
        dst_port[n] = std::stoi(_tree->access<std::string>( rx_link_root(args.channels[n]) + "/port" ).get());
    }

    bool little_endian_supported = true;

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){

                const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
                std::string num     = boost::lexical_cast<std::string>((char)(chan + 'A'));
                const fs_path rx_path   = CRIMSON_TNG_MB_PATH / "rx";
                const fs_path rx_fe_path    = CRIMSON_TNG_MB_PATH / "dboards" / num / "rx_frontends" / ch;
                const fs_path rx_link_path  = CRIMSON_TNG_MB_PATH / "rx_link" / chan;
                const fs_path rx_dsp_path   = CRIMSON_TNG_MB_PATH / "rx_dsps" / chan;

                // stop streaming
                _tree->access<std::string>(rx_path / chan / "stream").set("0");
                if(little_endian_supported) {
                    // enables endian swap (by default the packets are big endian, x86 CPUs are little endian)
                    _tree->access<int>(rx_link_path / "endian_swap").set(1);
                    // Checks if the server accepted the endian swap request
                    // If 0 then the device does not support endian swap
                    int endian_status = _tree->access<int>(rx_link_path / "endian_swap").get();
                    if(endian_status == 0) {
                        little_endian_supported = false;
                    }
                } else {
                    // Don't need to attempt to enable little endian for other channels if one has already failed, since they will all fail
                }
                // vita enable
                _tree->access<std::string>(rx_link_path / "vita_en").set("1");
            }
        }
    }

    // Creates streamer
    // must be done after setting stream to 0 in the state tree so flush works correctly
    std::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer = std::make_shared<crimson_tng_recv_packet_streamer>(args.channels, dst_ip, dst_port, args.cpu_format, args.otw_format, little_endian_supported, rx_channel_in_use);

    //init some streamer stuff
    my_streamer->resize(args.channels.size());

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){
                std::string scmd_pre( "rx_" + std::string( 1, 'a' + chan ) + "/stream" );
                my_streamer->set_issue_stream_cmd(chan_i, std::bind(
                    &crimson_tng_impl::set_stream_cmd, this, scmd_pre, ph::_1));
                my_streamer->set_on_fini(chan_i, std::bind( & rx_pwr_off, _tree, std::string( "/mboards/" + mb + "/rx/" + std::to_string( chan ) ) ) );
                _mbc[mb].rx_streamers[chan] = my_streamer; //store weak pointer
                break;
            }
        }
    }

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){

                const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
                std::string num     = boost::lexical_cast<std::string>((char)(chan + 'A'));
                const fs_path rx_path   = CRIMSON_TNG_MB_PATH / "rx";
                const fs_path rx_fe_path    = CRIMSON_TNG_MB_PATH / "dboards" / num / "rx_frontends" / ch;
                const fs_path rx_link_path  = CRIMSON_TNG_MB_PATH / "rx_link" / chan;
                const fs_path rx_dsp_path   = CRIMSON_TNG_MB_PATH / "rx_dsps" / chan;

                _tree->access<std::string>(rx_path / chan / "stream").set("0");
                // vita enable
                _tree->access<std::string>(rx_link_path / "vita_en").set("1");

                // power on the channel
                _tree->access<std::string>(rx_path / chan / "pwr").set("1");
                _tree->access<std::string>(rx_path / chan / "stream").set("1");

// FIXME: @CF: 20180316: our TREE macros do not populate update(), unfortunately
#define _update( t, p ) \
    _tree->access<t>( p ).set( _tree->access<t>( p ).get() )

                _update( int, rx_fe_path / "freq" / "band" );
            }
        }
    }

    //sets all tick and samp rates on this streamer
    this->update_all_rx_rates();

    allocated_rx_streamers.push_back( my_streamer );
    ::atexit( shutdown_lingering_rx_streamers );

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/

static void get_fifo_lvl_udp_abs( const size_t channel, const int64_t bl_multiple, uhd::transport::udp_simple::sptr xport, uint64_t & lvl, uint64_t & uflow, uint64_t & oflow, uhd::time_spec_t & now ) {

	static constexpr double tick_period_ps = 1.0 / CRIMSON_TNG_MASTER_TICK_RATE;

	#pragma pack(push,1)
	struct fifo_lvl_req {
		uint64_t header; // 000000010001CCCC (C := channel bits, x := WZ,RAZ)
		//uint64_t cookie;
	};
	#pragma pack(pop)

	#pragma pack(push,1)
	struct fifo_lvl_rsp {
		uint64_t header; // CCCC00000000FFFF (C := channel bits, F := fifo bits)
		uint64_t oflow;
		uint64_t uflow;
		uint64_t tv_sec;
		uint64_t tv_tick;
		//uint64_t cookie;
	};
	#pragma pack(pop)

	fifo_lvl_req req;
	fifo_lvl_rsp rsp;

	req.header = (uint64_t)0x10001 << 16;
	req.header |= (channel & 0xffff);

	boost::endian::big_to_native_inplace( req.header );

	size_t r = 0;

	for( size_t tries = 0; tries < 100; tries++ ) {
		r = xport->send( boost::asio::mutable_buffer( & req, sizeof( req ) ) );
		if ( sizeof( req ) != r ) {
			continue;
		}

		r = xport->recv( boost::asio::mutable_buffer( & rsp, sizeof( rsp ) ) );
		if ( sizeof( rsp ) != r ) {
			continue;
		}

		boost::endian::big_to_native_inplace( rsp.header );
		if ( channel != ( ( rsp.header >> 48 ) & 0xffff ) ) {
			r = 0;
			continue;
		}

		break;
	}
	if ( 0 == r ) {
		UHD_LOGGER_ERROR(CRIMSON_TNG_DEBUG_NAME_C) << "Failed to retrieve buffer level for channel " + std::string( 1, 'A' + channel ) + "\nCheck SFP port connections and cofiguration" << std::endl;
		throw new io_error( "Failed to retrieve buffer level for channel " + std::string( 1, 'A' + channel ) );
	}

	boost::endian::big_to_native_inplace( rsp.oflow );
	boost::endian::big_to_native_inplace( rsp.uflow );
	boost::endian::big_to_native_inplace( rsp.tv_sec );
	boost::endian::big_to_native_inplace( rsp.tv_tick );

    //fifo level provided by FPGA
	lvl = rsp.header & 0xffff;

    lvl = lvl * bl_multiple;

#ifdef BUFFER_LVL_DEBUG
    static uint32_t last[4];
    static uint32_t curr[4];
    last[channel] = curr[channel];
    curr[channel] = lvl;

    std::printf("%10u\t", lvl);
    if(channel == 3)
    {
        std::printf("%10u\t", last[0] - curr[0]);
        std::printf("%10u\t", last[1] - curr[1]);
        std::printf("%10u\t", last[2] - curr[2]);
        std::printf("%10u\t", last[3] - curr[3]);

        const uint32_t min = std::min(curr[0], std::min(curr[1], std::min(curr[2], curr[3])));
        const uint32_t max = std::max(curr[0], std::max(curr[1], std::max(curr[2], curr[3])));
        std::printf("%10u\t", max - min);
        std::printf("\n");
    }
#endif

	uflow = rsp.uflow & uint64_t( 0x0fffffffffffffff );
	oflow = rsp.oflow & uint64_t( 0x0fffffffffffffff );

	now = uhd::time_spec_t( rsp.tv_sec, rsp.tv_tick * tick_period_ps );

#ifdef UHD_TXRX_DEBUG_PRINTS
	std::stringstream ss;
	ss
			<< now << ": "
			<< (char)('A' + channel) << ": "
			<< '%' << std::dec << std::setw( 2 ) << std::setfill( ' ' ) << (unsigned)( pcnt * 100 )  << " "
			<< std::hex << std::setw( 4 ) << std::setfill( '0' ) << lvl << " "
			<< std::hex << std::setw( 16 ) << std::setfill( '0' ) << uflow << " "
			<< std::hex << std::setw( 16 ) << std::setfill( '0' ) << oflow << " "
			<< std::endl << std::flush;
	std::cout << ss.str();
#endif
}

tx_streamer::sptr crimson_tng_impl::get_tx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    if (args.otw_format != "sc16"){
        throw uhd::value_error("Crimson TNG TX cannot handle requested wire format: " + args.otw_format);
    }

    const size_t spp = CRIMSON_TNG_MAX_SEND_SAMPLE_BYTES/convert::get_bytes_per_item(args.otw_format);

    //make the new streamer given the samples per packet
    crimson_tng_send_packet_streamer::timenow_type timenow_ = std::bind( & crimson_tng_impl::get_time_now, this );

    std::vector<std::string> dst_ips(args.channels.size());
    std::vector<int> dst_ports(args.channels.size());
    for(size_t n = 0; n < args.channels.size(); n++) {
        uint16_t dst_port = 0;
        std::string sfp = "";
        get_tx_endpoint( _tree, args.channels[n], dst_ips[n], dst_port, sfp );
        dst_ports[n] = dst_port;
    }

    bool little_endian_supported = true;

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        size_t chan = args.channels[ chan_i ];
        const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
        const fs_path tx_path   = CRIMSON_TNG_MB_PATH / "tx";
        const fs_path tx_link_path  = CRIMSON_TNG_MB_PATH / "tx_link" / chan;

		// power on the channel
		_tree->access<std::string>(tx_path / chan / "pwr").set("1");

        if(little_endian_supported) {
            // enables endian swap (by default the packets are big endian, x86 CPUs are little endian)
            _tree->access<int>(tx_link_path / "endian_swap").set(1);
            // Checks if the server accepted the endian swap request
            // If 0 then the device does not support endian swap
            int endian_status = _tree->access<int>(tx_link_path / "endian_swap").get();
            if(endian_status == 0) {
                little_endian_supported = false;
            }
        } else {
            // Don't need to attempt to enable little endian for other channels if one has already failed, since they will all fail
        }

		// vita enable
		_tree->access<std::string>(tx_link_path / "vita_en").set("1");

        // Issue reset request to clear anything from initialization
        _tree->access<double>(tx_dsp_root(chan) + "/rstreq").set(1);
    }

    // Each streamer has its own FIFO buffer that can operate independantly
    // However there is a deprecated function in device for reading async message
    // To handle it, each streamer will have its own buffer and the device recv_async_msg will access the buffer from the most recently created streamer
    _async_msg_fifo = std::make_shared<bounded_buffer<async_metadata_t>>(1000/*Buffer contains 1000 messages*/);

    std::shared_ptr<crimson_tng_send_packet_streamer> my_streamer = std::make_shared<crimson_tng_send_packet_streamer>( args.channels, spp, CRIMSON_TNG_BUFF_SIZE , dst_ips, dst_ports, (int64_t) (CRIMSON_TNG_BUFF_PERCENT * CRIMSON_TNG_BUFF_SIZE), _async_msg_fifo, args.cpu_format, args.otw_format, little_endian_supported, tx_channel_in_use );

    //init some streamer stuff
    my_streamer->resize(args.channels.size());

    my_streamer->set_time_now_function(std::bind(&crimson_tng_impl::get_time_now,this));

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].tx_chan_occ;
            if (chan < num_chan_so_far){
                const size_t dsp = chan + _mbc[mb].tx_chan_occ - num_chan_so_far;
                my_streamer->set_channel_name(chan_i,std::string( 1, 'A' + chan ));

                my_streamer->set_on_fini(chan_i, std::bind( & tx_pwr_off, _tree, std::string( "/mboards/" + mb + "/tx/" + std::to_string( chan ) ) ) );

                my_streamer->set_xport_chan_fifo_lvl_abs(chan_i, std::bind(
                    &get_fifo_lvl_udp_abs, chan, CRIMSON_TNG_BUFF_SCALE, _mbc[mb].fifo_ctrl_xports[dsp], ph::_1, ph::_2, ph::_3, ph::_4
                ));

                _mbc[mb].tx_streamers[chan] = my_streamer; //store weak pointer
                break;
            }
        }
    }

    //sets all tick and samp rates on this streamer
    this->update_all_tx_rates();

    // Waits for time diff to converge
    wait_for_time_diff_converged();

    allocated_tx_streamers.push_back( my_streamer );
    ::atexit( shutdown_lingering_tx_streamers );

    return my_streamer;
}
