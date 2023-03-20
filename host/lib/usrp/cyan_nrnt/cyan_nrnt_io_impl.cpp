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

#include "cyan_nrnt_impl.hpp"
#include "cyan_nrnt_fw_common.h"
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

#include "../crimson_tng/system_time.hpp"

#if 0
  #ifndef UHD_TXRX_DEBUG_PRINTS
  #define UHD_TXRX_DEBUG_PRINTS
  #endif
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

//#define FLOW_CONTROL_DEBUG
//#define BUFFER_DEBUG

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace ph = std::placeholders;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

// XXX: @CF: 20180227: The only reason we need this class is issue STOP in ~()
class cyan_nrnt_recv_packet_streamer : public sph::recv_packet_streamer_mmsg {
public:
	typedef std::function<void(void)> onfini_type;

	cyan_nrnt_recv_packet_streamer(const std::vector<std::string>& dsp_ip, std::vector<int>& dst_port)
    //TODO: soft code bytes_per sample (3G has 12 bit samples, currently will only work for 1G). Also might need to change CYAN_NRNT_MAX_NBYTES to be dynamic
	: sph::recv_packet_streamer_mmsg(dsp_ip, dst_port, CYAN_NRNT_MAX_NBYTES, CYAN_NRNT_HEADER_SIZE, /*(bytes_per_sample)*/ 4)
	{
    }

	virtual ~cyan_nrnt_recv_packet_streamer() {
		teardown();
	}

    void issue_stream_cmd(const stream_cmd_t &stream_cmd)
    {
        return recv_packet_handler_mmsg::issue_stream_cmd(stream_cmd);
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
	}

private:

    struct eprops_type{
        onfini_type on_fini;
    };
    std::vector<eprops_type> _eprops;
};

static std::vector<std::weak_ptr<cyan_nrnt_recv_packet_streamer>> allocated_rx_streamers;
static void shutdown_lingering_rx_streamers() {
	// This is required as a workaround, because the relevent destructurs are not called
	// when you close the top block in gnu radio. Unsolved mystery for the time being.
	for( auto & rx: allocated_rx_streamers ) {
		if ( ! rx.expired() ) {
			std::shared_ptr<cyan_nrnt_recv_packet_streamer> my_streamer = rx.lock();
			if ( my_streamer ) {
				my_streamer->teardown();
			}
		}
	}
	allocated_rx_streamers.clear();
}


// XXX: @CF: 20180227: We need this for several reasons
// 1) need to power-down the tx channel (similar to sending STOP on rx) when the streamer is finalized
// 2) to wrap sphc::send_packet_streamer::send() and use our existing flow control algorithm
class cyan_nrnt_send_packet_streamer : public sph::send_packet_streamer_mmsg {
public:

	typedef std::function<void(void)> onfini_type;
	typedef std::function<uhd::time_spec_t(void)> timenow_type;
	typedef std::function<void(double&,uint64_t&,uint64_t&,uhd::time_spec_t&)> xport_chan_fifo_lvl_type;
    typedef std::function<void(uint64_t&,uint64_t&,uint64_t&,uhd::time_spec_t&)> xport_chan_fifo_lvl_abs_type;
	typedef std::function<bool(async_metadata_t&)> async_pusher_type;

    bool use_blocking_fc = false;
    uint64_t blocking_setpoint;
    // Maximum buffer level in nsamps. Named this way to avoid confusion with the same variable belonging to different stuff
    int64_t stream_max_bl;

	cyan_nrnt_send_packet_streamer(const std::vector<size_t>& channels, const size_t max_num_samps, const size_t max_bl)
	:
		sph::send_packet_streamer_mmsg( channels, max_num_samps, max_bl ),
		stream_max_bl(max_bl),
		_first_call_to_send( true ),
		_max_num_samps( max_num_samps ),
		_actual_num_samps( max_num_samps ),
		_samp_rate( 1.0 ),
		_streaming( false ),
		_stop_streaming( false )

	{
	}

	virtual ~cyan_nrnt_send_packet_streamer() {
		teardown();
	}

	void teardown() {
		stop_streaming();
		for( auto & ep: _eprops ) {
			if ( ep.on_fini ) {
				ep.on_fini();
			}
			std::cout << "CH " << ep.name << ": Overflow Count: " << ep.oflow << ", Underflow Count: " << ep.uflow << "\n";
		}
		_eprops.clear();
	}

    size_t get_num_channels(void) const{
        return this->size();
    }

    size_t get_max_num_samps(void) const{
        return _max_num_samps;
    }

    // The start time of the next batch of samples in ticks
    uhd::time_spec_t next_send_time;

    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff_,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    ){
        size_t nsamps_per_buff = ((size_t)(nsamps_per_buff_/CYAN_NRNT_PACKET_NSAMP_MULTIPLE)) * CYAN_NRNT_PACKET_NSAMP_MULTIPLE;

#ifdef UHD_TXRX_DEBUG_PRINTS
        if(nsamps_per_buff_ != nsamps_per_buff) {
            std::cout << "Warning: the number of samples attempted must be a multiple of " << CYAN_NRNT_PACKET_NSAMP_MULTIPLE << ", this will call will only attempt to send " << nsamps_per_buff << std::endl;
        }
#endif
        
        static const double default_sob = 1.0;

        size_t r = 0;

        uhd::tx_metadata_t metadata = metadata_;

        uhd::time_spec_t now = get_time_now();

        if ( _first_call_to_send ) {
            if ( ! metadata.start_of_burst ) {
                #ifdef UHD_TXRX_DEBUG_PRINTS
                std::cout << "Warning: first call to send but no start of burst!" << std::endl;
                #endif
                metadata.start_of_burst = true;
                metadata.time_spec = now + default_sob;
            }
        } else {
            if ( ! metadata.has_time_spec ) {
                metadata.has_time_spec = true;
                metadata.time_spec = next_send_time;
            }
        }

        if ( metadata.start_of_burst ) {
            for( auto & ep: _eprops ) {
                ep.flow_control->set_start_of_burst_time( metadata.time_spec );
            }
        }

        _first_call_to_send = false;

        // XXX: @CF: 20180320: Our strategy of predictive flow control is not 100% compatible with
        // the UHD API. As such, we need to bury this variable in order to pass it to check_fc_condition.
        // std::cout<<"nsamps_per_buff: " << nsamps_per_buff <<" Max samps: "<<_max_num_samps<< std::endl;

        _actual_num_samps = nsamps_per_buff > _max_num_samps ? _max_num_samps : nsamps_per_buff;

        now = get_time_now();

        start_packet_streamer_thread();

        if ( 0 == nsamps_per_buff && metadata.end_of_burst ) {
            #ifdef UHD_TXRX_DEBUG_PRINTS
            std::cout << "UHD::" CYAN_NRNT_DEBUG_NAME_C "::Info: " << now << ": " << "eob @ " << now << " | " << now.to_ticks( CYAN_NRNT_TICK_RATE ) << std::endl;
            #endif

            async_metadata_t am;
            am.has_time_spec = true;
            am.time_spec = now;
            am.event_code = async_metadata_t::EVENT_CODE_BURST_ACK;

            stop_streaming();
        }
        r = send_packet_handler::send(buffs, nsamps_per_buff, metadata, 0.00);

        next_send_time = metadata.time_spec + time_spec_t::from_ticks(r, _samp_rate);

        return r;
    }

    void enable_blocking_fc(uint64_t blocking_setpoint) {
        use_blocking_fc = true;
        if(blocking_setpoint > 0.9 * stream_max_bl) {
            blocking_setpoint = (uint64_t) (0.9 * stream_max_bl);
        };
        this->blocking_setpoint = blocking_setpoint;
    }

    void disable_blocking_fc() {
        use_blocking_fc = false;
        blocking_setpoint = 0;
    }
    
    static managed_send_buffer::sptr get_send_buff( std::weak_ptr<uhd::tx_streamer> tx_streamer, const size_t chan, double timeout ){

        std::shared_ptr<cyan_nrnt_send_packet_streamer> my_streamer =
            std::dynamic_pointer_cast<cyan_nrnt_send_packet_streamer>( tx_streamer.lock() );

        if (my_streamer.get() == NULL) return managed_send_buffer::sptr();

        //wait on flow control w/ timeout
        //if (not my_streamer->check_fc_condition( chan, timeout) ) return managed_send_buffer::sptr();

        //get a buffer from the transport w/ timeout
        managed_send_buffer::sptr buff = my_streamer->_eprops.at( chan ).xport_chan->get_send_buff( timeout );

        return buff;
    }

    static void update_fc_send_count( std::weak_ptr<uhd::tx_streamer> tx_streamer, const size_t chan, size_t nsamps ){

        std::shared_ptr<cyan_nrnt_send_packet_streamer> my_streamer =
            std::dynamic_pointer_cast<cyan_nrnt_send_packet_streamer>( tx_streamer.lock() );

        my_streamer->check_fc_update( chan, nsamps);
    }
    
    static bool check_flow_control(std::weak_ptr<uhd::tx_streamer> tx_streamer, const size_t chan, double timeout) {
        std::shared_ptr<cyan_nrnt_send_packet_streamer> my_streamer =
            std::dynamic_pointer_cast<cyan_nrnt_send_packet_streamer>( tx_streamer.lock() );

        return my_streamer->check_fc_condition( chan, timeout);
    }

    void set_on_fini( size_t chan, onfini_type on_fini ) {
		_eprops.at(chan).on_fini = on_fini;
    }
    void set_time_now( timenow_type time_now ) {
        _time_now = time_now;
    }
    uhd::time_spec_t get_time_now() {
        return _time_now ? _time_now() : get_system_time();
    }
    void set_xport_chan( size_t chan, uhd::transport::zero_copy_if::sptr xport ) {
		_eprops.at(chan).xport_chan = xport;
    }
    void set_xport_chan_fifo_lvl( size_t chan, xport_chan_fifo_lvl_type get_fifo_lvl ) {
		_eprops.at(chan).xport_chan_fifo_lvl = get_fifo_lvl;
    }
    void set_xport_chan_fifo_lvl_abs( size_t chan, xport_chan_fifo_lvl_abs_type get_fifo_lvl_abs ) {
		_eprops.at(chan).xport_chan_fifo_lvl_abs = get_fifo_lvl_abs;
    }
    void set_async_pusher( async_pusher_type pusher ) {
		async_pusher = pusher;
    }
    void set_channel_name( size_t chan, std::string name ) {
        _eprops.at(chan).name = name;
    }

    void resize(const size_t size){
		_eprops.resize( size );
		for( auto & ep: _eprops ) {
            // the nominal sample rate (first argument of make) is set later)
			ep.flow_control = uhd::flow_control_nonlinear::make( 1.0, CYAN_NRNT_BUFF_PERCENT, stream_max_bl );
			ep.flow_control->set_buffer_level( 0, get_time_now() );
		}
		sph::send_packet_handler::resize(size);
    }

    void set_samp_rate(const double rate){
        sph::send_packet_handler::set_samp_rate( rate );
        _samp_rate = rate;
        uhd::time_spec_t now = get_time_now();
        for( auto & ep: _eprops ) {
            if ( nullptr != ep.flow_control.get() ) {
                ep.flow_control->set_sample_rate( now, rate );
            }
        }
    }

    //create a new viking thread for each zc if (skryke!!)
    //starts the tx packet streamer, I think. This section all has viking names so I might be wrong and it might be rx
	void start_packet_streamer_thread() {
		// probably should also (re)start the "bm thread", which currently just manages time diff
		std::lock_guard<std::mutex> lck( _mutex );
		if ( ! _streaming ) {
			_stop_streaming = false;

            // Assuming pillage is called for each send(), and thus each stacked command,
            // the buffer level must be set to zero else flow control will crash since it thinks
            // the transfer buffer is already primed.
            for( auto & ep: _eprops ) {
                ep.flow_control->set_buffer_level(0, get_time_now());
            }

			//spawn a new viking to raid the send hoardes
			_streamer_thread = std::thread( cyan_nrnt_send_packet_streamer::send_viking_loop, this );
			_streaming = true;
		}
	}

	void stop_streaming() {
		// probably should also stop the "bm thread", which currently just manages time diff
		std::lock_guard<std::mutex> lock( _mutex );
		if ( _streaming ) {
			_stop_streaming = true;
			if ( _streamer_thread.joinable() ) {
				_streamer_thread.join();
				_streaming = false;
			}
		}
	}

private:
	bool _first_call_to_send;
    size_t _max_num_samps;
    size_t _actual_num_samps;
    double _samp_rate;
    bool _streaming;
    bool _stop_streaming;
    std::thread _streamer_thread;
    async_pusher_type async_pusher;
    timenow_type _time_now;
    std::mutex _mutex;

    // extended per-channel properties, beyond what is available in sphc::send_packet_handler::xport_chan_props_type
    struct eprops_type{
		onfini_type on_fini;
		uhd::transport::zero_copy_if::sptr xport_chan;
		xport_chan_fifo_lvl_type xport_chan_fifo_lvl;
        xport_chan_fifo_lvl_abs_type xport_chan_fifo_lvl_abs;
		uhd::flow_control::sptr flow_control;
		uint64_t oflow;
		uint64_t uflow;
        size_t _remaining_num_samps;
        std::mutex buffer_mutex;
        std::string name;
        eprops_type() : oflow( -1 ), uflow( -1 ) {}
        eprops_type( const eprops_type & other )
        :
            xport_chan( other.xport_chan ),
            xport_chan_fifo_lvl( other.xport_chan_fifo_lvl ),
            flow_control( other.flow_control ),
            oflow( other.oflow ),
            uflow( other.uflow )
        {}
    };
    std::vector<eprops_type> _eprops;

    void push_async_msg( uhd::async_metadata_t &async_metadata ){
		if ( async_pusher ) {
			async_pusher( async_metadata );
		}
    }

    void check_fc_update( const size_t chan, size_t nsamps) {
		_eprops.at( chan ).buffer_mutex.lock();
        _eprops.at( chan ).flow_control->update( nsamps, get_time_now() );
		_eprops.at( chan ).buffer_mutex.unlock();
    }
    
    // timeout must be small but non-zero, polling it at the max rate with result in something triggering,
    // which results in a 35ms pause every 1s
    // the send function in super_send_packet_handler should poll this function until it returns true
    
    bool check_fc_condition( const size_t chan, const double & timeout ) {

        if(BOOST_LIKELY(!use_blocking_fc)) {

            #ifdef UHD_TXRX_SEND_DEBUG_PRINTS
            static uhd::time_spec_t last_print_time( 0.0 ), next_print_time( get_time_now() );
            #endif

            uhd::time_spec_t now, then, dt;
            struct timespec req,rem;

            now = get_time_now();
            dt = _eprops.at( chan ).flow_control->get_time_until_next_send( _actual_num_samps, now );
            then = now + dt;

            if (( dt > timeout ) and (!_eprops.at( chan ).flow_control->start_of_burst_pending( now ))) {
    #ifdef UHD_TXRX_SEND_DEBUG_PRINTS
                std::cout << __func__ << ": returning false, search FLAG216" << std::endl;
                std::cout << "dt: " << dt << std::endl;
                std::cout << "dt.to_ticks: " << dt.to_ticks(CYAN_NRNT_TICK_RATE) << std::endl;
                std::cout << "dt.get_real_secs: " << dt.get_real_secs() << std::endl;
                std::cout << "timout: " << timeout << std::endl;
    #endif
                return false;
            }

            #ifdef UHD_TXRX_SEND_DEBUG_PRINTS
            if ( _eprops.at( chan ).flow_control->start_of_burst_pending( now ) || now >= next_print_time ) {
                last_print_time = now;
                next_print_time = last_print_time + 0.2;
                std::stringstream ss;
                ss << now << ": " << _eprops.at(chan).name << ": Queued " << std::dec << _actual_num_samps << " Buffer Level: " << std::dec << size_t( _eprops.at( chan ).flow_control->get_buffer_level_pcnt( now ) * 100 )  << "%, Time to next send: " << dt << std::endl << std::flush;
                std::cout << ss.str();
            }
            #endif

            // The time delta (dt) may be negative from the linear interpolator.
            // In such a case, do not bother with the delay calculations and send right away.
            if(dt <= 0.0) {
    #ifdef FLOW_CONTROL_DEBUG
                std::cout << __func__ << ": returning true, search FLAG655" << std::endl;
                std::cout << __func__ << ": R1: " << _eprops.at( chan ).flow_control->get_buffer_level_pcnt( now ) << std::endl;
    #endif
                return true;
            }

            req.tv_sec = (time_t) dt.get_full_secs();
            req.tv_nsec = dt.get_frac_secs()*1e9;

            nanosleep(&req, &rem);

            return true;
        }  else {
            uint64_t lvl = 0;
            uint64_t uflow = 0;
            uint64_t oflow = 0;
            uhd::time_spec_t then;
            _eprops.at( chan ).xport_chan_fifo_lvl_abs( lvl, uflow, oflow, then );
            bool send_now = lvl < blocking_setpoint;

            if(send_now) return true;
            else {
                // The function that calls function will call it repeatedly until it returns true. This delay creates some rest between checks
                // It is done here since in the normal mode its delay is determined by predicted time to send
                struct timespec req, rem;
                req.tv_sec = (time_t)(int64_t)timeout;
                req.tv_nsec = (time_t)(int64_t)((timeout - (int64_t)timeout)*1e9);
                nanosleep(&req, &rem);
                return false;
            }
        }
    }

    /***********************************************************************
     * Send Viking Loop
     * - while pillaging, raid for message packet
     * - update buffer levels
     * - update over / underflow counters
     * - put async message packets into queue
     **********************************************************************/
	static void send_viking_loop( cyan_nrnt_send_packet_streamer *self ) {
		// pillage! plunder! (S)he who peaks at the buffer levels, will find her or his way to Valhalla!

		// std::cout << __func__ << "(): beginning viking loop for tx streamer @ " << (void *) self << std::endl;

		for( ; ! self->_stop_streaming; ) {

			const auto t0 = std::chrono::high_resolution_clock::now();

			for( size_t i = 0; i < self->_eprops.size(); i++ ) {

				eprops_type & ep = self->_eprops[ i ];

				xport_chan_fifo_lvl_type get_fifo_level;
				uhd::flow_control::sptr fc;

				get_fifo_level = ep.xport_chan_fifo_lvl;
				fc = ep.flow_control;

				if ( !( get_fifo_level && fc.get() ) ) {
					continue;
				}

				uhd::time_spec_t now, then;
				double level_pcnt;
				uint64_t uflow;
				uint64_t oflow;
				async_metadata_t metadata;

				size_t max_level = fc->get_buffer_size();

				try {
					get_fifo_level( level_pcnt, uflow, oflow, then );
				} catch( ... ) {

#ifdef DEBUG_FC
				  std::printf("%10d\t", -1);
#endif
					continue;
				}

				if ( self->_stop_streaming ) {
					break;
				}

				now = self->get_time_now();

				size_t level = level_pcnt * max_level;
#ifdef BUFFER_DEBUG
                std::cout << __func__ << ": max_level: " << max_level << std::endl;
                std::cout << __func__ << ": level_pcnt: " << level_pcnt << std::endl;
#endif

				if ( ! fc->start_of_burst_pending( then ) ) {
					level -= ( now - then ).get_real_secs() / self->_samp_rate;
#ifdef BUFFER_DEBUG
                    std::cout << __func__ << ": level: " << level << std::endl;
#endif
					fc->set_buffer_level( level, now );
#ifdef DEBUG_FC
				    std::printf("%10lu\t", level);
#endif
				}
#ifdef DEBUG_FC
				std::printf("%10ld\t", uflow);
#endif
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

#ifdef DEBUG_FC
				std::printf("%10ld", oflow);
#endif
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
			const long long usloop = 1.0 / (double)CYAN_NRNT_UPDATE_PER_SEC * 1e6;
			const long long usdelay = usloop - us;

#ifdef DEBUG_FC
			std::printf("%10lld %10lld %10lld\n", us, usloop, usdelay);
#endif

#ifdef UHD_TXRX_DEBUG_TIME
			::usleep( 200000 );
#else
			::usleep( usdelay < 0 ? 0 : usdelay );
#endif
		}
		//std::cout << __func__ << "(): ending viking loop for tx streamer @ " << (void *) self << std::endl;
	}
};

static std::vector<std::weak_ptr<cyan_nrnt_send_packet_streamer>> allocated_tx_streamers;
static void shutdown_lingering_tx_streamers() {
	// This is required as a workaround, because the relevent destructurs are not called
	// when you close the top block in gnu radio. Unsolved mystery for the time being.
	for( auto & tx: allocated_tx_streamers ) {
		if ( ! tx.expired() ) {
			std::shared_ptr<cyan_nrnt_send_packet_streamer> my_streamer = tx.lock();
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
 * io impl details (internal to this file)
 * - alignment buffer
 **********************************************************************/
struct cyan_nrnt_impl::cyan_nrnt_io_impl{

    cyan_nrnt_io_impl(void):
        async_msg_fifo(1000/*messages deep*/)
    {
        /* NOP */
    }

    ~cyan_nrnt_io_impl(void){
    }

    //methods and variables for the viking scourge
    bounded_buffer<async_metadata_t> async_msg_fifo;

    // TODO: @CF: 20180301: move time diff code into cyan_nrnt_io_impl
};

/***********************************************************************
 * Helper Functions
 **********************************************************************/
void cyan_nrnt_impl::io_init(void){

	// TODO: @CF: 20180301: move time diff code into cyan_nrnt_io_impl
	_cyan_nrnt_io_impl = UHD_PIMPL_MAKE(cyan_nrnt_io_impl, ());

    //allocate streamer weak ptrs containers
    for (const std::string &mb : _mbc.keys()) {
        _mbc[mb].rx_streamers.resize( num_rx_channels );
        _mbc[mb].tx_streamers.resize( num_tx_channels );
    }
}

void cyan_nrnt_impl::update_rx_samp_rate(const std::string &mb, const size_t dsp, const double rate_){

    set_double( "rx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate", rate_ );
    double rate = get_double( "rx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate" );

    std::shared_ptr<cyan_nrnt_recv_packet_streamer> my_streamer =
        std::dynamic_pointer_cast<cyan_nrnt_recv_packet_streamer>(_mbc[mb].rx_streamers[dsp].lock());
    if (my_streamer.get() == NULL) return;

    my_streamer->set_samp_rate(rate);
    my_streamer->set_tick_rate( CYAN_NRNT_TICK_RATE );
}

void cyan_nrnt_impl::update_tx_samp_rate(const std::string &mb, const size_t dsp, const double rate_ ){

    set_double( "tx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate", rate_ );
    double rate = get_double( "tx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate" );

	std::shared_ptr<cyan_nrnt_send_packet_streamer> my_streamer =
        std::dynamic_pointer_cast<cyan_nrnt_send_packet_streamer>(_mbc[mb].tx_streamers[dsp].lock());
    if (my_streamer.get() == NULL) return;

    my_streamer->set_samp_rate(rate);
    my_streamer->set_tick_rate( CYAN_NRNT_TICK_RATE );
}

void cyan_nrnt_impl::update_rates(void){
    for (const std::string &mb : _mbc.keys()) {
        fs_path root = "/mboards/" + mb;
        _tree->access<double>(root / "tick_rate").update();
        if(num_rx_channels > 0) {
            //and now that the tick rate is set, init the host rates to something
            for(const std::string &name : _tree->list(root / "rx_dsps")) {
                // XXX: @CF: 20180301: on the server, we currently turn rx power (briefly) on any time that rx properties are set.
                // if the current application does not require rx, then we should not enable it
                // just checking for power is not a great way to do this, but it mostly works
                if ( "1" == _tree->access<std::string>( root / "rx" / name / "pwr").get() ) {
                    _tree->access<double>(root / "rx_dsps" / name / "rate" / "value").update();
                }
            }
        }

        if(num_tx_channels > 0) {
            for(const std::string &name : _tree->list(root / "tx_dsps")) {
                // XXX: @CF: 20180301: on the server, we currently turn tx power on any time that tx properties are set.
                // if the current application does not require tx, then we should not enable it
                // just checking for power is not a great way to do this, but it mostly works
                if ( "1" == _tree->access<std::string>( root / "tx" / name / "pwr").get() ) {
                    _tree->access<double>(root / "tx_dsps" / name / "rate" / "value").update();
                }
            }
        }
    }
}

void cyan_nrnt_impl::update_rx_subdev_spec(const std::string &which_mb, const subdev_spec_t &spec){
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

void cyan_nrnt_impl::update_tx_subdev_spec(const std::string &which_mb, const subdev_spec_t &spec){
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
bool cyan_nrnt_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    boost::this_thread::disable_interruption di; //disable because the wait can throw
    return _cyan_nrnt_io_impl->async_msg_fifo.pop_with_timed_wait(async_metadata, timeout);
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr cyan_nrnt_impl::get_rx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? otw_rx_s : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    if (args.otw_format != otw_rx_s){
        throw uhd::value_error(CYAN_NRNT_DEBUG_NAME_S " RX cannot handle requested wire format: " + args.otw_format);
    }

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;

    std::vector<std::string> dst_ip(args.channels.size());
    for(size_t n = 0; n < dst_ip.size(); n++) {
        dst_ip[n] = _tree->access<std::string>( rx_link_root(args.channels[n]) + "/ip_dest" ).get();
    }

    std::vector<int> dst_port(args.channels.size());
    for(size_t n = 0; n < dst_port.size(); n++) {
        dst_port[n] = std::stoi(_tree->access<std::string>( rx_link_root(args.channels[n]) + "/port" ).get());
    }

    //make the new streamer given the samples per packet
    std::shared_ptr<cyan_nrnt_recv_packet_streamer> my_streamer = std::make_shared<cyan_nrnt_recv_packet_streamer>(dst_ip, dst_port);

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_unpacker(&vrt::if_hdr_unpack_be);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.otw_format + "_item32_be";
    id.num_inputs = 1;
    id.output_format = args.cpu_format;
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    if ( "fc32" == args.cpu_format ) {
        my_streamer->set_scale_factor( 1.0 / (double)((1<<(otw_rx-1))-1) );
    } else if ( "sc16" == args.cpu_format ) {
        my_streamer->set_scale_factor( 16.0 / otw_rx-1 );
    }

    // XXX: @CF: 20180424: Also nasty.. if crimson did not shut down properly last time
    // then the "/*flush*/" below will not work unless we turn it off ahead of time.
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){

                const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
                std::string num     = boost::lexical_cast<std::string>((char)(chan + 'A'));
                const fs_path mb_path   = "/mboards/" + mb;
                const fs_path rx_path   = mb_path / "rx";
                const fs_path rx_fe_path    = mb_path / "dboards" / num / "rx_frontends" / ch;
                const fs_path rx_link_path  = mb_path / "rx_link" / chan;
                const fs_path rx_dsp_path   = mb_path / "rx_dsps" / chan;

                // stop streaming
                _tree->access<std::string>(rx_path / chan / "stream").set("0");
                // vita enable
                _tree->access<std::string>(rx_link_path / "vita_en").set("1");
            }
        }
    }

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){
                const size_t dsp = chan + _mbc[mb].rx_chan_occ - num_chan_so_far;
                std::string scmd_pre( "rx_" + std::string( 1, 'a' + chan ) + "/stream" );
                /* XXX: @CF: 20180321: This causes QA to issue 'd' and then 'o' and fail.
                 * Shouldn't _really_ need it here, but it was originally here to shut down
                 * the channel in case it was not previously shut down
                 */
                //stream_cmd_t scmd( stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS );
                //scmd.stream_now = true;
                //set_stream_cmd( scmd_pre, scmd );
                //
//                 my_streamer->set_xport_chan_get_buff(chan_i, std::bind(
//                     &zero_copy_if::get_recv_buff, _mbc[mb].rx_dsp_xports[dsp], ph::_1, ph::_2
//                 ), true /*flush*/);
                my_streamer->set_issue_stream_cmd(chan_i, std::bind(
                    &cyan_nrnt_impl::set_stream_cmd, this, scmd_pre, ph::_1));
                my_streamer->set_on_fini(chan_i, std::bind( & rx_pwr_off, _tree, std::string( "/mboards/" + mb + "/rx/" + std::to_string( chan ) ) ) );
                _mbc[mb].rx_streamers[chan] = my_streamer; //store weak pointer
                break;
            }
        }
    }

    //set the packet threshold to be an entire socket buffer's worth
    const size_t packets_per_sock_buff = size_t(50e6/9000);//_mbc[_mbc.keys().front()].rx_dsp_xports[0]->get_recv_frame_size());
    my_streamer->set_alignment_failure_threshold(packets_per_sock_buff);

    // XXX: @CF: 20170227: extra setup for crimson
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){

                // XXX: @CF: this is so nasty..
                const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
                std::string num     = boost::lexical_cast<std::string>((char)(chan + 'A'));
                const fs_path mb_path   = "/mboards/" + mb;
                const fs_path rx_path   = mb_path / "rx";
                const fs_path rx_fe_path    = mb_path / "dboards" / num / "rx_frontends" / ch;
                const fs_path rx_link_path  = mb_path / "rx_link" / chan;
                const fs_path rx_dsp_path   = mb_path / "rx_dsps" / chan;

                _tree->access<std::string>(rx_path / chan / "stream").set("0");
                // vita enable
                _tree->access<std::string>(rx_link_path / "vita_en").set("1");

                // power on the channel
                _tree->access<std::string>(rx_path / chan / "pwr").set("1");
                // XXX: @CF: 20180214: Do we _really_ need to sleep 1/2s for power on for each channel??
                //usleep( 500000 );
                // stream enable
                _tree->access<std::string>(rx_path / chan / "stream").set("1");

// FIXME: @CF: 20180316: our TREE macros do not populate update(), unfortunately
#define _update( t, p ) \
    _tree->access<t>( p ).set( _tree->access<t>( p ).get() )

                _update( int, rx_fe_path / "freq" / "band" );
            }
        }
    }

    //sets all tick and samp rates on this streamer
    this->update_rates();

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){

                const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
                const fs_path mb_path   = "/mboards/" + mb;
                const fs_path rx_path   = mb_path / "rx";

                _tree->access<std::string>(rx_path / chan / "jesd/status").set("1");
                std::string jesd_status = _tree->access<std::string>(rx_path / chan / "jesd/status").get();
                if(jesd_status.compare(0, 4, "good")) {
                    UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "rx " << ch << ": unable to establish JESD link. This streamer will not work." << std::endl;
                }
            }
        }
    }

    // XXX: @CF: 20180117: Give any transient errors in the time-convergence PID loop sufficient time to subsidte. KB 4312
	for( ;! time_diff_converged() && _bm_thread_needed; ) {
		usleep( 10000 );
	}

    allocated_rx_streamers.push_back( my_streamer );
    ::atexit( shutdown_lingering_rx_streamers );

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/

static void get_fifo_lvl_udp_abs( const size_t channel, const int64_t bl_multiple, uhd::transport::udp_simple::sptr xport, uint64_t & lvl, uint64_t & uflow, uint64_t & oflow, uhd::time_spec_t & now ) {

	static constexpr double tick_period_ps = 1.0 / CYAN_NRNT_TICK_RATE;

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
		UHD_LOGGER_ERROR(CYAN_NRNT_DEBUG_NAME_C) << "Failed to retrieve buffer level for channel " + std::string( 1, 'A' + channel ) + "\nCheck SFP port connections and cofiguration" << std::endl;
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

static void get_fifo_lvl_udp( const size_t channel, const int64_t max_bl, const int64_t bl_multiple, uhd::transport::udp_simple::sptr xport, double & pcnt, uint64_t & uflow, uint64_t & oflow, uhd::time_spec_t & now ) {
    uint64_t lvl = 0;
    get_fifo_lvl_udp_abs( channel, bl_multiple, xport, lvl, uflow, oflow, now);
    pcnt = (double) lvl / max_bl;
}

tx_streamer::sptr cyan_nrnt_impl::get_tx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? otw_tx_s : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    if (args.otw_format != otw_tx_s){
        throw uhd::value_error(CYAN_NRNT_DEBUG_NAME_S " TX cannot handle requested wire format: " + args.otw_format);
    }

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt_send_header_offset_words32*sizeof(uint32_t)
        + vrt::max_if_hdr_words32*sizeof(uint32_t)
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().sid) //no stream id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
        ;

    const size_t bpp = _mbc[_mbc.keys().front()].tx_dsp_xports[0]->get_send_frame_size() - hdr_size;
    const size_t spp = bpp/convert::get_bytes_per_item(args.otw_format);

    //make the new streamer given the samples per packet
    cyan_nrnt_send_packet_streamer::timenow_type timenow_ = std::bind( & cyan_nrnt_impl::get_time_now, this );
    std::vector<uhd::transport::zero_copy_if::sptr> xports;
    for( auto & i: args.channels ) {
        xports.push_back( _mbc[ _mbc.keys().front() ].tx_dsp_xports[ i ] );
    }
    std::shared_ptr<cyan_nrnt_send_packet_streamer> my_streamer = std::make_shared<cyan_nrnt_send_packet_streamer>( args.channels, spp, max_buffer_level );

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_packer(&vrt::if_hdr_pack_be, vrt_send_header_offset_words32);
    my_streamer->set_enable_trailer( false );

    my_streamer->set_time_now(std::bind(&cyan_nrnt_impl::get_time_now,this));

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = 1;
    id.output_format = args.otw_format + "_item32_be";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    if ( false ) {
    } else if ( "fc32" == args.cpu_format ) {
        my_streamer->set_scale_factor( (double)((1<<(otw_tx-1))-1) );
    } else if ( "sc16" == args.cpu_format ) {
        my_streamer->set_scale_factor( otw_tx / 16.0 );
    }

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

                std::weak_ptr<uhd::tx_streamer> my_streamerp = my_streamer;

                my_streamer->set_xport_chan_get_buff(chan_i, std::bind(
                    &cyan_nrnt_send_packet_streamer::get_send_buff, my_streamerp, chan_i, ph::_1
                ));

                my_streamer->set_xport_chan(chan_i,_mbc[mb].tx_dsp_xports[dsp]);
                my_streamer->set_xport_chan_update_fc_send_size(chan_i, std::bind(
                    &cyan_nrnt_send_packet_streamer::update_fc_send_count, my_streamerp, chan_i, ph::_1
                ));
                
                my_streamer->set_xport_chan_check_flow_control(chan_i, std::bind(
                    &cyan_nrnt_send_packet_streamer::check_flow_control, my_streamerp, chan_i, ph::_1
                ));

                my_streamer->set_xport_chan_fifo_lvl(chan_i, std::bind(
                    &get_fifo_lvl_udp, chan, max_buffer_level, buffer_level_multiple, _mbc[mb].fifo_ctrl_xports[dsp], ph::_1, ph::_2, ph::_3, ph::_4
                ));

                my_streamer->set_xport_chan_fifo_lvl_abs(chan_i, std::bind(
                    &get_fifo_lvl_udp_abs, chan, buffer_level_multiple, _mbc[mb].fifo_ctrl_xports[dsp], ph::_1, ph::_2, ph::_3, ph::_4
                ));

                my_streamer->set_async_receiver(std::bind(&bounded_buffer<async_metadata_t>::pop_with_timed_wait, &(_cyan_nrnt_io_impl->async_msg_fifo), ph::_1, ph::_2));

                my_streamer->set_async_pusher(std::bind(&bounded_buffer<async_metadata_t>::push_with_pop_on_full, &(_cyan_nrnt_io_impl->async_msg_fifo), ph::_1));

                _mbc[mb].tx_streamers[chan] = my_streamer; //store weak pointer
                break;
            }
        }
    }

    // XXX: @CF: 20170228: extra setup for crimson
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        size_t chan = args.channels[ chan_i ];
        const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
        const fs_path mb_path   = "/mboards/0";
        const fs_path tx_path   = mb_path / "tx";
        const fs_path tx_link_path  = mb_path / "tx_link" / chan;

		// power on the channel
        //_tree->access<std::string>(tx_path / ch / "pwr").set("0");
		_tree->access<std::string>(tx_path / chan / "pwr").set("1");
		// XXX: @CF: 20180214: Do we _really_ need to sleep 1/2s for power on for each channel??
		//usleep( 500000 );
		// vita enable
		_tree->access<std::string>(tx_link_path / "vita_en").set("1");
    }

    //sets all tick and samp rates on this streamer
    this->update_rates();
    
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        for (const std::string &mb : _mbc.keys()) {
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){

                const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
                const fs_path mb_path   = "/mboards/" + mb;
                const fs_path tx_path   = mb_path / "tx";

                _tree->access<std::string>(tx_path / chan / "jesd/status").set("1");
                std::string jesd_status = _tree->access<std::string>(tx_path / chan / "jesd/status").get();
                if(jesd_status.compare(0, 4, "good")) {
                    UHD_LOGGER_WARNING(CYAN_NRNT_DEBUG_NAME_C) << "tx " << ch << ": unable to establish JESD link. This streamer will not work." << std::endl;
                }
            }
        }
    }

    // XXX: @CF: 20180117: Give any transient errors in the time-convergence PID loop sufficient time to subsidte. KB 4312
	for( ;! time_diff_converged(); ) {
		usleep( 10000 );
	}
    //my_streamer->start_packet_streamer_thread();

    allocated_tx_streamers.push_back( my_streamer );
    ::atexit( shutdown_lingering_tx_streamers );

    return my_streamer;
}
