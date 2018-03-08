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

#include <iomanip>
#include <mutex>

#include "validate_subdev_spec.hpp"
#include "async_packet_handler.hpp"
#include "../../transport/super_recv_packet_handler.hpp"
#include "../../transport/super_send_packet_handler.hpp"
#include "crimson_tng_impl.hpp"
#include "crimson_tng_fw_common.h"
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <thread>
#include <vector>

#include <boost/endian/buffers.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

/***********************************************************************
 * helpers
 **********************************************************************/

std::ostream & operator<<( std::ostream & os, const uhd::time_spec_t & ts ) {
	os << ts.get_real_secs();
	return os;
}

// XXX: @CF: 20180227: The only reason we need this class is issue STOP in ~()
class crimson_tng_recv_packet_streamer : public sph::recv_packet_streamer {
public:
	crimson_tng_recv_packet_streamer(const size_t max_num_samps)
	: sph::recv_packet_streamer( max_num_samps )
	{
        _max_num_samps = max_num_samps;
    }

	virtual ~crimson_tng_recv_packet_streamer() {
		static const stream_cmd_t cmd( stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS );
		issue_stream_cmd( cmd );
	}

    size_t get_num_channels(void) const{
        return this->size();
    }

    size_t get_max_num_samps(void) const{
        return _max_num_samps;
    }

    size_t recv(
        const rx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t &metadata,
        const double timeout,
        const bool one_packet
    ){
        return recv_packet_handler::recv(buffs, nsamps_per_buff, metadata, timeout, one_packet);
    }

    void issue_stream_cmd(const stream_cmd_t &stream_cmd)
    {
        return recv_packet_handler::issue_stream_cmd(stream_cmd);
    }

private:
    size_t _max_num_samps;
};

// XXX: @CF: 20180227: We need this for several reasons
// 1) need to power-down the tx channel (similar to sending STOP on rx) when the streamer is finalized
// 2) to wrap sph::send_packet_streamer::send() and use our existing flow control algorithm
class crimson_tng_send_packet_streamer : public sph::send_packet_streamer {
public:

	typedef boost::function<void(void)> onfini_type;
	typedef boost::function<uhd::time_spec_t(void)> timenow_type;
	typedef boost::function<void(double&,uint64_t&,uint64_t&,uhd::time_spec_t&)> xport_chan_fifo_lvl_type;
	typedef boost::function<bool(async_metadata_t&)> async_pusher_type;

	crimson_tng_send_packet_streamer( const size_t max_num_samps )
	:
		sph::send_packet_streamer( max_num_samps ),
		_first_call_to_send( true ),
		_max_num_samps( max_num_samps ),
		_blessbless( false ) // icelandic (viking) for bye
	{
	}

	virtual ~crimson_tng_send_packet_streamer() {
		_blessbless = true;
		if ( _pillage_thread.joinable() ) {
			_pillage_thread.join();
		}
		for( auto & ep: _eprops ) {
			if ( ep.on_fini ) {
				ep.on_fini();
			}
		}
		_eprops.clear();
	}

    size_t get_num_channels(void) const{
        return this->size();
    }

    size_t get_max_num_samps(void) const{
        return _max_num_samps;
    }

    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    ){
        size_t r;

        uhd::tx_metadata_t metadata = metadata_;

        if ( _first_call_to_send && ! metadata.start_of_burst ) {
            uhd::time_spec_t now = get_time_now();
            //UHD_MSG( error ) << "Warning: first call to send but no start of burst!" << std::endl;
            metadata.start_of_burst = true;
            metadata.has_time_spec = true;
            metadata.time_spec = now + 0.01;
            for( auto & ep: _eprops ) {
                //std::cout << "Set SoB Time to " << metadata.time_spec << std::endl;
                ep.flow_control->set_buffer_level( 0, now );
            }
        }

        if ( metadata.start_of_burst ) {
            if ( ! metadata.has_time_spec ) {
                UHD_MSG( error ) << "Warning: first call to send but no time spec supplied" << std::endl;
            }
            for( auto & ep: _eprops ) {
				//std::cout << "Set SoB Time to " << metadata.time_spec << std::endl;
				ep.flow_control->set_start_of_burst_time( metadata.time_spec );
            }
        }
        _first_call_to_send = false;
        r = send_packet_handler::send(buffs, nsamps_per_buff, metadata, timeout);

        if ( metadata.end_of_burst && ( 0 == nsamps_per_buff || nsamps_per_buff == r ) ) {

            async_metadata_t am;
            am.has_time_spec = true;
            am.time_spec = get_time_now();
            am.event_code = async_metadata_t::EVENT_CODE_BURST_ACK;
            for( size_t i = 0; i < _eprops.size(); i++ ) {
				am.channel = i;
				push_async_msg( am );
            }

            _blessbless = true;
        }

        return r;
    }

    managed_send_buffer::sptr get_send_buff( const size_t chan, const size_t samples, double timeout ){

        //wait on flow control w/ timeout
        if (not check_fc_condition( chan, samples, timeout) ) return managed_send_buffer::sptr();

        //get a buffer from the transport w/ timeout
        managed_send_buffer::sptr buff = _eprops.at( chan ).xport_chan->get_send_buff( timeout );

        return buff;
    }

    void set_on_fini( size_t chan, onfini_type on_fini ) {
    	_eprops.at(chan).on_fini = on_fini;
    }
    void set_time_now( timenow_type time_now ) {
    	_time_now = time_now;
    }
    uhd::time_spec_t get_time_now() {
    	return _time_now ? _time_now() : uhd::time_spec_t::get_system_time();
    }
    void set_xport_chan( size_t chan, uhd::transport::zero_copy_if::sptr xport ) {
    	_eprops.at(chan).xport_chan = xport;
    }
    void set_xport_chan_fifo_lvl( size_t chan, xport_chan_fifo_lvl_type get_fifo_lvl ) {
    	_eprops.at(chan).xport_chan_fifo_lvl = get_fifo_lvl;
    }
    void set_async_pusher( async_pusher_type pusher ) {
    	async_pusher = pusher;
    }

    void resize(const size_t size){
		_eprops.resize( size );
		for( auto & ep: _eprops ) {
			ep.flow_control = uhd::flow_control_nonlinear::make( 1.0, 0.8, CRIMSON_TNG_BUFF_SIZE );
			ep.flow_control->set_buffer_level( 0, get_time_now() );
		}
    	sph::send_packet_handler::resize(size);
    }

    void set_samp_rate(const double rate){
        sph::send_packet_handler::set_samp_rate( rate );

        uhd::time_spec_t now = get_time_now();
        for( auto & ep: _eprops ) {
            if ( nullptr != ep.flow_control.get() ) {
                ep.flow_control->set_sample_rate( now, rate );
            }
        }
    }

    //create a new viking thread for each zc if (skryke!!)
	void pillage() {
		//spawn a new viking to raid the send hoardes
		_pillage_thread = std::thread( crimson_tng_send_packet_streamer::send_viking_loop, this );
	}

private:
	bool _first_call_to_send;
	std::mutex _buffer_mutex;
    size_t _max_num_samps;
    bool _blessbless;
    std::thread _pillage_thread;
    async_pusher_type async_pusher;
    timenow_type _time_now;

    // extended per-channel properties, beyond what is available in sph::send_packet_handler::xport_chan_props_type
    struct eprops_type{
    	onfini_type on_fini;
    	uhd::transport::zero_copy_if::sptr xport_chan;
    	xport_chan_fifo_lvl_type xport_chan_fifo_lvl;
    	uhd::flow_control::sptr flow_control;
    	uint64_t oflow;
    	uint64_t uflow;
    	eprops_type() : oflow( -1 ), uflow( -1 ) {}
    };
    std::vector<eprops_type> _eprops;

    void push_async_msg( uhd::async_metadata_t &async_metadata ){
    	if ( async_pusher ) {
    		async_pusher( async_metadata );
    	}
    }

    bool check_fc_condition( const size_t chan, const size_t samples, const double & timeout ) {
        uhd::time_spec_t now, then, dt;

        _buffer_mutex.lock();

        now = get_time_now();
        dt = _eprops.at( chan ).flow_control->get_time_until_next_send( samples, now );
        then = now + dt;

        if ( dt > timeout ) {
            _buffer_mutex.unlock();
            return false;
        }

        //std::cout << "Buffer Level: " << size_t( _eprops.at( chan ).flow_control->get_buffer_level_pcnt( now ) * 100 )  << "%, Time to next send: " << dt << std::endl;
        _eprops.at( chan ).flow_control->update( samples, now );

        if ( dt <= 0.0 ) {
            _buffer_mutex.unlock();
            return true;
        }

		// XXX: @CF: 20170717: Instead of hard-coding values here, calibrate the delay loop on init using a method similar to rt_tests/cyclictest
		if ( dt.get_real_secs() > 100e-6 ) {
			dt -= 30e-6;
			struct timespec req, rem;
			req.tv_sec = (time_t) dt.get_full_secs();
			req.tv_nsec = dt.get_frac_secs()*1e9;
			nanosleep( &req, &rem );
		}
		for(
			now = get_time_now();
			now < then;
			now = get_time_now()
		) {
			// nop
			__asm__ __volatile__( "" );
			now += 1.0;
		}

		_buffer_mutex.unlock();
    	return true;
    }

    /***********************************************************************
     * Send Viking Loop
     * - while pillaging, raid for message packet
     * - update buffer levels
     * - update over / underflow counters
     * - put async message packets into queue
     **********************************************************************/
	static void send_viking_loop( crimson_tng_send_packet_streamer *self ) {
		// pillage! plunder! (S)he who peaks at the buffer levels, will find her or his way to Valhalla!

		// std::cout << __func__ << "(): beginning viking loop for tx streamer @ " << (void *) self << std::endl;

		for( ; ! self->_blessbless; ) {
			::usleep( 1.0 / (double)CRIMSON_TNG_UPDATE_PER_SEC * 1e6 );
			for( size_t i = 0; i < self->_eprops.size(); i++ ) {
				eprops_type & ep = self->_eprops[ i ];

				xport_chan_fifo_lvl_type get_fifo_level;
				uhd::flow_control::sptr fc;

				get_fifo_level = ep.xport_chan_fifo_lvl;
				fc = ep.flow_control;

				if ( !( get_fifo_level && fc.get() ) ) {
					continue;
				}

				uhd::time_spec_t now;
				double level_pcnt;
				uint64_t uflow;
				uint64_t oflow;
				async_metadata_t metadata;

				size_t max_level = fc->get_buffer_size();

				get_fifo_level( level_pcnt, uflow, oflow, now );

				if ( self->_blessbless ) {
					break;
				}

				size_t level = level_pcnt * max_level;
				fc->set_buffer_level( level, now );

				if ( (uint64_t)-1 == ep.uflow && uflow != ep.uflow ) {
					// XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
		            // async_metadata_t metadata;
		            // load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
					metadata.channel = i;
					metadata.has_time_spec = true;
					metadata.time_spec = now;
					metadata.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
					// assumes that underflow counter is monotonically increasing
					self->push_async_msg( metadata );
				}
				ep.uflow = uflow;

				if ( (uint64_t)-1 == ep.oflow && oflow != ep.oflow ) {
					// XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
		            // async_metadata_t metadata;
		            // load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
					metadata.channel = i;
					metadata.has_time_spec = true;
					metadata.time_spec = now;
					metadata.event_code = uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR;
					// assumes that overflow counter is monotonically increasing
					self->push_async_msg( metadata );
				}
				ep.oflow = oflow;
			}
		}
        for( auto & ep: self->_eprops ) {
            ep.on_fini();
        }
		//std::cout << __func__ << "(): ending viking loop for tx streamer @ " << (void *) self << std::endl;
	}
};

/***********************************************************************
 * constants
 **********************************************************************/
static const size_t vrt_send_header_offset_words32 = 0;

/***********************************************************************
 * io impl details (internal to this file)
 * - alignment buffer
 **********************************************************************/
struct crimson_tng_impl::io_impl{

    io_impl(void):
        async_msg_fifo(1000/*messages deep*/)
    {
        /* NOP */
    }

    ~io_impl(void){
    }

    //methods and variables for the viking scourge
    bounded_buffer<async_metadata_t> async_msg_fifo;

    // TODO: @CF: 20180301: move time diff code into io_impl
};

/***********************************************************************
 * Helper Functions
 **********************************************************************/
void crimson_tng_impl::io_init(void){

	// TODO: @CF: 20180301: move time diff code into io_impl
	_io_impl = UHD_PIMPL_MAKE(io_impl, ());

    //allocate streamer weak ptrs containers
    BOOST_FOREACH(const std::string &mb, _mbc.keys()){
        _mbc[mb].rx_streamers.resize( CRIMSON_TNG_RX_CHANNELS );
        _mbc[mb].tx_streamers.resize( CRIMSON_TNG_TX_CHANNELS );
    }
}

void crimson_tng_impl::update_rx_samp_rate(const std::string &mb, const size_t dsp, const double rate_){

    set_double( "rx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate", rate_ );
    double rate = get_double( "rx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate" );

    boost::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<crimson_tng_recv_packet_streamer>(_mbc[mb].rx_streamers[dsp].lock());
    if (my_streamer.get() == NULL) return;

    my_streamer->set_samp_rate(rate);
    my_streamer->set_tick_rate( CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0 );
}

void crimson_tng_impl::update_tx_samp_rate(const std::string &mb, const size_t dsp, const double rate_ ){

    set_double( "tx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate", rate_ );
    double rate = get_double( "tx_" + std::string( 1, 'a' + dsp ) + "/dsp/rate" );

	boost::shared_ptr<crimson_tng_send_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<crimson_tng_send_packet_streamer>(_mbc[mb].tx_streamers[dsp].lock());
    if (my_streamer.get() == NULL) return;

    my_streamer->set_samp_rate(rate);
    my_streamer->set_tick_rate( CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0 );
}

void crimson_tng_impl::update_rates(void){
    BOOST_FOREACH(const std::string &mb, _mbc.keys()){
        fs_path root = "/mboards/" + mb;
        _tree->access<double>(root / "tick_rate").update();

        //and now that the tick rate is set, init the host rates to something
        BOOST_FOREACH(const std::string &name, _tree->list(root / "rx_dsps")){
            // XXX: @CF: 20180301: on the server, we currently turn rx power (briefly) on any time that rx properties are set.
            // if the current application does not require rx, then we should not enable it
            // just checking for power is not a great way to do this, but it mostly works
            if ( "1" == _tree->access<std::string>( root / "rx" / name / "pwr").get() ) {
                _tree->access<double>(root / "rx_dsps" / name / "rate" / "value").update();
            }
        }
        BOOST_FOREACH(const std::string &name, _tree->list(root / "tx_dsps")){
            // XXX: @CF: 20180301: on the server, we currently turn tx power on any time that tx properties are set.
            // if the current application does not require tx, then we should not enable it
            // just checking for power is not a great way to do this, but it mostly works
            if ( "1" == _tree->access<std::string>( root / "tx" / name / "pwr").get() ) {
                _tree->access<double>(root / "tx_dsps" / name / "rate" / "value").update();
            }
        }
    }
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool crimson_tng_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    boost::this_thread::disable_interruption di; //disable because the wait can throw
    return _io_impl->async_msg_fifo.pop_with_timed_wait(async_metadata, timeout);
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

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = _mbc[_mbc.keys().front()].rx_dsp_xports[0]->get_recv_frame_size() - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = args.args.cast<size_t>("spp", bpp/bpi);

    //make the new streamer given the samples per packet
    boost::shared_ptr<crimson_tng_recv_packet_streamer> my_streamer = boost::make_shared<crimson_tng_recv_packet_streamer>(spp);

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

    if ( false ) {
    } else if ( "fc32" == args.cpu_format ) {
        my_streamer->set_scale_factor( 1.0 / (double)((1<<15)-1) );
    } else if ( "sc16" == args.cpu_format ) {
        my_streamer->set_scale_factor( 1.0 );
    }

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        BOOST_FOREACH(const std::string &mb, _mbc.keys()){
            num_chan_so_far += _mbc[mb].rx_chan_occ;
            if (chan < num_chan_so_far){
                const size_t dsp = chan + _mbc[mb].rx_chan_occ - num_chan_so_far;
                std::string scmd_pre( "rx_" + std::string( 1, 'a' + chan ) + "/stream" );
                stream_cmd_t scmd( stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS );
                scmd.stream_now = true;
                set_stream_cmd( scmd_pre, scmd );
                my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
                    &zero_copy_if::get_recv_buff, _mbc[mb].rx_dsp_xports[dsp], _1
                ), true /*flush*/);
                my_streamer->set_issue_stream_cmd(chan_i, boost::bind(
                    &crimson_tng_impl::set_stream_cmd, this, scmd_pre, _1));
                _mbc[mb].rx_streamers[chan] = my_streamer; //store weak pointer
                break;
            }
        }
    }

    //set the packet threshold to be an entire socket buffer's worth
    const size_t packets_per_sock_buff = size_t(50e6/_mbc[_mbc.keys().front()].rx_dsp_xports[0]->get_recv_frame_size());
    my_streamer->set_alignment_failure_threshold(packets_per_sock_buff);

    // XXX: @CF: 20170227: extra setup for crimson
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        size_t chan = args.channels[ chan_i ];
        const std::string ch    = "Channel_" + std::string( 1, 'A' + chan );
        const fs_path mb_path   = "/mboards/0";
        const fs_path rx_path   = mb_path / "rx";
        const fs_path rx_link_path  = mb_path / "rx_link" / ch;

		// power on the channel
		_tree->access<std::string>(rx_path / ch / "pwr").set("1");
		// XXX: @CF: 20180214: Do we _really_ need to sleep 1/2s for power on for each channel??
		//usleep( 500000 );
		// vita enable
		_tree->access<std::string>(rx_link_path / "vita_en").set("1");
		// stream enable
		_tree->access<std::string>(rx_link_path / "stream").set("1");
    }

    //sets all tick and samp rates on this streamer
    this->update_rates();

    // XXX: @CF: 20180117: Give any transient errors in the time-convergence PID loop sufficient time to subsidte. KB 4312
	for( ;! time_diff_converged(); ) {
		usleep( 10000 );
	}

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
static void get_fifo_lvl_udp( const size_t channel, uhd::transport::udp_simple::sptr xport, double & pcnt, uint64_t & uflow, uint64_t & oflow, uhd::time_spec_t & now ) {

	static constexpr double tick_period_ps = 2.0 / CRIMSON_TNG_MASTER_CLOCK_RATE;

	#pragma pack(push,1)
	struct fifo_lvl_req {
		uint64_t header; // 000000010001CCCC (C := channel bits, x := WZ,RAZ)
	};
	#pragma pack(pop)

	#pragma pack(push,1)
	struct fifo_lvl_rsp {
		uint64_t header; // CCCC00000000FFFF (C := channel bits, F := fifo bits)
		uint64_t oflow;
		uint64_t uflow;
		uint64_t tv_sec;
		uint64_t tv_tick;
	};
	#pragma pack(pop)

	fifo_lvl_req req;
	fifo_lvl_rsp rsp;

	req.header = (uint64_t)0x10001 << 16;
	req.header |= (channel & 0xffff);

	boost::endian::big_to_native_inplace( req.header );

	size_t r;

	for( size_t tries = 0; tries < 10; tries++ ) {
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
			continue;
		}

		break;
	}
	if ( 0 == r ) {
		throw new io_error( "Failed to retrieve buffer level for channel " + std::string( 1, 'A' + channel ) );
	}

	boost::endian::big_to_native_inplace( rsp.oflow );
	boost::endian::big_to_native_inplace( rsp.uflow );
	boost::endian::big_to_native_inplace( rsp.tv_sec );
	boost::endian::big_to_native_inplace( rsp.tv_tick );

	uint16_t lvl = rsp.header & 0xffff;
	pcnt = (double)lvl / CRIMSON_TNG_BUFF_SIZE;

	uflow = rsp.uflow & uint64_t( 0x7fffffffffffffff );
	oflow = rsp.oflow & uint64_t( 0x7fffffffffffffff );

	now = uhd::time_spec_t( rsp.tv_sec, rsp.tv_tick * tick_period_ps );

#if 0
	std::cout
			<< now << ": "
			<< (char)('A' + channel) << ": "
			<< '%' << std::dec << std::setw( 2 ) << std::setfill( ' ' ) << (unsigned)( pcnt * 100 )  << " "
			<< std::hex << std::setw( 4 ) << std::setfill( '0' ) << lvl << " "
			<< std::hex << std::setw( 16 ) << std::setfill( '0' ) << uflow << " "
			<< std::hex << std::setw( 16 ) << std::setfill( '0' ) << oflow << " "
			<< std::endl << std::flush;
#endif
}

static void pwr_off( uhd::property_tree::sptr tree, std::string path ) {
	//std::cout << __func__ << "(): Writing 0 to " << path << std::endl;
	tree->access<std::string>( path ).set( "0" );
}

tx_streamer::sptr crimson_tng_impl::get_tx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    if (args.otw_format != "sc16"){
        throw uhd::value_error("Crimson TNG TX cannot handle requested wire format: " + args.otw_format);
    }

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt_send_header_offset_words32*sizeof(uint32_t)
        + vrt::max_if_hdr_words32*sizeof(uint32_t)
        - sizeof(vrt::if_packet_info_t().tlr) //crimson tng does not use trailer on tx
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().sid) //no stream id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = _mbc[_mbc.keys().front()].tx_dsp_xports[0]->get_send_frame_size() - hdr_size;
    const size_t spp = bpp/convert::get_bytes_per_item(args.otw_format);

    //make the new streamer given the samples per packet
    crimson_tng_send_packet_streamer::timenow_type timenow_ = boost::bind( & crimson_tng_impl::get_time_now, this );
    std::vector<uhd::transport::zero_copy_if::sptr> xports;
    for( auto & i: args.channels ) {
    	xports.push_back( _mbc[ _mbc.keys().front() ].tx_dsp_xports[ i ] );
    }
    boost::shared_ptr<crimson_tng_send_packet_streamer> my_streamer = boost::make_shared<crimson_tng_send_packet_streamer>( spp );

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_packer(&vrt::if_hdr_pack_be, vrt_send_header_offset_words32);

    my_streamer->set_time_now(boost::bind(&crimson_tng_impl::get_time_now,this));

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = 1;
    id.output_format = args.otw_format + "_item32_be";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    if ( false ) {
    } else if ( "fc32" == args.cpu_format ) {
        my_streamer->set_scale_factor( (double)((1<<15)-1) );
    } else if ( "sc16" == args.cpu_format ) {
        my_streamer->set_scale_factor( 1.0 );
    }

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        BOOST_FOREACH(const std::string &mb, _mbc.keys()){
            num_chan_so_far += _mbc[mb].tx_chan_occ;
            if (chan < num_chan_so_far){
                const size_t dsp = chan + _mbc[mb].tx_chan_occ - num_chan_so_far;
                my_streamer->set_on_fini(chan_i, boost::bind( & pwr_off, _tree, std::string( "/mboards/" + mb + "/tx/Channel_" + std::string( 1, 'A' + chan ) + "/pwr" ) ) );
                my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
                    &crimson_tng_send_packet_streamer::get_send_buff, my_streamer, chan_i, spp, _1
                ));
                my_streamer->set_xport_chan(chan_i,_mbc[mb].tx_dsp_xports[dsp]);
                my_streamer->set_xport_chan_fifo_lvl(chan_i, boost::bind(
                    &get_fifo_lvl_udp, chan_i, _mbc[mb].fifo_ctrl_xports[dsp], _1, _2, _3, _4
                ));
                my_streamer->set_async_receiver(boost::bind(&bounded_buffer<async_metadata_t>::pop_with_timed_wait, &(_io_impl->async_msg_fifo), _1, _2));
                my_streamer->set_async_pusher(boost::bind(&bounded_buffer<async_metadata_t>::push_with_pop_on_full, &(_io_impl->async_msg_fifo), _1));
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
        const fs_path tx_link_path  = mb_path / "tx_link" / ch;

		// power on the channel
		_tree->access<std::string>(tx_path / ch / "pwr").set("1");
		// XXX: @CF: 20180214: Do we _really_ need to sleep 1/2s for power on for each channel??
		//usleep( 500000 );
		// vita enable
		_tree->access<std::string>(tx_link_path / "vita_en").set("1");
    }

    //sets all tick and samp rates on this streamer
    this->update_rates();

    // XXX: @CF: 20180117: Give any transient errors in the time-convergence PID loop sufficient time to subsidte. KB 4312
	for( ;! time_diff_converged(); ) {
		usleep( 10000 );
	}
    my_streamer->pillage();

    return my_streamer;
}
