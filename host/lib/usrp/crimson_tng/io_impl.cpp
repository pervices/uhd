//
// Copyright 2010-2012 Ettus Research LLC
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
#include <iostream>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "uhd/stream.hpp"
#include "uhd/transport/udp_stream_zero_copy.hpp"

#include "../../transport/super_recv_packet_handler.hpp"
#include "../../transport/super_send_packet_handler.hpp"

#include "crimson_tng_impl.hpp"

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

static void get_tx_endpoint( uhd::property_tree::sptr tree, const size_t & chan, std::string & ip_addr, uint16_t & udp_port );

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr crimson_tng_impl::get_rx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;
    _rx_channels = args.channels;

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = 9000 - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

    const fs_path mb_path   = "/mboards/0";
	const fs_path link_path = mb_path / "rx_link";

    zero_copy_xport_params zcxp;
    udp_zero_copy::buff_params bp;

#ifndef DEFAULT_NUM_FRAMES
#define DEFAULT_NUM_FRAMES 32
#endif

    zcxp.send_frame_size = 0;
    zcxp.recv_frame_size = bpp;
    zcxp.num_send_frames = 0;
    zcxp.num_recv_frames = DEFAULT_NUM_FRAMES;

    set_properties_from_addr();

    _rx_if.resize( _rx_channels.size() );
    for( size_t i = 0; i < _rx_channels.size(); i++ ) {
		// get the channel parameters
		std::string ch       = std::string( 1, _rx_channels[ i ] + 'A' );
		std::string udp_port = _tree->access<std::string>(link_path / "Channel_"+ch / "port").get();
		std::string ip_addr  = _tree->access<std::string>(link_path / "Channel_"+ch / "ip_dest").get();
		std::string iface    = _tree->access<std::string>(link_path / "Channel_"+ch / "iface").get();

		// power on the channel
		_tree->access<std::string>(mb_path / "rx" / "Channel_"+ch / "pwr").set("1");
		// XXX: @CF: 20180214: Do we _really_ need to sleep 1/2s for power on for each channel??
		//usleep( 500000 );

		// vita enable
		_tree->access<std::string>(link_path / "Channel_"+ch / "vita_en").set("1");
		// stream enable
		_tree->access<std::string>(link_path / "Channel_"+ch / "stream").set("1");

		_rx_if[ i ] = udp_stream_zero_copy::make( ip_addr, udp_port, "127.0.0.1", "1", zcxp, bp, _addr );
    }

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp);

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

    //bind callbacks for the handler
    _rx_streamers.resize( _rx_channels.size() );
    for ( size_t i = 0; i < _rx_channels.size(); i++ ) {
        my_streamer->set_xport_chan_get_buff(
			i,
			boost::bind(
				& zero_copy_if::get_recv_buff,
				// for non-static methods, the first argument is a reference to the instance.
				// the default behaviour is to copy arguments, and in this case, that's ok, because it's
				// a shared_ptr.
				_rx_if[ i ],
				_1
			),
			true /*flush*/
		);
		my_streamer->set_issue_stream_cmd(
			i,
			boost::bind(
				& crimson_tng_impl::set_stream_cmd,
				// for non-static methods, the first argument is a reference to the instance.
				// the default behaviour is to copy arguments, and in this case, that's ok, because it's
				// a shared_ptr.
				this,
				"rx_" + std::string( 1, (char)( 'a' + _rx_channels[ i ] ) ) + "/stream",
				_1
			)
		);
		_rx_streamers[ i ] = my_streamer; //store weak pointer
    }

    //set the packet threshold to be an entire socket buffer's worth
//    const size_t packets_per_sock_buff = size_t(50e6/_mbc[_mbc.keys().front()].rx_dsp_xports[0]->get_recv_frame_size());
//    my_streamer->set_alignment_failure_threshold( packets_per_sock_buff );

    //sets all tick and samp rates on this streamer
    for( auto & i: _rx_channels ) {

    	std::string path = (
			boost::format( "/mboards/0/rx_dsps/Channel_%c/rate/value" )
    		% ( (char)('A' + i) )
    	).str();

    	_tree->access<double>( path ).update();
    }

    // XXX: @CF: 20180117: Give any transient errors in the time-convergence PID loop sufficient time to subsidte. KB 4312
    for( ;! time_diff_converged(); ) {
		usleep( 10000 );
    }

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/

class crimson_tng_send_packet_streamer : public sph::send_packet_handler, public tx_streamer {
public:

	typedef boost::function<void(const size_t,const uhd::time_spec_t&)> update_flow_control_type;

	crimson_tng_send_packet_streamer( const size_t max_num_samps, crimson_tng_impl *impl )
	:
		_impl( impl ),
		_max_num_samps( max_num_samps )
	{
		set_max_samples_per_packet( max_num_samps );
	}

	size_t send(
		const tx_streamer::buffs_type &buffs,
		const size_t nsamps_per_buff,
		const uhd::tx_metadata_t &metadata,
		const double timeout
	){
		size_t num_samps_sent = sph::send_packet_handler::send(
			buffs, nsamps_per_buff, metadata, timeout
		);
		for( size_t i = 0; i < buffs.size(); i++ ) {
			_update_flow_control_func[ i ]( num_samps_sent, _impl->get_time_now() );
		}
		return num_samps_sent;
	}

    size_t get_num_channels() const{
        return this->size();
    }

    size_t get_max_num_samps() const{
        return _max_num_samps;
    }

    bool recv_async_msg( uhd::async_metadata_t &async_metadata, double timeout = 0.1 ) {
    	return _impl->recv_async_msg( async_metadata, timeout );
    }

	crimson_tng_impl::sptr _impl;
    size_t _max_num_samps;
	std::vector<update_flow_control_type> _update_flow_control_func;
};

tx_streamer::sptr crimson_tng_impl::get_tx_stream(const uhd::stream_args_t &args_){
	stream_args_t args = args_;

	//setup defaults for unspecified values
	args.otw_format = "sc16";
	args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;
	_tx_channels = args.channels;

	const fs_path mb_path   = "/mboards/0";
	const fs_path link_path = mb_path / "tx_link";

	set_properties_from_addr();

	std::string ip_addr;
	uint16_t udp_port;
	uint16_t if_mtu;

	//calculate packet size
	static const size_t hdr_size = 0
		+ vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
		- sizeof(vrt::if_packet_info_t().tlr) //crimson tng does not use trailer on tx
		- sizeof(vrt::if_packet_info_t().cid) //no class id ever used
		- sizeof(vrt::if_packet_info_t().sid) //no stream id ever used
		// crimson tng uses tsi. For SoB, set to TSF_TYPE_OTHER and populate seconds, otherwise, set to TSF_TYPE_UTC and ignore.
		// crimson_tng uses tsf. If it's an SoB, we send TSF_TYPE_PICO, otherwise TSF_TYPE_FREE.
		;

	const size_t ip_size = 60; // IPv4 Header
	const size_t udp_size = 8;  // UDP Header
	const size_t bpp = 9000 - ip_size - udp_size - hdr_size;
	const size_t spp = bpp/convert::get_bytes_per_item(args.otw_format);

	zero_copy_xport_params zcxp;
	udp_zero_copy::buff_params bp;

	#ifndef DEFAULT_NUM_FRAMES
	#define DEFAULT_NUM_FRAMES 32
	#endif

	zcxp.send_frame_size = bpp;
	zcxp.recv_frame_size = 0;
	zcxp.num_send_frames = DEFAULT_NUM_FRAMES;
	zcxp.num_recv_frames = 0;

	_tx_if.resize( _tx_channels.size() );
	for( size_t i = 0; i < _tx_channels.size(); i++ ) {

		get_tx_endpoint( _tree, _tx_channels[ i ], ip_addr, udp_port );

		// get the channel parameters
		std::string ch = std::string( 1, _tx_channels[ i ] + 'A' );

		// power on the channel
		_tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("1");
		//usleep( 500000 );

		// vita enable
		_tree->access<std::string>(link_path / "Channel_"+ch / "vita_en").set("1");

		_tx_if[ i ] = udp_zero_copy::make( ip_addr, std::to_string( udp_port ), zcxp, bp, _addr );
	}

	//make the new streamer given the samples per packet
	boost::shared_ptr<crimson_tng_send_packet_streamer> my_streamer = boost::make_shared<crimson_tng_send_packet_streamer>(spp, this);

	//init some streamer stuff
	my_streamer->resize( _tx_channels.size() );
	my_streamer->set_vrt_packer( & vrt::if_hdr_pack_be );
	my_streamer->set_enable_trailer( false );

	//set the converter
	uhd::convert::id_type id;
	id.input_format = args.cpu_format;
	id.num_inputs = 1;
	id.output_format = args.otw_format + "_item32_be";
	id.num_outputs = 1;
	my_streamer->set_converter( id );

	_tx_streamers.resize( _tx_channels.size() );
	_flow_control.resize( _tx_channels.size() );
	//bind callbacks for the handler
	for ( size_t i = 0; i < _tx_channels.size(); i++ ) {

		const size_t chan = _tx_channels[ i ];

		// reset flow control
		//             if (not args.args.has_key("noclear")){
		//                     _io_impl->fc_mons[abs]->clear();
		//             }

		// setup dsp
		//_mbc[mb].tx_dsp->setup( args );


		my_streamer->set_xport_chan_get_buff(
			i,
			boost::bind(
				& crimson_tng_impl::get_send_buff,
				this,
				chan,
				_1
			)
		);

		my_streamer->set_async_receiver(
			boost::bind(
				&bounded_buffer<async_metadata_t>::pop_with_timed_wait,
				& _async_msg_fifo,
				_1,
				_2
			)
		);
		_tx_streamers[ i ] = my_streamer; //store weak pointer

		const double nominal_sample_rate = _tree->access<double>( "/mboards/0/tx_dsps/Channel_" + std::string(1, 'A' + i ) + "/rate/value" ).get();
		const double nominal_buffer_level_pcnt = 0.8;
		_flow_control[ i ] =
			uhd::flow_control_nonlinear::make(
				nominal_sample_rate,
				nominal_buffer_level_pcnt,
				(size_t)CRIMSON_TNG_BUFF_SIZE
			);

		crimson_tng_send_packet_streamer::update_flow_control_type flow_control =
			boost::bind(
				& flow_control::update,
				_flow_control[ i ],
				_1,
				_2
			);

		my_streamer->_update_flow_control_func.push_back( flow_control );
	}

	//sets all tick and samp rates on this streamer
	for( auto & i: _tx_channels ) {

		std::string path = (
			boost::format( "/mboards/0/tx_dsps/Channel_%c/rate/value" )
			% ( (char)('A' + i) )
		).str();

		_tree->access<double>( path ).update();
	}

	// XXX: @CF: 20180117: Give any transient errors in the time-convergence PID loop sufficient time to subsidte. KB 4312
	for( ;! time_diff_converged(); ) {
		usleep( 10000 );
	}

	return my_streamer;
}

static void get_tx_endpoint( uhd::property_tree::sptr tree, const size_t & chan, std::string & ip_addr, uint16_t & udp_port ) {

       // get the property root path
       const fs_path mb_path   = "/mboards/0";
       const fs_path time_path = mb_path / "time";
       const fs_path prop_path = mb_path / "tx_link";

       std::string sfp;
       switch( chan ) {
       case 0:
       case 2:
               sfp = "sfpa";
               break;
       case 1:
       case 3:
               sfp = "sfpb";
               break;
       }

       std::string ch       = boost::lexical_cast<std::string>((char)( chan + 'A' ));
       std::string udp_port_str = tree->access<std::string>(prop_path / "Channel_"+ch / "port").get();

       std::stringstream udp_port_ss( udp_port_str );

       udp_port_ss >> udp_port;

       std::string iface    = tree->access<std::string>(prop_path / "Channel_"+ch / "iface").get();
       ip_addr  = tree->access<std::string>( mb_path / "link" / sfp / "ip_addr").get();

       //if_mtu = iputils::get_if_mtu( ip_addr );
}

