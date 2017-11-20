#include <bitset>
#include <cstddef>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/uio.h>

#include <boost/core/ignore_unused.hpp>
#include <boost/endian/buffers.hpp>

#include "uhd/utils/msg.hpp"

#include "crimson_tng_impl.hpp"
#include "crimson_tng_tx_streamer.hpp"
#include "crimson_tng_fw_common.h"

#include "iputils.hpp"

#ifdef DEBUG_TX
#include <iostream>
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE( x ) ((int)( sizeof( x ) / sizeof( (x)[ 0 ] ) ))
#endif

using namespace uhd;
using namespace uhd::transport;

// number of channels for streamer
size_t crimson_tng_tx_streamer::get_num_channels() const {
	return _channels.size();
}

// max samples per buffer per packet for sfpa, (4-bytes: 16 I, 16Q)
size_t crimson_tng_tx_streamer::get_max_num_samps(void) const {
	return _pay_len/4;
}

/**
 * It's been found that the existing uhd if_packet_info_t is lacking and
 * so we will use it primarily for a place to store variables rather than
 * for it's constructor and other methods until it is rewritten.
 *
 * Once it's been rewritten, the output buffer should be removed, as
 * presumably the VRT49 structure (currently if_packet_info_t) would
 * handle that.
 *
 * Lastly, if we ever switch to vector writes, we must also eliminate
 * copying to the output buffer.
 *
 * @param metadata            metadata for packet (input)
 * @param sample_data         sample data (input)
 * @param mtu_bytes           mtu for the channel, in bytes (input)
 * @param remaining_samples   samples remaining for the channel (input)
 * @param buf_len             output buffer length (input)
 * @param buf                 output buffer (output)
 * @param ifo                 VRT49 structure (output)
 */
void crimson_tng_tx_streamer::compose_vrt49_packet(
	const uhd::tx_metadata_t &metadata,
	const size_t mtu_bytes,
	const size_t remaining_samples,
	const size_t buf_len,
	const size_t sample_count,
	uint32_t *buf,
	uhd::transport::vrt::if_packet_info_t &ifo
) {

	const size_t N =
		std::min(
			( mtu_bytes
				- 60 // IPv4 Header
				- 8  // UDP Header
				- uhd::transport::vrt::max_if_hdr_words32 * sizeof( uint32_t )
			) / sizeof( uint32_t ),
			remaining_samples
		);

	size_t k;

	//translate the metadata to vrt if packet info
	ifo.link_type = vrt::if_packet_info_t::LINK_TYPE_NONE;
	ifo.packet_type = vrt::if_packet_info_t::PACKET_TYPE_DATA;
	ifo.has_sid = false; // currently unused
	ifo.has_cid = false; // currently unused
	ifo.has_tlr = false; // currently unused
	ifo.has_tsi = metadata.has_time_spec && metadata.start_of_burst;
	ifo.has_tsf = true;//metadata.has_time_spec;

	if ( metadata.has_time_spec && metadata.start_of_burst) {
		ifo.tsi_type = vrt::if_packet_info_t::TSI_TYPE_OTHER;
		ifo.tsi = (uint32_t) metadata.time_spec.get_full_secs();
		ifo.tsf_type = vrt::if_packet_info_t::TSF_TYPE_PICO;
		ifo.tsf = (uint64_t)( metadata.time_spec.get_frac_secs() * 1e12 );
	} else {
		ifo.tsi_type = vrt::if_packet_info_t::TSI_TYPE_NONE;
		ifo.tsf_type = vrt::if_packet_info_t::TSF_TYPE_FREE;
		ifo.tsf = (uint64_t)( sample_count );
	}

	// XXX: these flags denote the first and last packets in burst sample data
	// NOT indication that timed, (possibly) multi-channel burst will start
	ifo.sob = metadata.start_of_burst;
	ifo.eob	= metadata.end_of_burst;

	ifo.num_header_words32 = (metadata.has_time_spec && metadata.start_of_burst)? 4 : 3;
	ifo.num_payload_words32 = N;
	ifo.num_payload_bytes = N * sizeof( uint32_t );

	ifo.num_packet_words32 = ifo.num_header_words32 + ifo.num_payload_words32 + ( ifo.has_tlr ? 1 : 0 );

	if ( BOOST_UNLIKELY( buf_len < ifo.num_header_words32 ) ) {
		throw runtime_error(
			(
				boost::format( "buf_len ( %u ) is not large enough for header ( %u )" )
				% buf_len
				% ifo.num_header_words32
			).str()
		);
	}

	buf[ 0 ] = 0;
	buf[ 0 ] |= vrt::if_packet_info_t::PACKET_TYPE_DATA << 28;
	buf[ 0 ] |= 1 << 25; // set reserved bit (so wireshark works). this should eventually be removed
	buf[ 0 ] |= (uint16_t) ifo.num_packet_words32;

	buf[ 0 ] |= ifo.tsi_type << 22;
	buf[ 0 ] |= ifo.tsf_type << 20;

	k = 1;
	if ( metadata.has_time_spec && metadata.start_of_burst) {
		buf[ k++ ] = ifo.tsi;
	}
	buf[ k++ ] = (uint32_t)( ifo.tsf >> 32 );
	buf[ k++ ] = (uint32_t)( ifo.tsf >> 0  );

	for( size_t k = 0; k < ifo.num_header_words32; k++ ) {
		boost::endian::native_to_big_inplace( buf[ k ] );
	}
}

time_spec_t crimson_tng_tx_streamer::get_time_now() {
	uhd::usrp::crimson_tng_impl *dev = static_cast<uhd::usrp::crimson_tng_impl *>( _dev );
	uhd::time_spec_t r = uhd::time_spec_t::get_system_time();
	r += NULL == dev ? 0.0 : dev->time_diff_get();
	return r;
}

void crimson_tng_tx_streamer::set_device( uhd::device *dev ) {
	_dev = dev;
}

void crimson_tng_tx_streamer::on_buffer_level_read( const std::vector<size_t> & buffer_levels ) {
	for( size_t i = 0; i < _channels.size(); i++ ) {
		int ch = _channels[ i ];
		_flow_control[ i ]->set_buffer_level_async( buffer_levels[ ch ] );
	}
}

void crimson_tng_tx_streamer::push_async_msg( uhd::async_metadata_t &async_metadata ) {
	std::lock_guard<std::mutex> _lock( _async_mutex );
	_async_msg_fifo.push_with_pop_on_full( async_metadata );
}

size_t crimson_tng_tx_streamer::send(
		const buffs_type &buffs,
		const size_t nsamps_per_buff,
		const tx_metadata_t &_metadata,
		const double timeout = 0.1)
{

	size_t samp_sent = 0;
	size_t remaining_bytes[ _channels.size() ];
	vrt::if_packet_info_t if_packet_info;

	iovec iov[ 2 ];

	// need r/w capabilities for 'has_time_spec'
	std::vector<tx_metadata_t> metadata;

	uhd::time_spec_t now, then, dt;

	uhd::time_spec_t send_deadline;

	uoflow_enable_reporting();

	if (
		true
		&& false == _metadata.start_of_burst
		&& true == _metadata.end_of_burst
		&& 0 == nsamps_per_buff
	) {
		// empty end-of-burst packet signals tx_streamer to stop
		for ( size_t i = 0; i < _channels.size(); i++ ) {
				_sample_count[ i ] = 0;
		}
		fini_tx_streamer();
		return 0;
	}

	tx_metadata_t md = _metadata;
	if ( _first_send && _sob_arg > 0.0 ) {
		md.has_time_spec = true;
		md.start_of_burst = true;
		md.time_spec = get_time_now() + _sob_arg;
		//std::cout << "SOB, sending " <<  nsamps_per_buff << " samples in " << _sob_arg << " s" << std::endl;
	}
	// XXX: @CF: workaround for current overflow issue
	// found that when SoB was zero, buffer level did not get up to set point. 
	// suggested a minimal SoB to pre-fill the buffer.
	if ( _first_send && ! md.has_time_spec ) {
		md.has_time_spec = true;
		md.start_of_burst = true;
		md.time_spec = get_time_now() + 2.0;
		//std::cout << "NO SOB, sending " <<  nsamps_per_buff << " samples in " << 2 << " s" << std::endl;
	}
	//if ( _first_send ) 		std::cout << "Time now " <<  get_time_now().get_real_secs()  << " Send at " << md.time_spec.get_real_secs() << std::endl;


	for ( size_t i = 0; i < _channels.size(); i++ ) {
		remaining_bytes[ i ] = nsamps_per_buff * sizeof( uint32_t );
		metadata.push_back( md );
		if ( md.has_time_spec && md.start_of_burst) {
			_flow_control[ i ]->set_start_of_burst_time( md.time_spec );
			_sample_count[ i ] = 0;
		}
	}

	now = get_time_now();

	send_deadline = now;
	send_deadline += timeout;
	send_deadline += ( md.has_time_spec ? md.time_spec : 0.0 );

	for(
		;
		true
		&& get_time_now() < send_deadline
		&& ((samp_sent < nsamps_per_buff * _channels.size())|| md.start_of_burst)
		;
	) {
		for ( size_t i = 0; i < _channels.size(); i++ ) {

			//
			// Compose VRT49 Packet
			//

			size_t sample_byte_offs = nsamps_per_buff * sizeof( uint32_t ) - remaining_bytes[ i ];
			compose_vrt49_packet(
				metadata[ i ],
				_if_mtu[ i ],
				remaining_bytes[ i ] / sizeof( uint32_t ),
				CRIMSON_TNG_MAX_MTU / sizeof( uint32_t ),
				_sample_count[ i ],
				_vita_hdr_buf,
				if_packet_info
			);

			//
			// Flow Control
			//

			now = get_time_now();
			dt =
				_flow_control[ i ]->get_time_until_next_send(
					if_packet_info.num_payload_words32,
					now
				);
			then = now + dt;
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
			}

			//
			// Ensure that we have primed the buffers if SoB was given
			//
/*
			if (
				true
				&& 0.0 != _sob_time // set to 0.0 if we nanosleep
				&& now >= _sob_time
				&& _flow_control[ i ]->get_buffer_level( now ) < _flow_control[ i ]->get_nominal_buffer_level()
			) {
				throw runtime_error(
					(
						boost::format( "Premature Start-of-Burst detected on channel %c ( expected: %u, actual: %u )" )
						% (char)( 'A' + _channels[ i ] )
						% _flow_control[ i ]->get_nominal_buffer_level()
						% _flow_control[ i ]->get_buffer_level( now )
					).str()
				);
			}
*/

			//
			// Send Data
			//

			iov[ 0 ].iov_base = _vita_hdr_buf;
			iov[ 0 ].iov_len = if_packet_info.num_header_words32 * sizeof( uint32_t );
			iov[ 1 ].iov_base = & ( (uint8_t *)buffs[ i ] )[ sample_byte_offs ];
			iov[ 1 ].iov_len = if_packet_info.num_payload_bytes;

			ssize_t r = writev( _udp_socket[ i ], iov, ARRAY_SIZE( iov ) );
			if ( -1 == r ) {
				throw runtime_error(
					(
						boost::format( "writev: %s (%d)" )
						% std::strerror( errno )
						% errno
					).str()
				);
			}
			if ( r != (ssize_t) ( if_packet_info.num_packet_words32 * sizeof( uint32_t ) ) ) {
				throw runtime_error(
					(
						boost::format( "Failed to send a packet ( expected: %u, actual: %u )" )
						% ( if_packet_info.num_packet_words32 * sizeof( uint32_t ) )
						% r
					).str()
				);
			}

			//
			// Update Flow Control
			//

			_flow_control[ i ]->update( if_packet_info.num_payload_words32, now );

			//
			// Decrement Byte / Sample Counters
			//

			_sample_count[ i ] += if_packet_info.num_payload_words32;
			remaining_bytes[ i ] -= if_packet_info.num_payload_bytes;
			samp_sent += if_packet_info.num_payload_words32;

			// this ensures we only send the vita time spec on the first packet of the burst
			metadata[ i ].has_time_spec = false;
			metadata[ i ].start_of_burst = false;
		}
		md.start_of_burst = false;
	}

	_first_send = false;
	return samp_sent / _channels.size();
}

// async messages are currently disabled
bool crimson_tng_tx_streamer::recv_async_msg(
	uhd::async_metadata_t &async_metadata,
	double timeout
) {
	std::lock_guard<std::mutex> _lock( _async_mutex );
	return _async_msg_fifo.pop_with_timed_wait( async_metadata, timeout );
}

void crimson_tng_tx_streamer::init_tx_streamer(
	device_addr_t addr,
	property_tree::sptr tree,
	std::vector<size_t> channels
) {

	// save the tree
	_tree = tree;
	_channels = channels;

	// get the property root path
	const fs_path mb_path   = "/mboards/0";
	const fs_path time_path = mb_path / "time";
	const fs_path prop_path = mb_path / "tx_link";

	// if no channels specified, default to channel 1 (0)
	_channels = _channels.empty() ? std::vector<size_t>(1, 0) : _channels;
	_if_mtu = std::vector<size_t>( _channels.size() );
	_sample_count = std::vector<size_t>( _channels.size() );

	if ( addr.has_key( "crimson:sob" )  ) {
		if ( ! sscanf( addr[ "crimson:sob" ].c_str(), "%lf", & _sob_arg ) ) {
			UHD_MSG( warning )  << __func__ << "(): Unrecognized argument crimson:sob=" << addr[ "crimson:sob" ] << std::endl;
		}
	}

	//Set up constants
	_max_clock_ppm_error = 100;

	for (unsigned int i = 0; i < _channels.size(); i++) {

		std::string sfp;
		switch( _channels[ i ] ) {
		case 0:
		case 2:
			sfp = "sfpa";
			break;
		case 1:
		case 3:
			sfp = "sfpb";
			break;
		}

		std::string ch       = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
		std::string udp_port = tree->access<std::string>(prop_path / "Channel_"+ch / "port").get();

		std::stringstream udp_port_ss( udp_port );

		uint16_t port;
		udp_port_ss >> port;

		std::string iface    = tree->access<std::string>(prop_path / "Channel_"+ch / "iface").get();
		std::string ip_addr  = tree->access<std::string>( mb_path / "link" / sfp / "ip_addr").get();

		_pay_len = tree->access<int>(mb_path / "link" / sfp / "pay_len").get();

		_if_mtu[ i ] = iputils::get_if_mtu( ip_addr );

		// power on the channel
		tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("1");
		usleep( 500000 );

		// vita enable (as of kb #3804, always use vita headers for tx)
		tree->access<std::string>(prop_path / "Channel_"+ch / "vita_en").set("1");

		// connect UDP socket

		std::string local_addrs;
		struct sockaddr_storage remote_addr, local_addr;
		socklen_t remote_addr_len = sizeof( remote_addr ), local_addr_len = sizeof( local_addr );
		int fd;

		iputils::get_route( iputils::get_route_info( ip_addr ), iface, local_addrs );
		iputils::to_sockaddr( ip_addr, (sockaddr *) & remote_addr, remote_addr_len );
		((sockaddr_in *)& remote_addr )->sin_port = htons( port );
		iputils::to_sockaddr( local_addrs, (sockaddr *) & local_addr, local_addr_len );
		fd = iputils::connect_udp( (sockaddr *) & local_addr, local_addr_len, (sockaddr *) & remote_addr, remote_addr_len );
		_udp_socket.push_back( fd );

		const double nominal_sample_rate = _tree->access<double>( "/mboards/0/tx_dsps/Channel_" + ch + "/rate/value" ).get();
		const double nominal_buffer_level_pcnt = 0.8;
		_flow_control.push_back(
			uhd::flow_control_nonlinear::make(
				nominal_sample_rate,
				nominal_buffer_level_pcnt,
				(size_t)CRIMSON_TNG_BUFF_SIZE
			)
		);
	}
}

void crimson_tng_tx_streamer::fini_tx_streamer() {

	uhd::usrp::crimson_tng_impl *dev = static_cast<uhd::usrp::crimson_tng_impl *>( _dev );

	if ( NULL != dev ) {
		dev->bm_listener_rem( this );
	}

	const fs_path mb_path   = "/mboards/0";
	const fs_path prop_path = mb_path / "tx_link";

	_tree->access<int>( mb_path / "cm" / "chanmask-tx" ).set( 0 );

	for (unsigned int i = 0; i < _channels.size(); i++) {
		std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
		// power off the channel
		_tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("0");
		usleep(4000);
	}

	_dev = NULL;
}

void crimson_tng_tx_streamer::on_uoflow_read( const uhd::usrp::time_diff_resp & tdr ) {

	async_metadata_t metadata;

	for ( int j = 0; j < CRIMSON_TNG_TX_CHANNELS; j++ ) {

		// update uflow counters, notify user on change

		if ( _uoflow_report_en && _uflow[ j ] != tdr.uoflow[ j ].uflow ) {

			// XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
            // async_metadata_t metadata;
            // load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
			metadata.channel = j;
			metadata.has_time_spec = true;
			metadata.time_spec = uhd::time_spec_t::get_system_time();
			metadata.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
			push_async_msg( metadata );
			//UHD_MSG( fastpath ) << "U" << ((char) ( 'a' + j ) );
		}
		_uflow[ j ] = tdr.uoflow[ j ].uflow;

		// update oflow counters, notify user on change

		if ( _uoflow_report_en && _oflow[ j ] != tdr.uoflow[ j ].oflow ) {
			// XXX: @CF: 20170905: Eventually we want to return tx channel metadata as VRT49 context packets rather than custom packets. See usrp2/io_impl.cpp
            // async_metadata_t metadata;
            // load_metadata_from_buff( uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index );
			metadata.channel = j;
			metadata.has_time_spec = true;
			metadata.time_spec = uhd::time_spec_t::get_system_time();
			metadata.event_code = uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR;
			push_async_msg( metadata );
			//UHD_MSG( fastpath ) << "O" << ((char) ( 'a' + j ) );
		}
		_oflow[ j ] = tdr.uoflow[ j ].oflow;

	}
}

void crimson_tng_tx_streamer::uoflow_enable_reporting( bool en ) {
	_uoflow_report_en = en;
}
