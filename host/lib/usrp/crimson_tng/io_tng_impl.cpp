//
// Copyright 2014-2015 Per Vices
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

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
#include <thread>
#include <vector>
#include <mutex>

#include <boost/range/numeric.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>
#include <boost/endian/buffers.hpp>

#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/diff.hpp>
#include "flow_control.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/pidc_tl.hpp>
#include <uhd/utils/sma.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <uhd/transport/udp_stream.hpp>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/types/wb_iface.hpp>

#include "crimson_tng_impl.hpp"
#include "crimson_tng_fw_common.h"

#include "iputils.hpp"

#ifndef DEBUG_RX
//#define DEBUG_RX 1
#endif
#ifndef DEBUG_TX
#define DEBUG_TX 1
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE( x ) ((int)( sizeof( x ) / sizeof( (x)[ 0 ] ) ))
#endif

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
using namespace uhd::transport::vrt;

namespace bo = boost::endian;
namespace asio = boost::asio;
namespace pt = boost::posix_time;
namespace trans = uhd::transport;

// kb 4001: stop-gap solution until kb #4000 is fixed!!!
static void destroy_other_processes_using_crimson();

static int channels_to_mask( std::vector<size_t> channels ) {
	unsigned mask = 0;

	for( size_t x: channels ) {
		if ( x <= 8 * sizeof(mask) - 1 ) {
			mask |= 1 << x;
		}
	}

	return mask;
}

static uint32_t get_if_mtu( const std::string & remote_addr ) {

	std::string iface;

	iputils::get_route( remote_addr, iface );
	size_t mtu = iputils::get_mtu( iface );

	if ( mtu < CRIMSON_TNG_MIN_MTU ) {
		throw runtime_error(
			(
				boost::format( "mtu %u on iface %s is below minimum recommended size of %u. Use 'sudo ifconfig %s mtu %u' to correct." )
					% mtu
					% iface
					% CRIMSON_TNG_MIN_MTU
					% iface
					% CRIMSON_TNG_MIN_MTU
			).str()
		);
	}

	return mtu;
}

class crimson_tng_rx_streamer : public uhd::rx_streamer {
public:
	crimson_tng_rx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels) {
		init_rx_streamer(addr, tree, channels);
	}

	crimson_tng_rx_streamer(device_addr_t addr, property_tree::sptr tree) {
		init_rx_streamer(addr, tree, std::vector<size_t>(1, 0));
	}

	~crimson_tng_rx_streamer() {
		fini_rx_streamer();
	}

	// number of channels for streamer
	size_t get_num_channels(void) const {
		return _channels.size();
	}

	// max samples per buffer per packet for sfpa, (4-bytes: 16 I, 16Q)
	size_t get_max_num_samps(void) const {
		return _pay_len/4;
	}

	void update_fifo_metadata( rx_metadata_t &meta, size_t n_samples ) {
		meta.time_spec += time_spec_t::from_ticks( n_samples, _rate );
	}

	size_t recv(
        	const buffs_type &buffs,
        	const size_t nsamps_per_buff,
        	rx_metadata_t &metadata,
        	const double timeout = 0.1,
        	const bool one_packet = true )
	{
		const size_t vita_hdr = 4;
		const size_t vita_tlr = 1;
		const size_t vita_pck = nsamps_per_buff + vita_hdr + vita_tlr;
		size_t nbytes = 0;
		size_t nsamples = 0;

		double _timeout = timeout;

#ifdef DEBUG_RX
		//UHD_MSG( status ) << __func__ << "( buffs: " << (void *) & buffs << ", nsamps_per_buff: " << nsamps_per_buff << ", metadata: " << (void *) & metadata << ", timeout: " << timeout << ", one_packet: " << one_packet << " )" << std::endl;

		// XXX: do not timeout when debugging
		//_timeout = 1e6;

		// XXX: use the following infinite loop to start a debugger session and attach to process at a known "entry point"
		//static bool _true = true;
		//do {
		//	std::cout << "";
		//} while( _true );
#endif

		// temp buffer: vita hdr + data
		uint32_t vita_buf[vita_pck];

		std::vector<size_t> fifo_level( _channels.size() );
		for( unsigned i = 0; i < _channels.size(); i++ ) {
			fifo_level[ i ] = _fifo[ i ].size();
		}

		if ( 0 != boost::accumulate( fifo_level, 0 ) ) {

			for( unsigned i = 0; i < _channels.size(); i++ ) {
				for( unsigned j = 0; j < nsamps_per_buff * 4 && ! _fifo[ i ].empty(); j++ ) {
					( (uint8_t *) buffs[ i ] )[ j ] = _fifo[ i ].front();
					_fifo[ i ].pop();
				}
				nbytes = fifo_level[ i ] - _fifo[ 0 ].size();
				nsamples = nbytes / 4;

#ifdef DEBUG_RX
				UHD_MSG( status ) << __func__ << "():" << __LINE__ << ": POP [ " << (char)( i + 'A' ) << " ]: nbytes: " << nbytes << ", nsamples: " << nsamples << std::endl;
#endif
			}

			metadata = _fifo_metadata;

			update_fifo_metadata( _fifo_metadata, nsamples );

			return nsamples;
		}

		// read from each connected stream and dump it into the buffs[i]
		for (unsigned int i = 0; i < _channels.size(); i++) {

			// clear temp buffer and output buffer
			memset(vita_buf, 0, vita_pck * 4);
			memset(buffs[i], 0, nsamps_per_buff * 4);

			// read in vita_pck*4 bytes to temp buffer
			nbytes = _udp_stream[i] -> stream_in(vita_buf, vita_pck * 4, _timeout);
			if (nbytes == 0) {
				metadata.error_code =rx_metadata_t::ERROR_CODE_TIMEOUT;
				return 0;
			}
			nbytes -= (vita_hdr + vita_tlr) * sizeof( uint32_t );
			nsamples = nbytes / sizeof( uint32_t );

#ifdef DEBUG_RX
			UHD_MSG( status ) << __func__ << "():" << __LINE__ << ": STREAM [ " << (char)( i + 'A' ) << " ]: nbytes: " << nbytes << ", nsamples: " << nsamples << std::endl;
#endif

			// copy non-vita packets to buffs[0]
			memcpy(buffs[i], vita_buf + vita_hdr , nbytes );
		}

		// process vita timestamps based on the last stream input's time stamp
		uint32_t vb2 = (uint32_t)vita_buf[2];
		uint32_t vb3 = (uint32_t)vita_buf[3];
		vb2 = ((vb2 &  0x000000ff) << 24)
			| ((vb2 &  0x0000ff00) << 8 )
			| ((vb2 &  0x00ff0000) >> 8 )
			| ((vb2 &  0xff000000) >> 24);

		vb3 = ((vb3 &  0x000000ff) << 24)
			| ((vb3 &  0x0000ff00) << 8 )
			| ((vb3 &  0x00ff0000) >> 8 )
			| ((vb3 &  0xff000000) >> 24);

		uint64_t time_ticks = ((uint64_t)vb2 << 32) | ((uint64_t)vb3);

		// determine the beginning of time
		if (_start_ticks == 0) {
			_start_ticks = time_ticks;
		}

		// save the time to metadata
		time_ticks = time_ticks - _start_ticks;
        metadata.time_spec = time_spec_t::from_ticks(time_ticks, _rate);

		// process vita sequencing
		uint32_t header = vita_buf[0];
		if (_prev_frame > (header & 0xf0000) >> 16) {
			metadata.out_of_sequence = true;
		} else {
			metadata.out_of_sequence = false;
		}
		_prev_frame = (header & 0xf0000) >> 16;

		// populate metadata
		metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;
		metadata.start_of_burst = true;		// valid for a one packet
		metadata.end_of_burst = true;		// valid for a one packet
		metadata.fragment_offset = 0;		// valid for a one packet
		metadata.more_fragments = false;	// valid for a one packet
		metadata.has_time_spec = true;		// valis for Crimson

		uint32_t vb0 = boost::endian::big_to_native( vita_buf[ 0 ] );
		size_t vita_payload_len_bytes = ( ( vb0 & 0xffff ) - (vita_hdr + vita_tlr) ) * 4;

		if ( nbytes < vita_payload_len_bytes ) {

			// buffer the remainder of the vita payload that was not received
			// so that the next subsequent call to recv() reads that.

			for ( unsigned i = 0; i < _channels.size(); i++ ) {
				size_t nb;
				size_t remaining_vita_payload_len_bytes;
				for(
					nb = nbytes,
						remaining_vita_payload_len_bytes = vita_payload_len_bytes - nb;
					remaining_vita_payload_len_bytes > 0;
					remaining_vita_payload_len_bytes -= nb
				) {
					nb = _udp_stream[i] -> stream_in( vita_buf, std::min( sizeof( vita_buf ), remaining_vita_payload_len_bytes ), _timeout );
					if ( nb == 0 ) {
						metadata.error_code =rx_metadata_t::ERROR_CODE_TIMEOUT;
						return 0;
					}

					for( unsigned j = 0; j < nb; j++ ) {
						_fifo[ i ].push( ( (uint8_t *) vita_buf ) [ j ] );
					}
				}

				// read the vita trailer (1 32-bit word)
				nb = _udp_stream[i] -> stream_in( vita_buf, vita_tlr * 4, _timeout );
				if ( nb != 4 ) {
					metadata.error_code =rx_metadata_t::ERROR_CODE_TIMEOUT;
					return 0;
				}

#ifdef DEBUG_RX
				UHD_MSG( status ) << __func__ << "():" << __LINE__ << ": PUSH [ " << (char)( i + 'A' ) << " ]: nbytes: " << vita_payload_len_bytes - nbytes_payload << ", nsamples: " << ( vita_payload_len_bytes - nbytes_payload ) / 4 << std::endl;
#endif
			}

			update_fifo_metadata( _fifo_metadata, ( vita_payload_len_bytes - nbytes ) / 4 );
		}

		return nsamples;		// removed the 5 VITA 32-bit words
	}

	void issue_stream_cmd(const stream_cmd_t &stream_cmd) {
		multi_crimson_tng m( _addr );
		for( unsigned i = 0; i < _channels.size(); i++ ) {
			m.issue_stream_cmd( stream_cmd, _channels[ i ] );
		}
		if ( uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS == stream_cmd.stream_mode ) {
			fini_rx_streamer();
		}
	}
//	std::vector<size_t> _channels;
private:
	// init function, common to both constructors
	void init_rx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels) {

		// save the tree
		_tree = tree;
		_channels = channels;
		_prev_frame = 0;
		_start_ticks = 0;
		_addr = addr;

		// get the property root path
		const fs_path mb_path   = "/mboards/0";
		const fs_path link_path = mb_path / "rx_link";

		// if no channels specified, default to channel 1 (0)
		_channels = _channels.empty() ? std::vector<size_t>(1, 0) : _channels;
		_if_mtu = std::vector<size_t>( _channels.size() );

		_fifo = std::vector<std::queue<uint8_t>>( _channels.size() );

		if ( addr.has_key( "sync_multichannel_params" ) && "1" == addr[ "sync_multichannel_params" ] ) {
			tree->access<int>( mb_path / "cm" / "chanmask-rx" ).set( channels_to_mask( _channels ) );
		}

		for (unsigned int i = 0; i < _channels.size(); i++) {
			// get the channel parameters
			std::string ch       = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			std::string udp_port = tree->access<std::string>(link_path / "Channel_"+ch / "port").get();
			std::string ip_addr  = tree->access<std::string>(link_path / "Channel_"+ch / "ip_dest").get();
			std::string iface    = tree->access<std::string>(link_path / "Channel_"+ch / "iface").get();
			_rate = tree->access<double>(mb_path / "rx_dsps" / "Channel_"+ch / "rate" / "value").get();
			_pay_len = tree->access<int>(mb_path / "link" / iface / "pay_len").get();

			_if_mtu[ i ] = get_if_mtu( ip_addr );

			// power on the channel
			tree->access<std::string>(mb_path / "rx" / "Channel_"+ch / "pwr").set("1");
			usleep(500000);

			// vita enable
			tree->access<std::string>(link_path / "Channel_"+ch / "vita_en").set("1");

			// connect to UDP port
			_udp_stream.push_back(uhd::transport::udp_stream::make_rx_stream( ip_addr, udp_port));
		}
	}

	void fini_rx_streamer() {

		const fs_path mb_path   = "/mboards/0";
		const fs_path link_path = mb_path / "rx_link";

		_tree->access<int>( mb_path / "cm" / "chanmask-rx" ).set( 0 );

		for (unsigned int i = 0; i < _channels.size(); i++) {
			std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			// power off the channel
			_tree->access<std::string>(mb_path / "rx" / "Channel_"+ch / "pwr").set("0");
			usleep(4000);
		}
	}

	// helper function to convert 8-bit allignment ==> 32-bit allignment
	void _32_align(uint32_t* data) {
		*data = (*data & 0x000000ff) << 24 |
			(*data & 0x0000ff00) << 8  |
			(*data & 0x00ff0000) >> 8  |
			(*data & 0xff000000) >> 24;
	}

	std::vector<uhd::transport::udp_stream::sptr> _udp_stream;
	std::vector<size_t> _channels;
	std::vector<std::queue<uint8_t>> _fifo;
	rx_metadata_t _fifo_metadata;
	property_tree::sptr _tree;
	size_t _prev_frame;
	size_t _pay_len;
	std::vector<size_t> _if_mtu;
	double _rate;
	uint64_t _start_ticks;
	device_addr_t _addr;
};

class crimson_tng_tx_streamer : public uhd::tx_streamer {
public:

	typedef boost::shared_ptr<crimson_tng_tx_streamer> sptr;

	crimson_tng_tx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels, boost::mutex* udp_mutex_add, std::vector<int>* async_comm,  boost::mutex* async_mutex) {
		init_tx_streamer(addr, tree, channels, udp_mutex_add, async_comm, async_mutex);
	}

	crimson_tng_tx_streamer(device_addr_t addr, property_tree::sptr tree, boost::mutex* udp_mutex_add, std::vector<int>* async_comm,  boost::mutex* async_mutex) {
		init_tx_streamer(addr, tree, std::vector<size_t>(1, 0), udp_mutex_add, async_comm, async_mutex);
	}

	~crimson_tng_tx_streamer() {
		fini_tx_streamer();
	}

	// number of channels for streamer
	size_t get_num_channels(void) const {
		return _channels.size();
	}

	// max samples per buffer per packet for sfpa, (4-bytes: 16 I, 16Q)
	size_t get_max_num_samps(void) const {
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
	static void compose_vrt49_packet(
		const tx_metadata_t &metadata,
		const uint32_t *sample_data,
		const size_t mtu_bytes,
		const size_t remaining_samples,
		const size_t buf_len,
		uint32_t *buf,
		if_packet_info_t &ifo
	) {

		const size_t N =
			std::min(
				( mtu_bytes
					- 60 // IPv4 Header
					- 8  // UDP Header
					- vrt::max_if_hdr_words32 * sizeof( uint32_t )
				) / sizeof( uint32_t ),
				remaining_samples
			);

		//translate the metadata to vrt if packet info
		ifo.link_type = vrt::if_packet_info_t::LINK_TYPE_NONE;
		ifo.packet_type = vrt::if_packet_info_t::PACKET_TYPE_DATA;
		ifo.has_sid = false; // currently unused
		ifo.has_cid = false; // currently unused
		ifo.has_tlr = false; // currently unused
		ifo.has_tsi = metadata.has_time_spec;
		ifo.has_tsf = metadata.has_time_spec;

		if ( metadata.has_time_spec ) {
			ifo.tsi_type = vrt::if_packet_info_t::TSI_TYPE_OTHER;
			ifo.tsi = (uint32_t) metadata.time_spec.get_full_secs();
			ifo.tsf_type = vrt::if_packet_info_t::TSF_TYPE_PICO;
			ifo.tsf = (uint64_t)( metadata.time_spec.get_frac_secs() * 1e12 );
		}

		// XXX: these flags denote the first and last packets in burst sample data
		// NOT indication that timed, (possibly) multi-channel burst will start
		ifo.sob = metadata.start_of_burst;
		ifo.eob	= metadata.end_of_burst;

		ifo.num_header_words32 = metadata.has_time_spec ? 4 : 1;
		ifo.num_payload_words32 = N;
		ifo.num_payload_bytes = N * sizeof( uint32_t );

		ifo.num_packet_words32 = ifo.num_header_words32 + ifo.num_payload_words32 + ( ifo.has_tlr ? 1 : 0 );

		if ( BOOST_UNLIKELY( buf_len < ifo.num_packet_words32 ) ) {
			throw runtime_error(
				(
					boost::format( "buf_len ( %u ) is not large enough for packet ( %u )" )
					% buf_len
				    % ifo.num_packet_words32
				).str()
			);
		}

		buf[ 0 ] = 0;
		buf[ 0 ] |= vrt::if_packet_info_t::PACKET_TYPE_DATA << 28;
		buf[ 0 ] |= 1 << 25; // set reserved bit (so wireshark works). this should eventually be removed
		buf[ 0 ] |= (uint16_t) ifo.num_packet_words32;

		if ( metadata.has_time_spec ) {

			buf[ 0 ] |= vrt::if_packet_info_t::TSI_TYPE_OTHER << 22;
			buf[ 0 ] |= vrt::if_packet_info_t::TSF_TYPE_PICO << 20;

			buf[ 1 ] = ifo.tsi;
			buf[ 2 ] = (uint32_t)( ifo.tsf >> 32 );
			buf[ 3 ] = (uint32_t)( ifo.tsf >> 0  );
		}
		for( size_t k = 0; k < ifo.num_header_words32; k++ ) {
			boost::endian::native_to_big_inplace( buf[ k ] );
		}

		memcpy(
			& buf[ ifo.num_header_words32 ],
			sample_data,
			N * sizeof( uint32_t )
		);

		if ( ifo.has_tlr ) {
			buf[ ifo.num_packet_words32 -1 ] = ifo.tlr;
		}
	}

	inline bool is_start_of_burst( vrt::if_packet_info_t & if_packet_info ) {
		return true
		&& if_packet_info.has_tsi
		&& if_packet_info.has_tsf
		&& vrt::if_packet_info_t::TSF_TYPE_PICO == if_packet_info.tsf_type;
	}

//	void set_sample_rate(int channel, double new_samp_rate) {
//		_samp_rate[channel] = new_samp_rate;
//		_samp_rate_usr[channel] = _samp_rate[channel];
//	}

	inline time_spec_t get_time_now() {
		if ( NULL == _crimson_tng_impl || NULL == _crimson_tng_impl->get_multi() ) {
			return time_spec_t::get_system_time();
		} else {
			return _crimson_tng_impl->get_multi()->get_time_now();
		}
	}


	size_t send(
        	const buffs_type &buffs,
        	const size_t nsamps_per_buff,
        	const tx_metadata_t &_metadata,
        	const double timeout = 0.1)
	{

		size_t samp_sent = 0;
		size_t remaining_bytes[ _channels.size() ];
		vrt::if_packet_info_t if_packet_info;

		// need r/w capabilities for 'has_time_spec'
		std::vector<tx_metadata_t> metadata;

		uhd::time_spec_t now, then, dt;

		uhd::time_spec_t send_deadline;

		if (
			true
			&& false == _metadata.start_of_burst
			&& true == _metadata.end_of_burst
			&& 0 == nsamps_per_buff
		) {
			// empty end-of-burst packet signals tx_streamer to stop
			fini_tx_streamer();
			return 0;
		}

		for ( size_t i = 0; i < _channels.size(); i++ ) {
			remaining_bytes[ i ] = nsamps_per_buff * sizeof( uint32_t );
			metadata.push_back( _metadata );
			if ( _metadata.has_time_spec ) {
				_flow_control[ i ]->set_start_of_burst_time( _metadata.time_spec );
			}
		}

		send_deadline = get_time_now();
		send_deadline += timeout;
		send_deadline += ( _metadata.has_time_spec ? _metadata.time_spec : 0.0 );

		for(
			;
			true
#ifndef DEBUG_TX
			&& get_time_now() > send_deadline
#endif
			&& samp_sent < nsamps_per_buff * _channels.size()
			;
		) {
			for ( size_t i = 0; i < _channels.size(); i++ ) {

				if ( 0 == remaining_bytes[ i ] ) {
					continue;
				}

				//
				// Compose VRT49 Packet
				//

				size_t sample_byte_offs = nsamps_per_buff * sizeof( uint32_t ) - remaining_bytes[ i ];
				compose_vrt49_packet(
					metadata[ i ],
					(uint32_t *)( & ( (uint8_t *)buffs[ i ] )[ sample_byte_offs ] ),
					_if_mtu[ i ],
					remaining_bytes[ i ] / sizeof( uint32_t ),
					CRIMSON_TNG_MAX_MTU / sizeof( uint32_t ),
					_tmp_buf[ i ],
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

				if ( dt.get_real_secs() > 30e-6 ) {
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
				// Send Data
				//

				_udp_stream[ i ]->stream_out( _tmp_buf[ i ], if_packet_info.num_packet_words32 * sizeof( uint32_t ) );

				//
				// Update Flow Control
				//

				_flow_control[ i ]->update( if_packet_info.num_payload_words32, now );

				//
				// Decrement Byte / Sample Counters
				//

				remaining_bytes[ i ] -= if_packet_info.num_payload_bytes;
				samp_sent += if_packet_info.num_payload_words32;

				// this ensures we only send the vita time spec on the first packet of the burst
				metadata[ i ].has_time_spec = false;
			}
		}

		return samp_sent / _channels.size();
	}

	// async messages are currently disabled
	bool recv_async_msg( async_metadata_t &async_metadata, double timeout = 0.1) {
		boost::ignore_unused( async_metadata, timeout );
		return false;
	}

	void set_device( crimson_tng_impl *dev ) {
		_crimson_tng_impl = dev;
	}

private:
	// init function, common to both constructors
	void init_tx_streamer( device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels,boost::mutex* udp_mutex_add, std::vector<int>* async_comm, boost::mutex* async_mutex ) {

		boost::ignore_unused( udp_mutex_add );
		boost::ignore_unused( async_comm );
		boost::ignore_unused( async_mutex );

		// kb 4001: stop-gap solution until kb #4000 is fixed!!!
		destroy_other_processes_using_crimson();

		// kb #3850: we only instantiate / converge the PID controller for the 0th txstreamer instance
		// to prevent any other constructors from returning before the PID is locked, surround the entire
		// init_tx_streamer() with mutex protection.
		num_instances_lock.lock();

		_instance_num = instance_counter++;
		num_instances++;

		// save the tree
		_tree = tree;
		_channels = channels;

		// setup the flow control UDP channel
    		_flow_iface = crimson_tng_iface::make( udp_simple::make_connected(
		        addr["addr"], BOOST_STRINGIZE(CRIMSON_TNG_FLOW_CNTRL_UDP_PORT)) );

		// get the property root path
		const fs_path mb_path   = "/mboards/0";
		const fs_path time_path = mb_path / "time";
		const fs_path prop_path = mb_path / "tx_link";

		// if no channels specified, default to channel 1 (0)
		_channels = _channels.empty() ? std::vector<size_t>(1, 0) : _channels;
		_if_mtu = std::vector<size_t>( _channels.size() );

		if ( addr.has_key( "sync_multichannel_params" ) && "1" == addr[ "sync_multichannel_params" ] ) {
			tree->access<int>( mb_path / "cm" / "chanmask-tx" ).set( channels_to_mask( _channels ) );
		}

		//Set up constants
		_max_clock_ppm_error = 100;

		bool have_time_diff_iface = false;

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
			std::string iface    = tree->access<std::string>(prop_path / "Channel_"+ch / "iface").get();
			std::string ip_addr  = tree->access<std::string>( mb_path / "link" / sfp / "ip_addr").get();
			_pay_len = tree->access<int>(mb_path / "link" / sfp / "pay_len").get();

			_if_mtu[ i ] = get_if_mtu( ip_addr );

			// power on the channel
			tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("1");
			usleep( 500000 );

			// vita enable (as of kb #3804, always use vita headers for tx)
			tree->access<std::string>(prop_path / "Channel_"+ch / "vita_en").set("1");

			_tmp_buf.push_back( new uint32_t[ CRIMSON_TNG_MAX_MTU / sizeof( uint32_t ) ] );

			// connect to UDP port
			_udp_stream.push_back(uhd::transport::udp_stream::make_tx_stream(ip_addr, udp_port));

			if ( 0 == _instance_num && ! have_time_diff_iface ) {

				// it does not currently matter whether we use the sfpa or sfpb port atm, they both access the same fpga hardware block
				int sfpa_port = tree->access<int>( mb_path / "fpga/board/flow_control/sfpa_port" ).get();
				std::string time_diff_port = std::to_string( sfpa_port );
				_time_diff_iface = uhd::transport::udp_simple::make_connected( ip_addr, time_diff_port );

				have_time_diff_iface = true;
			}

			std::vector<uint32_t> *counter = new std::vector<uint32_t>();
			counter->push_back(0);
			counter->push_back(0);

			const double nominal_sample_rate = _tree->access<double>( "/mboards/0/tx_dsps/Channel_" + ch + "/rate/value" ).get();
			const double nominal_buffer_level_pcnt = 0.5;
			_flow_control.push_back(
				uhd::flow_control_nonlinear::make(
					nominal_sample_rate,
					nominal_buffer_level_pcnt,
					(size_t)CRIMSON_TNG_BUFF_SIZE
				)
			);
		}

		if ( 0 == _instance_num ) {
			//Initialize "Time Diff" mechanism before starting flow control thread
			time_spec_t ts = time_spec_t::get_system_time();
			_streamer_start_time = ts.get_real_secs();
			// The problem is that this class does not hold a multi_crimson instance
			tree->access<time_spec_t>( time_path / "now" ).set( ts );

			// Tyreus-Luyben tuned PID controller
			_time_diff_pidc = uhd::pidc_tl(
				0.0, // desired set point is 0.0s error
				1.0, // measured K-ultimate occurs with Kp = 1.0, Ki = 0.0, Kd = 0.0
				// measured P-ultimate is inverse of 1/2 the flow-control sample rate
				2.0 / (double)CRIMSON_TNG_UPDATE_PER_SEC
			);

			//Set up initial flow control variables
			_bm_thread_should_exit = false;
			_bm_thread = std::thread( bm_thread_fn, this );
		}

		for(
			time_spec_t time_then = uhd::time_spec_t::get_system_time(),
			time_now = time_then
			; ! _time_diff_converged;
			time_now = uhd::time_spec_t::get_system_time()
		) {
			if ( (time_now - time_then).get_full_secs() > 20 ) {
				throw runtime_error( "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling Crimson?" );
			}
			usleep( 100000 );
		}

		num_instances_lock.unlock();
	}

	void fini_tx_streamer() {

		// kb #3850: while other instances are active, do not destroy the 0th instance, so that the PID
		// controller continues to track changes
		for( bool should_delay = true; should_delay;  ) {
			num_instances_lock.lock();
			if ( 0 == _instance_num && num_instances > 1 ) {
				should_delay = true;
			} else {
				num_instances--;
				should_delay = false;
			}
			num_instances_lock.unlock();

			if ( should_delay ) {
				usleep( (size_t)100e3 );
			}
		}

		if ( 0 == _instance_num ) {
			_bm_thread_should_exit = true;
			_bm_thread.join();		// wait for flow control thread to exit
		}

		const fs_path mb_path   = "/mboards/0";
		const fs_path prop_path = mb_path / "tx_link";

		_tree->access<int>( mb_path / "cm" / "chanmask-tx" ).set( 0 );

		for (unsigned int i = 0; i < _channels.size(); i++) {
			delete[] _tmp_buf[ i ];
			std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			// power off the channel
			_tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("0");
			usleep(4000);
		}

		_crimson_tng_impl = NULL;
	}

	// SoB: Time Diff (Time Diff mechanism is used to get an accurate estimate of Crimson's absolute time)
	static constexpr double tick_period_ns = 2.0 / CRIMSON_TNG_MASTER_CLOCK_RATE * 1e9;
	static inline int64_t ticks_to_nsecs( int64_t tv_tick ) {
		return (int64_t)( (double) tv_tick * tick_period_ns ) /* [tick] * [ns/tick] = [ns] */;
	}
	static inline int64_t nsecs_to_ticks( int64_t tv_nsec ) {
		return (int64_t)( (double) tv_nsec / tick_period_ns )  /* [ns] / [ns/tick] = [tick] */;
	}

	#pragma pack(push,1)
	struct time_diff_packet {
		uint64_t header; // 1 for time diff
		int64_t tv_sec;
		int64_t tv_tick;
	};
	#pragma pack(pop)

	static inline void make_time_diff_packet( time_diff_packet & pkt, time_spec_t ts = time_spec_t::get_system_time() ) {
		pkt.header = 1;
		pkt.tv_sec = ts.get_full_secs();
		pkt.tv_tick = nsecs_to_ticks( (int64_t) ( ts.get_frac_secs() * 1e9 ) );

		bo::native_to_big_inplace( pkt.header );
		bo::native_to_big_inplace( (uint64_t &) pkt.tv_sec );
		bo::native_to_big_inplace( (uint64_t &) pkt.tv_tick );
	}

	/// SoB Time Diff: send sync packet (must be done before reading flow iface)
	void time_diff_send( const uhd::time_spec_t & crimson_now ) {

		time_diff_packet pkt;

		// Input to Process (includes feedback from PID Controller)
		make_time_diff_packet(
			pkt,
			crimson_now
		);

		_time_diff_iface->send(
			boost::asio::const_buffer(
				(void *) &pkt,
				sizeof( pkt )
			)
		);
	}

	/// Extract time diff details from flow-control message string
	static inline double time_diff_extract( std::stringstream &ss ) {

		// note: requires position of stringstream to be at beginning of time diff

		int64_t tv_sec;
		int64_t tv_tick;
		uint64_t tmp;
		ss >> std::hex >> tmp;
		tv_sec = (int64_t) tmp;
		ss.ignore();
		ss >> std::hex >> tmp;
		tv_tick = (int64_t) tmp;

		// Output from Process (fed back into PID controller)
		double pv = (double)tv_sec + (double)ticks_to_nsecs( tv_tick ) / 1e9;

		return pv;
	}

	/// SoB Time Diff: feed the time diff error back into out control system
	void time_diff_process( const double pv, const uhd::time_spec_t & now ) {

		static const double sp = 0.0;
		double cv = _time_diff_pidc.update_control_variable( sp, pv, now.get_real_secs() );
		_time_diff_converged = _time_diff_pidc.is_converged( now.get_real_secs() );

		// For SoB, record the instantaneous time difference + compensation
		if ( NULL != _crimson_tng_impl && _time_diff_converged ) {
			_crimson_tng_impl->set_time_diff( cv );
		}
	}

	// the buffer monitor thread
	static void bm_thread_fn( crimson_tng_tx_streamer* txstream ) {

		uhd::set_thread_priority_safe();

		const uhd::time_spec_t T( 1.0 / (double) CRIMSON_TNG_UPDATE_PER_SEC );
		uint16_t fifo_lvl[ CRIMSON_TNG_TX_CHANNELS ];
		uhd::time_spec_t now, then, dt, overrun;
		uhd::time_spec_t crimson_now;
		struct timespec req, rem;

		double time_diff;

		for(
			overrun = 0.0,
				now = uhd::time_spec_t::get_system_time(),
				then = now + T
				;

			! txstream->_bm_thread_should_exit
				;

			then += T
		) {

			for(
				dt = then - now - overrun
					;
				dt > 0.0
					;
				now = uhd::time_spec_t::get_system_time(),
					dt = then - now - overrun
			) {
				req.tv_sec = dt.get_full_secs();
				req.tv_nsec = dt.get_frac_secs() * 1e9;
				nanosleep( &req, &rem );
			}

			time_diff = txstream->_time_diff_pidc.get_control_variable();
			crimson_now = now + time_diff;
			txstream->time_diff_send( crimson_now );

			txstream->_flow_iface -> poke_str("Read fifo");
			std::string buff_read = txstream->_flow_iface -> peek_str();

			buff_read.erase(0, 5); // remove "flow,"
			std::stringstream ss(buff_read);
			for ( int j = 0; j < CRIMSON_TNG_TX_CHANNELS; j++ ) {
				ss >> fifo_lvl[ j ];
				ss.ignore(); // skip ','
			}

			time_diff = time_diff_extract( ss );
			txstream->time_diff_process( time_diff, now );
/*
			// update flow controllers with actual buffer levels
			for( size_t i = 0; i < txstream->_channels.size(); i++ ) {
				int ch = txstream->_channels[ i ];
				txstream->_flow_control[ i ]->set_buffer_level(
					fifo_lvl[ ch ],
					crimson_now
				);
			}
*/

//			// XXX: overruns - we need to fix this
//			now = uhd::time_spec_t::get_system_time();
//
//			if ( now >= then ) {
//				UHD_MSG( warning )
//					<< __func__ << "(): Overran time for update by " << ( now - then ).get_real_secs() << " s"
//					<< std::endl;
//				if ( now - then > overrun ) {
//					overrun = now - then;
//				}
//			}
		}
	}

	// helper function to swap bytes, within 32-bits
	void _32_align(uint32_t* data) {
		*data = (*data & 0x000000ff) << 24 |
			(*data & 0x0000ff00) << 8  |
			(*data & 0x00ff0000) >> 8  |
			(*data & 0xff000000) >> 24;
	}

	std::vector<uhd::transport::udp_stream::sptr> _udp_stream;
	std::vector<uint32_t *> _tmp_buf;
	std::vector<size_t> _channels;
	std::thread _bm_thread;
	property_tree::sptr _tree;
	size_t _pay_len;
	std::vector<size_t> _if_mtu;
	std::vector<uhd::flow_control::sptr> _flow_control;
	std::vector<uhd::time_spec_t> _last_time;
	uhd::wb_iface::sptr _flow_iface;
	boost::mutex* _udp_mutex_add;
	boost::mutex* _async_mutex;
	std::vector<int>* _async_comm;
	double _max_clock_ppm_error;
	bool _bm_thread_should_exit;

	static std::mutex num_instances_lock; // mutex to prevent race conditions
	static size_t num_instances; // num_instances will fluctuate, but is non-negative
	static size_t instance_counter; // instance counter increases monotonically for _instance_num
	ssize_t _instance_num = -1; // per-instance unique id (w.r.t. libuhd.so copy)

	//debug
	time_spec_t _timer_tofreerun;

	/**
	 * Start of Burst (SoB) objects
	 */
	double _streamer_start_time;
	double _sob_time;
	/// UDP endpoint that receives our Time Diff packets
	udp_simple::sptr _time_diff_iface;
	/** PID controller that rejects differences between Crimson's clock and the host's clock.
	 *  -> The Set Point of the controller (the desired input) is the desired error between the clocks - zero!
	 *  -> The Process Variable (the measured value), is error between the clocks, as computed by Crimson.
	 *  -> The Control Variable of the controller (the output) is the required compensation for the host
	 *     such that the error is forced to zero.
	 *     => Crimson Time Now := Host Time Now + CV
	 *     => The CV contains several components:
	 *        1) A DC component that represents the (on-average) constant processing time
	 *           (i.e. the time to send / receive flow data over the network, convert strings,
	 *           do math, make system calls). The DC component is very large in comparison to the clock drift.
	 *        2) A significant amount of noise, which is the sum of the variances of network latency and
	 *           processing time (there may be other sources of noise). Even the noise is an order of
	 *           magnitude (or more) greater than the actual clock drift.
	 *        3) A slowly varying AC component that represents the actual clock drift. The actual clock
	 *           drift is dwarfed by both the noise and the DC component.
	 */
	uhd::pidc _time_diff_pidc;
	bool _time_diff_converged;
	/// Store results of time diff in _crimson_tng_impl object
	crimson_tng_impl *_crimson_tng_impl = NULL;
};
std::mutex crimson_tng_tx_streamer::num_instances_lock;
size_t crimson_tng_tx_streamer::num_instances = 0;
size_t crimson_tng_tx_streamer::instance_counter = 0;

/***********************************************************************
 * Async Data
 **********************************************************************/
bool crimson_tng_impl::recv_async_msg( async_metadata_t &async_metadata, double timeout ){
	boost::ignore_unused( async_metadata, timeout );
    return false;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr crimson_tng_impl::get_rx_stream(const uhd::stream_args_t &args){
	// Crimson currently only supports cpu_format of "sc16" (complex<int16_t>) stream
	if (strcmp(args.cpu_format.c_str(), "sc16") != 0 && strcmp(args.cpu_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON_TNG Stream only supports cpu_format of \
			\"sc16\" complex<int16_t>" << std::endl;
	}

	// Crimson currently only supports (over the wire) otw_format of "sc16" - Q16 I16 if specified
	if (strcmp(args.otw_format.c_str(), "sc16") != 0 && strcmp(args.otw_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON_TNG Stream only supports otw_format of \
			\"sc16\" Q16 I16" << std::endl;
	}

	// TODO firmware support for other otw_format, cpu_format
	return rx_streamer::sptr(new crimson_tng_rx_streamer(this->_addr, this->_tree, args.channels));
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr crimson_tng_impl::get_tx_stream(const uhd::stream_args_t &args){
	// Crimson currently only supports cpu_format of "sc16" (complex<int16_t>) stream
	if (strcmp(args.cpu_format.c_str(), "sc16") != 0 && strcmp(args.cpu_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON_TNG Stream only supports cpu_format of \
			\"sc16\" complex<int16_t>" << std::endl;
	}

	// Crimson currently only supports (over the wire) otw_format of "sc16" - Q16 I16 if specified
	if (strcmp(args.otw_format.c_str(), "sc16") != 0 && strcmp(args.otw_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON_TNG Stream only supports otw_format of \
			\"sc16\" Q16 I16" << std::endl;
	}

	// TODO firmware support for other otw_format, cpu_format
	crimson_tng_tx_streamer::sptr r( new crimson_tng_tx_streamer(this->_addr, this->_tree, args.channels, &this->_udp_mutex, &this->_async_comm, &this->_async_mutex) );
	r->set_device( this );
	return r;
}

// kb 4001: stop-gap solution until kb #4000 is fixed!!!
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>

#include <regex>

static void destroy_other_processes_using_crimson() {

	static const std::regex re( "^crimson-([1-9][0-9]*).lock$" );
	DIR *dp;
	std::string lock_file_name;
	std::smatch match;
	uint64_t pidn;
	std::stringstream ss;
	int fd;
	struct stat st;

	dp = opendir( "/tmp" );
	if ( NULL == dp ) {
		throw runtime_error(
			(
				boost::format( "opendir: %s (%d)" )
				% std::strerror( errno )
				% errno
			).str()
		);
	}

	for( dirent *de = readdir( dp ); NULL != de; de = readdir( dp ) ) {

		if ( DT_REG != de->d_type ) {
			continue;
		}

		lock_file_name = std::string( de->d_name );

		if ( ! std::regex_search( lock_file_name, match, re ) ) {
			continue;
		}

		ss = std::stringstream( match.str( 1 ) );
		ss >> pidn;

		if ( getpid() == pid_t( pidn ) ) {
			// skip this PID, if an existing tx streamer has already been made
			continue;
		}

		if ( 0 == kill( pid_t( pidn ), SIGKILL ) ) {
			UHD_MSG( warning ) << "killed hung process " << pidn << std::endl;
		}

		lock_file_name = "/tmp/" + lock_file_name;
		if ( 0 == remove( lock_file_name.c_str() ) ) {
			UHD_MSG( warning ) << "removed stale lockfile " << lock_file_name << std::endl;
		} else {
			UHD_MSG( warning ) << "failed to remove stale lockfile " << lock_file_name << std::endl;
		}
	}
	closedir( dp );

	ss.clear();
	ss << "/tmp/crimson-" << (uint64_t) getpid() << ".lock";

	lock_file_name = ss.str();

	if ( 0 == stat( lock_file_name.c_str(), &st ) ) {
		// file already exists (created by another tx streamer instance)
		return;
	}

	fd = open( lock_file_name.c_str(), O_RDWR | O_CREAT, 0666 );
	if ( -1 == fd ) {
		UHD_MSG( warning ) << "failed to create lockfile " << lock_file_name << ": " << strerror( errno ) << std::endl;
	} else {
		close( fd );
	}
}
