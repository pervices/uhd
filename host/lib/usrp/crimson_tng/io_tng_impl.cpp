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
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
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

#ifndef DEBUG_START_OF_BURST
//#define DEBUG_START_OF_BURST 1
#endif
#ifndef DEBUG_RECV
//#define DEBUG_RECV 1
#endif

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
using namespace uhd::transport::vrt;

namespace bo = boost::endian;
namespace asio = boost::asio;
namespace pt = boost::posix_time;
namespace trans = uhd::transport;

static int channels_to_mask( std::vector<size_t> channels ) {
	unsigned mask = 0;

	for( int x: channels ) {
		if ( 0 <= x && x <= 8 * sizeof(mask) - 1 ) {
			mask |= 1 << x;
		}
	}

	return mask;
}

static void check_mtu( const std::string & remote_addr ) {

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

#ifdef DEBUG_RECV
		//UHD_MSG( status ) << __func__ << "( buffs: " << (void *) & buffs << ", nsamps_per_buff: " << nsamps_per_buff << ", metadata: " << (void *) & metadata << ", timeout: " << timeout << ", one_packet: " << one_packet << " )" << std::endl;

		// XXX: do not timeout when debugging
		_timeout = 1e6;

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

#ifdef DEBUG_RECV
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
			nsamples = nbytes / 4 - (vita_hdr + vita_tlr);

#ifdef DEBUG_RECV
			UHD_MSG( status ) << __func__ << "():" << __LINE__ << ": STREAM [ " << (char)( i + 'A' ) << " ]: nbytes: " << nbytes << ", nsamples: " << nsamples << std::endl;
#endif

			// copy non-vita packets to buffs[0]
			memcpy(buffs[i], vita_buf + vita_hdr , nsamps_per_buff * 4);
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
		size_t nbytes_payload = nbytes - (vita_hdr + vita_tlr) * 4;
		size_t vita_payload_len_bytes = ( ( vb0 & 0xffff ) - (vita_hdr + vita_tlr) ) * 4;

		if ( nbytes_payload < vita_payload_len_bytes ) {

			// buffer the remainder of the vita payload that was not received
			// so that the next subsequent call to recv() reads that.

			for ( unsigned i = 0; i < _channels.size(); i++ ) {
				size_t nb;
				size_t remaining_vita_payload_len_bytes;
				for(
					nb = nbytes_payload,
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

#ifdef DEBUG_RECV
				UHD_MSG( status ) << __func__ << "():" << __LINE__ << ": PUSH [ " << (char)( i + 'A' ) << " ]: nbytes: " << vita_payload_len_bytes - nbytes_payload << ", nsamples: " << ( vita_payload_len_bytes - nbytes_payload ) / 4 << std::endl;
#endif
			}

			update_fifo_metadata( _fifo_metadata, ( vita_payload_len_bytes - nbytes_payload ) / 4 );
		}

		return nsamples;		// removed the 5 VITA 32-bit words
	}

	void issue_stream_cmd(const stream_cmd_t &stream_cmd) {
		multi_crimson_tng m( _addr );
		for( unsigned i = 0; i < _channels.size(); i++ ) {
			m.issue_stream_cmd( stream_cmd, _channels[ i ] );
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

			check_mtu( ip_addr );

			// power on the channel
			tree->access<std::string>(mb_path / "rx" / "Channel_"+ch / "pwr").set("1");
			usleep(500000);

			// vita enable
			tree->access<std::string>(link_path / "Channel_"+ch / "vita_en").set("1");

			// connect to UDP port
			_udp_stream.push_back(uhd::transport::udp_stream::make_rx_stream( ip_addr, udp_port));
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

		disable_fc();	// Wait for thread to finish
		delete _flowcontrol_thread;

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
	}

	// number of channels for streamer
	size_t get_num_channels(void) const {
		return _channels.size();
	}

	// max samples per buffer per packet for sfpa, (4-bytes: 16 I, 16Q)
	size_t get_max_num_samps(void) const {
		return _pay_len/4;
	}

	void compose_if_packet_info( const tx_metadata_t &metadata, if_packet_info_t &ifo ) {

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
			ifo.tsi = (uint32_t)metadata.time_spec.get_full_secs();
			ifo.tsf_type = vrt::if_packet_info_t::TSF_TYPE_PICO;
			ifo.tsf = (uint64_t) (metadata.time_spec.get_frac_secs() / 1e12);
		}

		// XXX: these flags denote the first and last packets in burst sample data
		// NOT indication that timed, (possibly) multi-channel burst will start
		ifo.sob = metadata.start_of_burst;
		ifo.eob	= metadata.end_of_burst;

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
			return time_spec_t( _crimson_tng_impl->get_multi()->get_time_now().get_real_secs() + _crimson_tng_impl->get_time_diff() );
		}
	}



	size_t send(
        	const buffs_type &buffs,
        	const size_t nsamps_per_buff,
        	const tx_metadata_t &_metadata,
        	const double timeout = 0.1)
	{

		static const size_t CRIMSON_MAX_VITA_PAYLOAD_LEN_BYTES =
				CRIMSON_TNG_MAX_MTU - vrt::max_if_hdr_words32 * sizeof(uint32_t);

		size_t samp_sent = 0;
		size_t bytes_sent = 0;
		size_t remaining_bytes[_channels.size()];
		vrt::if_packet_info_t if_packet_info;

		// need r/w capabilities for 'has_time_spec'
		tx_metadata_t metadata = _metadata;

		for (unsigned int i = 0; i < _channels.size(); i++) {
			remaining_bytes[i] =  (nsamps_per_buff * 4);
		}

		compose_if_packet_info( metadata, if_packet_info );
		if ( is_start_of_burst( if_packet_info ) ) {
			_sob_time = metadata.time_spec.get_real_secs();
			for( unsigned i = 0; i < _channels.size(); i++ ) {
				// start sending SoB data 1/2 a buffer (in time) before SoB
				_last_time[ i ] = metadata.time_spec - (double)( CRIMSON_TNG_BUFF_SIZE / 2 ) / _crimson_samp_rate[ i ] ;
			}
		}

		// Timeout
		time_spec_t timeout_lapsed = get_time_now() + time_spec_t(timeout) + ( metadata.has_time_spec ? metadata.time_spec : time_spec_t( 0.0 ) );

		while ( samp_sent / 4 < nsamps_per_buff * _channels.size() ) {			// All Samples for all channels must be sent
			// send to each connected stream data in buffs[i]
			for (unsigned int i = 0; i < _channels.size(); i++) {					// buffer to read in data plus room for VITA

				// Skip Channel is Nothing left to send
				if (remaining_bytes[i] == 0) {
					continue;
				}

				size_t ret = 0;
				// update sample rate if we don't know the sample rate
				setup_steadystate( i );

				size_t samp_ptr_offset = nsamps_per_buff * 4 - remaining_bytes[ i ];
				size_t data_len = std::min( CRIMSON_MAX_VITA_PAYLOAD_LEN_BYTES, remaining_bytes[ i ] ) & ~(4 - 1);

				if ( _en_fc ) {

					if ( metadata.has_time_spec ) {
						double dt = _last_time[ i ].get_real_secs() - get_time_now().get_real_secs();
						if ( dt > 0.001 ) {
							//UHD_MSG( status ) << "sleeping " <<  (unsigned) ( ( dt - 0.001 ) * 1e6 ) << " us" << std::endl;
							usleep( (unsigned) ( ( dt - 0.001 ) * 1e6 ) );
						}
					}
					while ( ( get_time_now() < _last_time[i] ) ) {
						update_samplerate( i );
					}
				}

				if_packet_info.num_payload_words32 = data_len / 4;
				if_packet_info.num_payload_bytes = data_len;

				_tmp_buf[ i ][ 0 ] = 0;

				if_packet_info.num_header_words32 = 1;
				_tmp_buf[ i ][ 0 ] |= vrt::if_packet_info_t::PACKET_TYPE_DATA << 28;
				_tmp_buf[ i ][ 0 ] |= 1 << 25; // set reserved bit (so wireshark works). this should eventually be removed

				if ( metadata.has_time_spec ) {

					uint64_t ps = metadata.time_spec.get_frac_secs() * 1e12;

					_tmp_buf[ i ][ 0 ] |= vrt::if_packet_info_t::TSI_TYPE_OTHER << 22;
					_tmp_buf[ i ][ 0 ] |= vrt::if_packet_info_t::TSF_TYPE_PICO << 20;

					_tmp_buf[ i ][ 1 ] = metadata.time_spec.get_full_secs();
					_tmp_buf[ i ][ 2 ] = (uint32_t)( ps >> 32 );
					_tmp_buf[ i ][ 3 ] = (uint32_t)( ps >> 0  );

					if_packet_info.num_header_words32 += 3;
				}

				_tmp_buf[ i ][ 0 ] |= (uint16_t) ( if_packet_info.num_payload_words32 + if_packet_info.num_header_words32 );

				for( int k = 0; k < if_packet_info.num_header_words32; k++ ) {
					boost::endian::native_to_big_inplace( _tmp_buf[ i ][ k ] );
				}

				size_t header_len_bytes = if_packet_info.num_header_words32 * sizeof(uint32_t);
				std::memcpy( (uint8_t *)_tmp_buf[ i ] + header_len_bytes, (uint8_t *)buffs[i] + samp_ptr_offset, data_len );

//				UHD_MSG( status ) << "sending " << if_packet_info.num_payload_words32 << " samples to channel " << (char)( 'A' + _channels[ i ] ) << std::endl;

				ret += _udp_stream[i] -> stream_out( _tmp_buf[ i ], header_len_bytes + data_len );
				//ret -= header_len_bytes;

				//update last_time with when it was supposed to have been sent:
				time_spec_t wait = time_spec_t( 0, ( (double)data_len/ 4.0 ) / (double)_crimson_samp_rate[ i ] );

				if (_en_fc) {
					_last_time[i] = _last_time[i] + wait;
				} else {
					_last_time[i] = get_time_now();
				}

				if ( data_len <= CRIMSON_MAX_VITA_PAYLOAD_LEN_BYTES && num_instances > 1 ) {
					boost::this_thread::sleep(boost::posix_time::microseconds(1));
				}

				remaining_bytes[i] -= data_len;
				samp_sent += data_len;
			}

			// this ensures we only send the vita time spec on the first packet of the burst
			metadata.has_time_spec = false;

			// Exit if Timeout has lapsed
			if (get_time_now() > timeout_lapsed) {
				return (samp_sent / 4) / _channels.size();
			}
		}

		return samp_sent / 4 / _channels.size();
	}

	// async messages are currently disabled
	bool recv_async_msg( async_metadata_t &async_metadata, double timeout = 0.1) {
		return false;
	}
	void disable_fc() {
		_en_fc = false;
		if (_flow_running) {
			_flowcontrol_thread->interrupt();	// thread exits on interrupt
			_flowcontrol_thread->join();		// wait for flow control thread to exit

			// Restore Adjusted Sample Rates to Original Values
			for (int c = 0; c < _channels.size(); c++) {
				_crimson_samp_rate[c] = _host_samp_rate[c];
			}
		}
	}
	void enable_fc() {
		_en_fc = true;
		if (!_flow_running) {
			_flow_running = true;
			_flowcontrol_thread = new boost::thread(init_flowcontrol, this);
		}
	}

	void set_device( crimson_tng_impl *dev ) {
		_crimson_tng_impl = dev;
	}

private:
	// init function, common to both constructors
	void init_tx_streamer( device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels,boost::mutex* udp_mutex_add, std::vector<int>* async_comm, boost::mutex* async_mutex) {

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

		if ( addr.has_key( "sync_multichannel_params" ) && "1" == addr[ "sync_multichannel_params" ] ) {
			tree->access<int>( mb_path / "cm" / "chanmask-tx" ).set( channels_to_mask( _channels ) );
		}

		//Set up mutex variables
		_udp_mutex_add = udp_mutex_add;
		_async_comm = async_comm;
		_async_mutex = async_mutex;

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

			check_mtu( ip_addr );

			// power on the channel
			tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("1");
			usleep(500000);

			// vita enable (as of kb #3804, always use vita headers for tx)
			tree->access<std::string>(prop_path / "Channel_"+ch / "vita_en").set("1");

			_tmp_buf.push_back( new uint32_t[ CRIMSON_TNG_MAX_MTU ] );

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
			_buffer_count.push_back(*counter);

			// initialize sample rate
			_crimson_samp_rate.push_back(0);
			_host_samp_rate.push_back(0);

			// initialize the _last_time
			_last_time.push_back(time_spec_t(0.0));

			// initialise FIFO Steady State Targets
			_fifo_level_perc.push_back(80);
			_underflow_flag.push_back(true);
			_overflow_flag.push_back(false);

		}

		if ( 0 == _instance_num ) {
			//Initialize "Time Diff" mechanism before starting flow control thread
			time_spec_t ts = time_spec_t::get_system_time();
			_streamer_start_time = ts.get_real_secs();
			_sob_time = _streamer_start_time;
			// The problem is that this class does not hold a multi_crimson instance
			tree->access<time_spec_t>( time_path / "now" ).set( ts );

			// Tyreus-Luyben tuned PID controller
			_time_diff_pidc = uhd::pidc_tl(
				0.0, // desired set point is 0.0s error
				1.0, // measured K-ultimate occurs with Kp = 1.0, Ki = 0.0, Kd = 0.0
				// measured P-ultimate is inverse of 1/2 the flow-control sample rate
				2.0 / (double)CRIMSON_TNG_UPDATE_PER_SEC
			);
			// initial values are just to ensure that our PID does not think its converged right away
			// which could be the case if the initial y value was 0.
			_pv_derivor = uhd::diff( 0, 100 );
			_cv_filter.set_window_size( (size_t)( crimson_tng_tx_streamer::CV_FILTER_WINDOW_S * (double)CRIMSON_TNG_UPDATE_PER_SEC ) );
			_dpv_filter.set_window_size( (size_t)( crimson_tng_tx_streamer::DPV_FILTER_WINDOW_S * (double)CRIMSON_TNG_UPDATE_PER_SEC ) );
		}

		//Set up initial flow control variables
		_flow_running=true;
		_en_fc=true;
		_flowcontrol_thread = new boost::thread(init_flowcontrol, this);

		for(
			time_spec_t time_then = uhd::time_spec_t::get_system_time(),
			time_now = time_then
			; ! _pid_converged;
			time_now = uhd::time_spec_t::get_system_time()
		) {
#ifndef DEBUG_START_OF_BURST
			if ( (time_now - time_then).get_full_secs() > 20 ) {
				throw runtime_error( "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling Crimson?" );
			}
#endif
			usleep( 100000 );
		}

		num_instances_lock.unlock();
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
	void time_diff_send() {

		time_diff_packet pkt;

		// Input to Process (includes feedback from PID Controller)
		make_time_diff_packet(
			pkt,
			time_spec_t(
				get_time_now().get_real_secs()
				+ _time_diff_pidc.get_control_variable()
			)
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
	void time_diff_process( double pv ) {

		static const double sp = 0.0;

		//double cv = _cv_filter.update( _time_diff_pidc.update_control_variable( sp, pv ) );
		double cv = _time_diff_pidc.update_control_variable( sp, pv );
		double x = _time_diff_pidc.get_last_time();

		double filtered_pv = _dpv_filter.update( std::abs( pv ) );

		if ( ! _pid_converged ) {
			if ( std::abs( filtered_pv ) < crimson_tng_tx_streamer::PV_MAX_ERROR_FOR_CONVERGENCE * 0.9  ) {
				_pid_converged = true;
#ifdef DEBUG_START_OF_BURST
				UHD_MSG(status)
					<< "t: " << std::fixed << std::setprecision(6) << x << ", "
					<< "PID converged after : " << std::scientific << x - _streamer_start_time << " s" << std::endl;
#endif
			}
		} else {
			if ( std::abs( filtered_pv ) >= crimson_tng_tx_streamer::PV_MAX_ERROR_FOR_CONVERGENCE * 1.10 ) {
				_pid_converged = false;
#ifdef DEBUG_START_OF_BURST
				UHD_MSG(status)
					<< "t: " << std::fixed << std::setprecision(6) << x << ", "
					<< "PID diverged after : " << std::scientific << x - _streamer_start_time << " s" << std::endl;
#endif
			}
		}

		// For SoB, record the absolute, instantaneous time difference + compensation
		if ( NULL != _crimson_tng_impl && _pid_converged ) {
			_crimson_tng_impl->set_time_diff( cv );
		}

#if DEBUG_START_OF_BURST
		if ( 0 == clock_drift_print_counter % CRIMSON_TNG_UPDATE_PER_SEC ) {
			UHD_MSG(status)
				<< "t: " << std::fixed << std::setprecision(6) << x << ", "
				<< "cv: " << std::fixed << std::setprecision( 20 ) << cv << ", "
				<< "pv: " << std::fixed << std::setprecision( 20 ) << filtered_pv << ", "
//				<< "dpv: " << std::fixed << std::setprecision( 20 ) << dpv << ", "
				<< std::endl;
		}
		clock_drift_print_counter++;
#endif
	}

	// Flow Control (should be called once on seperate thread)
	static void init_flowcontrol(crimson_tng_tx_streamer* txstream) {

		//Get flow control updates x times a second
		uint32_t wait = 1000/CRIMSON_TNG_UPDATE_PER_SEC;
		txstream->_flow_running = true;
		uint8_t samp_rate_update_ctr = 4;
		double new_samp_rate[txstream->_channels.size()];

		try {
				boost::this_thread::sleep(boost::posix_time::milliseconds(wait));
		} catch (boost::thread_interrupted&) {
			return;
		}

		while(true) {

			//Sleep for desired time
			// Catch Interrupts to Exit here
			try {
				if (samp_rate_update_ctr < 4) {
					boost::this_thread::sleep( boost::posix_time::milliseconds(wait) );
				} else {	// Reduce wait by the approx time (~2ms) it takes for sample rate updates
					boost::this_thread::sleep( boost::posix_time::milliseconds(wait - (txstream->_channels.size()*2)) );
				}
			} catch (boost::thread_interrupted&) {
				return;
			}

			// Get Sample Rate Information if update counter threshold has been reached

			if (samp_rate_update_ctr < 4) {		// Sample Rate will get updated every fifth loop
				samp_rate_update_ctr++;
			} else {
				for (int c = 0; c < txstream->_channels.size(); c++) {
					std::string ch = boost::lexical_cast<std::string>((char)(txstream->_channels[c] + 65));
					// Access time found to be between 1.3ms to 2.2ms
					new_samp_rate[c] = txstream->_tree->access<double>("/mboards/0/tx_dsps/Channel_"+ch+"/rate/value").get();
				}
				samp_rate_update_ctr = 0;
			}

			//get data under mutex lock
			txstream->_udp_mutex_add->lock();

			if ( 0 == txstream->_instance_num ) {
				txstream->time_diff_send();
			}

			txstream->_flow_iface -> poke_str("Read fifo");
			std::string buff_read = txstream->_flow_iface -> peek_str();

			txstream->_udp_mutex_add->unlock();

			// remove the "flow," at the beginning of the string
			buff_read.erase(0, 5);

			//Prevent multiple access
			txstream->_flowcontrol_mutex.lock();

			// Update Sample Rates
			if (samp_rate_update_ctr == 0) {
				for (int c = 0; c < txstream->_channels.size(); c++) {
					if (new_samp_rate[c] != txstream->_host_samp_rate[c]) {
						if (new_samp_rate[c] < CRIMSON_TNG_SS_FIFOLVL_THRESHOLD)
							txstream->_fifo_level_perc[c] = 50;
						else
							txstream->_fifo_level_perc[c] = 80;
						txstream->_crimson_samp_rate[c] = new_samp_rate[c];
						txstream->_host_samp_rate[c] = txstream->_crimson_samp_rate[c];
					}
				}
			}

			// read in each fifo level, ignore() will skip the commas
			std::stringstream ss(buff_read);
			for (int j = 0; j < 4; j++) {
				ss >> txstream->_fifo_lvl[j];
				ss.ignore();
			}

			if ( 0 == txstream->_instance_num ) {
				txstream->time_diff_process( time_diff_extract( ss ) );
			}

			//increment buffer count to say we have data
			for (int j = 0; j < txstream->_channels.size(); j++) {
				txstream->_buffer_count[j][0]++;	// For coordinating sample rate updates
			}
			txstream->_async_mutex->lock();

			//If under run, tell user
			for (int ch = 0; ch < txstream->_channels.size(); ch++) {	// Alert send when FIFO level is < 20% (~13106)
				if (txstream->_fifo_lvl[txstream->_channels[ch]] >=0 && txstream->_fifo_lvl[txstream->_channels[ch]] < 13106 ) {
					if (txstream->_fifo_lvl[txstream->_channels[ch]] < 15) {
						txstream->_async_comm->push_back(async_metadata_t::EVENT_CODE_UNDERFLOW);
					}
					txstream->_underflow_flag[ch] = true;
				}
				else if (txstream->_fifo_lvl[txstream->_channels[ch]] > 58979) {	// Alert send when FIFO level > 90% (~58979)
					txstream->_overflow_flag[ch] = true;
				}
			}

			//unlock
			txstream->_async_mutex->unlock();
			txstream->_flowcontrol_mutex.unlock();
		}

	}

	// Actual Flow Control Controller
	void update_samplerate(size_t channel) {
		int timeout = 0;
		if(_flowcontrol_mutex.try_lock()){
			if(_buffer_count[channel][0]!=_buffer_count[channel][1]){
				setup_steadystate(channel);	// Handles Underflows
//				for (unsigned int i = 0; i < _channels.size(); i++) {
				//If mutex is locked, let the streamer loop around and try again if we are still waiting

					// calculate the error - aim for steady state fifo level percentage
					double f_update = ((CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100)- _fifo_lvl[_channels[channel]]) / (CRIMSON_TNG_BUFF_SIZE);
					//apply correction
					_crimson_samp_rate[channel]=_crimson_samp_rate[channel]+(2*f_update*_crimson_samp_rate[channel])/10000000;

					//Limit the correction
					//Maximum correction is a half buffer per second (a buffer element is 2 samples).
					double max_corr = _host_samp_rate[channel] * (_max_clock_ppm_error/1000000);
					if (max_corr> CRIMSON_TNG_BUFF_SIZE) max_corr=CRIMSON_TNG_BUFF_SIZE;
					if(_crimson_samp_rate[channel] > (_host_samp_rate[channel] + max_corr)){
						_crimson_samp_rate[channel] = _host_samp_rate[channel] + max_corr;
					}else if(_crimson_samp_rate[channel] < (_host_samp_rate[channel] - max_corr)){
						_crimson_samp_rate[channel] = _host_samp_rate[channel] - max_corr;
					}

					//Adjust last time to try and correct to 50%
					//The adjust is 1/20th as that is the update period
					if (_fifo_lvl[_channels[channel]] > (CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100)){
						time_spec_t lvl_adjust = time_spec_t(0,
								((_fifo_lvl[_channels[channel]]-(CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100))*2/20) / (double)_crimson_samp_rate[channel]);
						_last_time[channel] += lvl_adjust;
					}else{
						time_spec_t lvl_adjust = time_spec_t(0,
								(((CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100)-_fifo_lvl[_channels[channel]])*2/20) / (double)_crimson_samp_rate[channel]);
						_last_time[channel] -= lvl_adjust;
					}

					// Handle OverFlow Alerts
					if (_overflow_flag[channel]) {
						time_spec_t delay_buffer_ss = time_spec_t(0, ((_fifo_level_perc[channel] / 2)/100*(double)(CRIMSON_TNG_BUFF_SIZE)) / (double)_crimson_samp_rate[channel]);
						_last_time[channel] += delay_buffer_ss;

						_overflow_flag[channel] = false;
					}

//				}
				//Buffer is now handled
				_buffer_count[channel][1] = _buffer_count[channel][0];
			}
			_flowcontrol_mutex.unlock();
		}

	}

	void setup_steadystate(size_t i) {	// i is the channel assignment
		if (_crimson_samp_rate[i] == 0) {	// Handle UnderFlow if it occurs
			//Get sample rate
			std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			_crimson_samp_rate[i] = _tree->access<double>("/mboards/0/tx_dsps/Channel_"+ch+"/rate/value").get();

			//Set the user set sample rate to refer to later
			_host_samp_rate[i] = _crimson_samp_rate[i];

			// Set FIFO level steady state target accordingly
			if (_crimson_samp_rate[i] < CRIMSON_TNG_SS_FIFOLVL_THRESHOLD)
				_fifo_level_perc[i] = 50;
			else
				_fifo_level_perc[i] = 80;
		}

		if (_crimson_samp_rate[i] == 0 || _underflow_flag[i]) {
			//Adjust sample rate to fill up buffer in first half second
			//we do this by setting the "last time " data was sent to be half a buffers worth in the past
			//each element in the buffer is 1 samples worth
			time_spec_t past_buffer_ss = time_spec_t(0, (_fifo_level_perc[i]/100*(double)(CRIMSON_TNG_BUFF_SIZE)) / (double)_crimson_samp_rate[i]);
			_last_time[i] = time_spec_t::get_system_time()-past_buffer_ss;
			//_timer_tofreerun = time_spec_t::get_system_time() + time_spec_t(15, 0);

			_underflow_flag[i] = false;
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
	boost::thread *_flowcontrol_thread;
	std::vector<double> _crimson_samp_rate;
	std::vector<double> _host_samp_rate;
	std::vector<bool> _underflow_flag;
	std::vector<bool> _overflow_flag;
	std::vector<time_spec_t> _last_time;
	property_tree::sptr _tree;
	size_t _pay_len;
	uhd::wb_iface::sptr _flow_iface;
	boost::mutex _flowcontrol_mutex;
	double _fifo_lvl[4];
	std::vector< std::vector<uint32_t> > _buffer_count;
	bool _flow_running;
	boost::mutex* _udp_mutex_add;
	boost::mutex* _async_mutex;
	std::vector<int>* _async_comm;
	std::vector<double> _fifo_level_perc;
	double _max_clock_ppm_error;
	bool _en_fc;

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
	/**
	 * For safetely purposes, we can only rely on the PID control variable (CV) when the
	 * state of the PID controller is in convergence.
	 *
	 * We define the state of convergence when the short-time average slope of the
	 * process variable becomes negligibly small.
	 */
	static constexpr double PV_MAX_ERROR_FOR_CONVERGENCE = 100e-6;
	static constexpr double CV_FILTER_WINDOW_S = 2;
	static constexpr double DPV_FILTER_WINDOW_S = 0.25;
	static bool _pid_converged;
	uhd::diff _pv_derivor;
	uhd::sma _dpv_filter;
	uhd::sma _cv_filter;
	/// Store results of time diff in _crimson_tng_impl object
	crimson_tng_impl *_crimson_tng_impl = NULL;
#ifdef DEBUG_START_OF_BURST
	size_t clock_drift_print_counter = 0;
#endif
};
std::mutex crimson_tng_tx_streamer::num_instances_lock;
size_t crimson_tng_tx_streamer::num_instances = 0;
size_t crimson_tng_tx_streamer::instance_counter = 0;
bool crimson_tng_tx_streamer::_pid_converged = false;

/***********************************************************************
 * Async Data
 **********************************************************************/
// async messages are currently disabled and are deprecated according to UHD
bool crimson_tng_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){

	_async_mutex.lock();
	if (!_async_comm.empty()){
	//	async_metadata.event_code = (async_metadata_t::event_code_t)_async_comm.front();
		_async_comm.erase(_async_comm.begin());
	//	return true;
	}
	_async_mutex.unlock();
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
