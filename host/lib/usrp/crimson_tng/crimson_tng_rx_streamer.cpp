#include <bitset>

#include <boost/endian/buffers.hpp>
#include <boost/range/numeric.hpp>

#include "crimson_tng_rx_streamer.hpp"
#include "crimson_tng_impl.hpp"
#include "uhd/utils/msg.hpp"
#include "uhd/usrp/multi_crimson_tng.hpp"

#include "iputils.hpp"

using namespace uhd;

size_t crimson_tng_rx_streamer::get_num_channels(void) const {
	return _channels.size();
}

// max samples per buffer per packet for sfpa, (4-bytes: 16 I, 16Q)
size_t crimson_tng_rx_streamer::get_max_num_samps(void) const {
	return _pay_len/4;
}

static void make_rx_sob_req_packet( const uhd::time_spec_t & ts, const size_t channel, uhd::usrp::rx_sob_req & pkt ) {
	pkt.header = 0x10000 + channel;
	pkt.tv_sec = ts.get_full_secs();
	pkt.tv_psec = ts.get_frac_secs() * 1e12;

	std::cout << "header: " << std::hex << std::setw( 16 ) << std::setfill('0') << pkt.header << std::endl;
	std::cout << "tv_sec: " << std::dec << pkt.tv_sec << std::endl;
	std::cout << "tv_psec: " << std::dec << pkt.tv_psec << std::endl;

	boost::endian::native_to_big_inplace( pkt.header );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_sec );
	boost::endian::native_to_big_inplace( (uint64_t &) pkt.tv_psec );
}

time_spec_t crimson_tng_rx_streamer::get_time_now() {
	uhd::usrp::crimson_tng_impl *dev = static_cast<uhd::usrp::crimson_tng_impl *>( _dev );
	uhd::time_spec_t r = uhd::time_spec_t::get_system_time();
	r += NULL == dev ? 0.0 : dev->time_diff_get();
	return r;
}

size_t crimson_tng_rx_streamer::recv(
		const buffs_type &buffs,
		const size_t nsamps_per_buff,
		rx_metadata_t &metadata,
		const double timeout = 0.1,
		const bool one_packet = true )
{
	uhd::usrp::crimson_tng_impl *dev = static_cast<uhd::usrp::crimson_tng_impl *>( _dev );

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

	if ( metadata.has_time_spec ) {

		std::cout << "have rx SoB" << std::endl;

		uhd::time_spec_t now;

		now = get_time_now();
		if ( now >= metadata.time_spec ) {
			UHD_MSG( warning ) << "time now ( " << now.get_real_secs() << " ) >= metadata.time_spec ( " << metadata.time_spec.get_real_secs() << " )" << std::endl;
		}

		uhd::usrp::rx_sob_req rx_sob;
		for( unsigned i = 0; i < _channels.size(); i++ ) {
			std::cout << "Sending rx SoB req on Channel " << _channels[ i ] << std::endl;
			make_rx_sob_req_packet( metadata.time_spec, _channels[ i ], rx_sob );
			dev->send_rx_sob_req( rx_sob );
		}

		for( unsigned i = 0; i < _channels.size(); i++ ) {
			std::cout << "Emptying Channel " << _channels[ i ] << " fifo" << std::endl;
			std::queue<uint8_t> empty;
			std::swap( _fifo[ i ], empty );
		}

		for( unsigned i = 0; i < _channels.size(); i++ ) {
			std::cout << "Flushing Channel " << _channels[ i ] << " socket" << std::endl;
			// flush socket for _channels[ i ]
			uint32_t xbuf[ 128 ];
			_udp_stream[ i ]->stream_in( xbuf, 128, 1e-9 );
		}

		now = get_time_now();
		if ( now >= metadata.time_spec ) {
			UHD_MSG( warning ) << "not enough time to empty the receive buffers!" << std::endl;
		}

		_timeout += metadata.time_spec.get_real_secs();
		std::cout << "Timeout set to " << _timeout  << " socket" << std::endl;
	}

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
	time_ticks -= _start_ticks;
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

void crimson_tng_rx_streamer::issue_stream_cmd(const stream_cmd_t &stream_cmd) {
	uhd::usrp::multi_crimson_tng m( _addr );
	for( unsigned i = 0; i < _channels.size(); i++ ) {
		m.issue_stream_cmd( stream_cmd, _channels[ i ] );
	}
	if ( uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS == stream_cmd.stream_mode ) {
		fini_rx_streamer();
	}
}

void crimson_tng_rx_streamer::update_fifo_metadata( rx_metadata_t &meta, size_t n_samples ) {
	meta.time_spec += time_spec_t::from_ticks( n_samples, _rate );
}

void crimson_tng_rx_streamer::init_rx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels) {

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
		std::bitset<32> bs;
		for( auto & ch: _channels ) {
			bs.set( ch );
		}
		tree->access<int>( mb_path / "cm" / "chanmask-rx" ).set( bs.to_ulong() );
	}

	for (unsigned int i = 0; i < _channels.size(); i++) {
		// get the channel parameters
		std::string ch       = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
		std::string udp_port = tree->access<std::string>(link_path / "Channel_"+ch / "port").get();
		std::string ip_addr  = tree->access<std::string>(link_path / "Channel_"+ch / "ip_dest").get();
		std::string iface    = tree->access<std::string>(link_path / "Channel_"+ch / "iface").get();
		_rate = tree->access<double>(mb_path / "rx_dsps" / "Channel_"+ch / "rate" / "value").get();
		_pay_len = tree->access<int>(mb_path / "link" / iface / "pay_len").get();

		_if_mtu[ i ] = iputils::get_if_mtu( ip_addr );

		// power on the channel
		tree->access<std::string>(mb_path / "rx" / "Channel_"+ch / "pwr").set("1");
		usleep(500000);

		// vita enable
		tree->access<std::string>(link_path / "Channel_"+ch / "vita_en").set("1");

		// connect to UDP port
		_udp_stream.push_back(uhd::transport::udp_stream::make_rx_stream( ip_addr, udp_port));
	}
}

void crimson_tng_rx_streamer::fini_rx_streamer() {

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
