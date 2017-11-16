#include <bitset>
#include <cinttypes>

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

//	std::cout << "header: " << std::hex << std::setw( 16 ) << std::setfill('0') << pkt.header << std::endl;
//	std::cout << "tv_sec: " << std::dec << pkt.tv_sec << std::endl;
//	std::cout << "tv_psec: " << std::dec << pkt.tv_psec << std::endl;

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
	const size_t _nsamps_per_buff =
		(
			false
			|| stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE == _stream_cmd.stream_mode
			|| stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE == _stream_cmd.stream_mode
		)
		? std::min (_stream_cmd_samples_remaining[ 0 ],nsamps_per_buff)
		: nsamps_per_buff;

	const size_t vita_hdr = 4;
	// XXX: @CF: 20170804: Should examine the received header rather than assuming the existence of a trailer
	const size_t vita_tlr = 1;
	const size_t vita_pck = _nsamps_per_buff + vita_hdr + vita_tlr;
	size_t nbytes = 0;
	size_t nsamples = 0;

	double _timeout = timeout;

	if ( _first_recv && _sob_arg > 0 && stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS != _stream_cmd.stream_mode ) {
		stream_cmd_t stream_cmd = _stream_cmd;
		stream_cmd.stream_now = false;
		stream_cmd.time_spec = get_time_now() + _sob_arg;

		// std::cout << "Time now " <<  (uint64_t)get_time_now().get_real_secs()  << " Recv at " << (uint64_t)stream_cmd.time_spec.get_real_secs() << std::endl;

		issue_stream_cmd( stream_cmd );
		_timeout += _sob_arg;
	}
	_first_recv = false;
	_sob_arg = 0;

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
			for( unsigned j = 0; j < _nsamps_per_buff * 4 && ! _fifo[ i ].empty(); j++ ) {
				( (uint8_t *) buffs[ i ] )[ j ] = _fifo[ i ].front();
				_fifo[ i ].pop();
			}
			nbytes = fifo_level[ i ] - _fifo[ i ].size();
			nsamples = nbytes / 4;

#ifdef DEBUG_RX
			UHD_MSG( status ) << __func__ << "():" << __LINE__ << ": POP [ " << (char)( i + 'A' ) << " ]: nbytes: " << nbytes << ", nsamples: " << nsamples << std::endl;
#endif
		}

		metadata = _fifo_metadata;

		update_fifo_metadata( _fifo_metadata, nsamples );

		if (
			false
			|| stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE == _stream_cmd.stream_mode
			|| stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE == _stream_cmd.stream_mode
		) {
			for( size_t i = 0; i < _channels.size(); i++ ) {
				_stream_cmd_samples_remaining[ i ] -= nsamples;
			}
		}

		return nsamples;
	}

	// read from each connected stream and dump it into the buffs[i]
	for (unsigned int i = 0; i < _channels.size(); i++) {

		// clear temp buffer and output buffer
		memset(vita_buf, 0, vita_pck * 4);
		memset(buffs[i], 0, _nsamps_per_buff * 4);

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

	boost::endian::big_to_native_inplace( vb2 );
	boost::endian::big_to_native_inplace( vb3 );

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

	if (
		false
		|| stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE == _stream_cmd.stream_mode
		|| stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE == _stream_cmd.stream_mode
	) {
		for( size_t i = 0; i < _channels.size(); i++ ) {
			_stream_cmd_samples_remaining[ i ] -= nsamples;
		}
	}

	return nsamples;		// removed the 5 VITA 32-bit words
}

static fs_path mb_root(const size_t mboard)
{
    try
    {
        //const std::string name = _tree->list("/mboards").at(mboard);
        return "/mboards/" + mboard;
    }
    catch(const std::exception &e)
    {
        throw uhd::index_error(str(boost::format("multi_usrp::mb_root(%u) - %s") % mboard % e.what()));
    }
}

static std::string chan_to_string(size_t chan) {
    return "Channel_" + boost::lexical_cast<std::string>((char)(chan + 65));
}

static fs_path rx_link_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_TNG_RX_CHANNELS) 	channel = 0;
    else				channel = chan;
    return fs_path( "/mboards/0" ) / "rx_link" / chan_to_string(channel);
}

void crimson_tng_rx_streamer::issue_stream_cmd(const stream_cmd_t &stream_cmd) {
	uhd::usrp::crimson_tng_impl *dev = static_cast<uhd::usrp::crimson_tng_impl *>( _dev );

	uhd::time_spec_t now;
	uhd::time_spec_t then;
	uhd::usrp::rx_sob_req rx_sob;
	std::string stream_prop;

	// store the _stream_cmd so that recv() can use it
	_stream_cmd = stream_cmd;

	if ( stream_cmd_t::STREAM_MODE_START_CONTINUOUS != _stream_cmd.stream_mode ) {
		for( size_t i = 0; i < _channels.size(); i++ ) {
			_tree->access<std::string>(rx_link_root( _channels[ i ] ) / "stream").set( "0" );
		}
	}

	if ( stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS == _stream_cmd.stream_mode ) {
		stream_prop = "0";
	} else {
		stream_prop = "1";
	}

	now = get_time_now();
	if ( _stream_cmd.stream_now || now >= _stream_cmd.time_spec ) {
		_stream_cmd.stream_now = true;
		then = now;
	} else {
		_stream_cmd.stream_now = false;
		then = _stream_cmd.time_spec;
	}

	for( size_t i = 0; i < _channels.size(); i++ ) {

		if ( stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS != _stream_cmd.stream_mode ) {

			make_rx_sob_req_packet( _stream_cmd.time_spec, _channels[ i ], rx_sob );
			dev->send_rx_sob_req( rx_sob );

			if ( stream_cmd_t::STREAM_MODE_START_CONTINUOUS != _stream_cmd.stream_mode ) {

				_stream_cmd_samples_remaining[ i ] = _stream_cmd.num_samps;

				clear_fifo( i );
				flush_socket( i );
			}
		}
		_tree->access<std::string>(rx_link_root( _channels[ i ] ) / "stream").set( stream_prop );
	}

	if ( uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS == stream_cmd.stream_mode ) {
		fini_rx_streamer();
	}
}

void crimson_tng_rx_streamer::update_fifo_metadata( rx_metadata_t &meta, size_t n_samples ) {
	meta.time_spec += time_spec_t::from_ticks( n_samples, _rate );
}

void crimson_tng_rx_streamer::clear_fifo( size_t chan ) {
	std::queue<uint8_t> empty;
	if ( ALL_CHANS == chan ) {
		for ( size_t i = 0; i < _fifo.size(); i++ ) {
			clear_fifo( i );
		}
		return;
	}
	std::swap( _fifo[ chan ], empty );
}

void crimson_tng_rx_streamer::flush_socket( size_t chan ) {
	uint32_t xbuf[ 128 ];
	if ( ALL_CHANS == chan ) {
		for ( size_t i = 0; i < _fifo.size(); i++ ) {
			flush_socket( i );
		}
		return;
	}
	while( _udp_stream[ chan ]->stream_in( xbuf, 128, 1e-6 ) );
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

	if ( addr.has_key( "crimson:sob" )  ) {
		if ( ! sscanf( addr[ "crimson:sob" ].c_str(), "%lf", & _sob_arg ) ) {
			UHD_MSG( warning )  << __func__ << "(): Unrecognized argument crimson:sob=" << addr[ "crimson:sob" ] << std::endl;
		}
	}

	// if no channels specified, default to channel 1 (0)
	_channels = _channels.empty() ? std::vector<size_t>(1, 0) : _channels;
	_if_mtu = std::vector<size_t>( _channels.size() );

	_fifo = std::vector<std::queue<uint8_t>>( _channels.size() );
	_stream_cmd_samples_remaining = std::vector<size_t>( _channels.size() );

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
