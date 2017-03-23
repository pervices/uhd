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

#include "crimson_tng_impl.hpp"
#include "crimson_tng_fw_common.h"
#include <uhd/utils/log.hpp>
#include <uhd/utils/pidc.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <uhd/transport/udp_stream.hpp>
#include <uhd/types/wb_iface.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>
#include <boost/endian/buffers.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

using namespace boost::endian;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

static int channels_to_mask( std::vector<size_t> channels ) {
	unsigned mask = 0;

	for( int x: channels ) {
		if ( 0 <= x && x <= 8 * sizeof(mask) - 1 ) {
			mask |= 1 << x;
		}
	}

	return mask;
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

		// temp buffer: vita hdr + data
		uint32_t vita_buf[vita_pck];

		// read from each connected stream and dump it into the buffs[i]
		for (unsigned int i = 0; i < _channels.size(); i++) {

			// clear temp buffer and output buffer
			memset(vita_buf, 0, vita_pck * 4);
			memset(buffs[i], 0, nsamps_per_buff * 4);

			// read in vita_pck*4 bytes to temp buffer
			nbytes = _udp_stream[i] -> stream_in(vita_buf, vita_pck * 4, timeout);
			if (nbytes == 0) {
				metadata.error_code =rx_metadata_t::ERROR_CODE_TIMEOUT;
				return 0;
			}

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
		return (nbytes / 4) - 5;		// removed the 5 VITA 32-bit words
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
	property_tree::sptr _tree;
	size_t _prev_frame;
	size_t _pay_len;
	double _rate;
	uint64_t _start_ticks;
	device_addr_t _addr;
};

class crimson_tng_tx_streamer : public uhd::tx_streamer {
public:
	crimson_tng_tx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels, boost::mutex* udp_mutex_add, std::vector<int>* async_comm,  boost::mutex* async_mutex) {
		init_tx_streamer(addr, tree, channels, udp_mutex_add, async_comm, async_mutex);
	}

	crimson_tng_tx_streamer(device_addr_t addr, property_tree::sptr tree, boost::mutex* udp_mutex_add, std::vector<int>* async_comm,  boost::mutex* async_mutex) {
		init_tx_streamer(addr, tree, std::vector<size_t>(1, 0), udp_mutex_add, async_comm, async_mutex);
	}

	~crimson_tng_tx_streamer() {
		num_instances--;
		disable_fc();	// Wait for thread to finish
		delete _flowcontrol_thread;

		const fs_path mb_path   = "/mboards/0";
		const fs_path prop_path = mb_path / "tx_link";

		_tree->access<int>( mb_path / "cm" / "chanmask-tx" ).set( 0 );

		for (unsigned int i = 0; i < _channels.size(); i++) {
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

//	void set_sample_rate(int channel, double new_samp_rate) {
//		_samp_rate[channel] = new_samp_rate;
//		_samp_rate_usr[channel] = _samp_rate[channel];
//	}

	size_t send(
        	const buffs_type &buffs,
        	const size_t nsamps_per_buff,
        	const tx_metadata_t &metadata,
        	const double timeout = 0.1)
	{
		size_t samp_sent =0;
		size_t remaining_bytes[_channels.size()];
		for (unsigned int i = 0; i < _channels.size(); i++) {
			remaining_bytes[i] =  (nsamps_per_buff * 4);
		}

		// Timeout
		time_spec_t timeout_lapsed = time_spec_t::get_system_time() + time_spec_t(timeout);

		while ((samp_sent / 4) < (nsamps_per_buff * _channels.size())) {			// All Samples for all channels must be sent
			// send to each connected stream data in buffs[i]
			for (unsigned int i = 0; i < _channels.size(); i++) {					// buffer to read in data plus room for VITA

				// Skip Channel is Nothing left to send
				if (remaining_bytes[i] == 0) continue;
				size_t ret = 0;
				// update sample rate if we don't know the sample rate
				setup_steadystate(i);

				size_t samp_ptr_offset = ((nsamps_per_buff*4) - remaining_bytes[i]);

				//If greater then max pl copy over what you can, leave the rest
				if (remaining_bytes[i] >= CRIMSON_TNG_MAX_MTU){
					if (_en_fc) {
						while ( (time_spec_t::get_system_time() < _last_time[i]) || _overflow_flag[i] ) {
							update_samplerate(i);
						}
					}
					//Send data (byte operation)
					ret += _udp_stream[i] -> stream_out(buffs[i] + samp_ptr_offset, CRIMSON_TNG_MAX_MTU);

					//update last_time with when it was supposed to have been sent:
					time_spec_t wait = time_spec_t(0, (double)(CRIMSON_TNG_MAX_MTU / 4.0) / (double)_samp_rate[i]);

					if (_en_fc)_last_time[i] = _last_time[i]+wait;//time_spec_t::get_system_time();
					else _last_time[i] = time_spec_t::get_system_time();

				} else {
					if (_en_fc) {
						while ( (time_spec_t::get_system_time() < _last_time[i]) || _overflow_flag[i] ) {
							update_samplerate(i);
						}
					}

					//Send data (byte operation)
					ret += _udp_stream[i] -> stream_out(buffs[i] + samp_ptr_offset, remaining_bytes[i]);

					//update last_time with when it was supposed to have been sent:
					time_spec_t wait = time_spec_t(0, (double)(remaining_bytes[i]/4) / (double)_samp_rate[i]);
					if (_en_fc)_last_time[i] = _last_time[i]+wait;//time_spec_t::get_system_time();
					else _last_time[i] = time_spec_t::get_system_time();

					if (num_instances > 1) {
						boost::this_thread::sleep(boost::posix_time::microseconds(1));
					}

				}
				remaining_bytes[i] -= ret;
				samp_sent += ret;
			}

			// Exit if Timeout has lapsed
			if (time_spec_t::get_system_time() > timeout_lapsed)
				return (samp_sent / 4) / _channels.size();
		}

		return (samp_sent / 4) / _channels.size();// -  vita_hdr - vita_tlr;	// vita is disabled
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
				_samp_rate[c] = _samp_rate_usr[c];
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

private:
	// init function, common to both constructors
	void init_tx_streamer( device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels,boost::mutex* udp_mutex_add, std::vector<int>* async_comm, boost::mutex* async_mutex) {
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

			// power on the channel
			tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("1");
			usleep(500000);

			// vita disable
			tree->access<std::string>(prop_path / "Channel_"+ch / "vita_en").set("0");

			// connect to UDP port
			_udp_stream.push_back(uhd::transport::udp_stream::make_tx_stream(ip_addr, udp_port));

			if ( ! have_time_diff_iface ) {

				// it does not currently matter whether we use the sfpa or sfpb port atm, they both access the same fpga hardware block
				std::string time_diff_port = "" + tree->access<int>( mb_path / "fpga/board/flow_control/sfpa_port" ).get();
				_time_diff_iface = uhd::transport::udp_simple::make_connected( ip_addr, time_diff_port );

				have_time_diff_iface = true;
			}

			std::vector<uint32_t> *counter = new std::vector<uint32_t>();
			counter->push_back(0);
			counter->push_back(0);
			_buffer_count.push_back(*counter);

			// initialize sample rate
			_samp_rate.push_back(0);
			_samp_rate_usr.push_back(0);

			// initialize the _last_time
			_last_time.push_back(time_spec_t(0.0));

			// initialise FIFO Steady State Targets
			_fifo_level_perc.push_back(80);
			_underflow_flag.push_back(true);
			_overflow_flag.push_back(false);

		}

		//Initialize "Time Diff" mechanism before starting flow control thread
		time_spec_t ts = time_spec_t::get_system_time();
		_start_time = ts.get_real_secs();
		tree->access<time_spec_t>( time_path / "now" ).set( ts );
		_time_diff_pidc = uhd::pidc( 0.0, 0.45454545454545453, 15.495867768595039, 0.000962000962000962 );

		//Set up initial flow control variables
		_flow_running=true;
		_en_fc=true;
		_flowcontrol_thread = new boost::thread(init_flowcontrol, this);
		num_instances++;
	}

	// SoB: Time Diff (Time Diff mechanism is used to get a really close estimate of Crimson's absolute time)
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

	static void make_time_diff_packet( time_diff_packet & pkt, double ts ) {
		make_time_diff_packet( pkt, time_spec_t( ts ) );
	}
	static void make_time_diff_packet( time_diff_packet & pkt, time_spec_t ts = time_spec_t::get_system_time() ) {
		pkt.header = 1;
		pkt.tv_sec = ts.get_full_secs();
		pkt.tv_tick = nsecs_to_ticks( (int64_t) ( ts.get_frac_secs() * 1e9 ) );

		native_to_big_inplace( pkt.header );
		native_to_big_inplace( (uint64_t &) pkt.tv_sec );
		native_to_big_inplace( (uint64_t &) pkt.tv_tick );
	}

	static void send_time_diff_packet( time_diff_packet & pkt, udp_simple::sptr & dst ) {
		boost::asio::const_buffer cb( (void *) &pkt, sizeof( pkt ) );
		dst->send( cb );
	}

	// Flow Control (should be called once on seperate thread)
	static void init_flowcontrol(crimson_tng_tx_streamer* txstream) {

		//Get flow control updates x times a second
		uint32_t wait = 1000/CRIMSON_TNG_UPDATE_PER_SEC;
		txstream->_flow_running = true;
		uint8_t samp_rate_update_ctr = 4;
		double new_samp_rate[txstream->_channels.size()];

		// SoB Time Diff
		time_diff_packet time_diff_packet;
		double sob_sync_time;

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

			if ( txstream->_sob_pending ) {
				// SoB Time Diff: send sync packet (must be done before reading flow iface)
				sob_sync_time = time_spec_t::get_system_time().get_real_secs();
				make_time_diff_packet( time_diff_packet, sob_sync_time );
				send_time_diff_packet( time_diff_packet, txstream->_time_diff_iface );
			}

			txstream->_flow_iface -> poke_str("Read fifo");
			std::string buff_read = txstream->_flow_iface -> peek_str();

			txstream->_udp_mutex_add->unlock();

			// remove the "flow," at the beginning of the string
			buff_read.erase(0, 5);

			//Prevent multiple access
			txstream->_flowcontrol_mutex.lock();

			if ( ! txstream->_sob_pending ) {

				// Update Sample Rates (used during steady-state operation)
				if (samp_rate_update_ctr == 0) {
					for (int c = 0; c < txstream->_channels.size(); c++) {
						if (new_samp_rate[c] != txstream->_samp_rate_usr[c]) {
							if (new_samp_rate[c] < CRIMSON_TNG_SS_FIFOLVL_THRESHOLD)
								txstream->_fifo_level_perc[c] = 50;
							else
								txstream->_fifo_level_perc[c] = 80;
							txstream->_samp_rate[c] = new_samp_rate[c];
							txstream->_samp_rate_usr[c] = txstream->_samp_rate[c];
						}
					}
				}
			}

			// read in each fifo level, ignore() will skip the commas
			std::stringstream ss(buff_read);
			for (int j = 0; j < 4; j++) {
				ss >> txstream->_fifo_lvl[j];
				ss.ignore();
			}

			if ( txstream->_sob_pending ) {

				// Update Sample Rates (used while preparing for SoB)

				int64_t tv_sec;
				int64_t tv_tick;
				ss >> std::hex >> tv_sec;
				ss.ignore();
				ss >> std::hex >> tv_tick;
				txstream->_time_diff =
					txstream->_time_diff_pidc.updateControlVariable(
						txstream->_time_diff,
						(double)tv_sec + (double)ticks_to_nsecs( tv_tick )
					);
				txstream->_host_to_crimson_tick_period_ratio = (sob_sync_time - txstream->_start_time) / (sob_sync_time - txstream->_start_time + txstream->_time_diff);
				txstream->_host_tick_period = txstream->_crimson_tick_period / txstream->_host_to_crimson_tick_period_ratio;


				if ( samp_rate_update_ctr == 0 ) {
					for ( int c = 0; c < txstream->_channels.size(); c++ ) {
						if (new_samp_rate[c] != txstream->_samp_rate_usr[c]) {
							if (new_samp_rate[c] < CRIMSON_TNG_SS_FIFOLVL_THRESHOLD) {
								txstream->_fifo_level_perc[c] = 50;
							} else {
								txstream->_fifo_level_perc[c] = 80;
							}
							txstream->_samp_rate[c] = new_samp_rate[c];
							txstream->_samp_rate_usr[c] = txstream->_samp_rate[c];
						}
					}
				}

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
					_samp_rate[channel]=_samp_rate[channel]+(2*f_update*_samp_rate[channel])/10000000;

					//Limit the correction
					//Maximum correction is a half buffer per second (a buffer element is 2 samples).
					double max_corr = _samp_rate_usr[channel] * (_max_clock_ppm_error/1000000);
					if (max_corr> CRIMSON_TNG_BUFF_SIZE) max_corr=CRIMSON_TNG_BUFF_SIZE;
					if(_samp_rate[channel] > (_samp_rate_usr[channel] + max_corr)){
						_samp_rate[channel] = _samp_rate_usr[channel] + max_corr;
					}else if(_samp_rate[channel] < (_samp_rate_usr[channel] - max_corr)){
						_samp_rate[channel] = _samp_rate_usr[channel] - max_corr;
					}

					//Adjust last time to try and correct to 50%
					//The adjust is 1/20th as that is the update period
					if (_fifo_lvl[_channels[channel]] > (CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100)){
						time_spec_t lvl_adjust = time_spec_t(0,
								((_fifo_lvl[_channels[channel]]-(CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100))*2/20) / (double)_samp_rate[channel]);
						_last_time[channel] += lvl_adjust;
					}else{
						time_spec_t lvl_adjust = time_spec_t(0,
								(((CRIMSON_TNG_BUFF_SIZE*_fifo_level_perc[channel]/100)-_fifo_lvl[_channels[channel]])*2/20) / (double)_samp_rate[channel]);
						_last_time[channel] -= lvl_adjust;
					}

					// Handle OverFlow Alerts
					if (_overflow_flag[channel]) {
						time_spec_t delay_buffer_ss = time_spec_t(0, ((_fifo_level_perc[channel] / 2)/100*(double)(CRIMSON_TNG_BUFF_SIZE)) / (double)_samp_rate[channel]);
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
		if (_samp_rate[i] == 0) {	// Handle UnderFlow if it occurs
			//Get sample rate
			std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			_samp_rate[i] = _tree->access<double>("/mboards/0/tx_dsps/Channel_"+ch+"/rate/value").get();

			//Set the user set sample rate to refer to later
			_samp_rate_usr[i] = _samp_rate[i];

			// Set FIFO level steady state target accordingly
			if (_samp_rate[i] < CRIMSON_TNG_SS_FIFOLVL_THRESHOLD)
				_fifo_level_perc[i] = 50;
			else
				_fifo_level_perc[i] = 80;
		}

		if (_samp_rate[i] == 0 || _underflow_flag[i]) {
			//Adjust sample rate to fill up buffer in first half second
			//we do this by setting the "last time " data was sent to be half a buffers worth in the past
			//each element in the buffer is 1 samples worth
			time_spec_t past_buffer_ss = time_spec_t(0, (_fifo_level_perc[i]/100*(double)(CRIMSON_TNG_BUFF_SIZE)) / (double)_samp_rate[i]);
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
	std::vector<size_t> _channels;
	boost::thread *_flowcontrol_thread;
	std::vector<double> _samp_rate;
	std::vector<double> _samp_rate_usr;
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
	static size_t num_instances;

	bool _sob_pending;
	uhd::pidc _time_diff_pidc;
	udp_simple::sptr _time_diff_iface;
	double _start_time;
	double _time_diff;
	const double _crimson_tick_period = 2.0 / CRIMSON_TNG_MASTER_CLOCK_RATE;
	double _host_tick_period;
	double _host_to_crimson_tick_period_ratio;

	//debug
	time_spec_t _timer_tofreerun;
};
size_t crimson_tng_tx_streamer::num_instances = 0;

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

	// Warning for preference to set the MTU size to 3600 to support Jumbo Frames
        boost::format base_message (
            "\nCrimson Warning:\n"
            "   Please set the MTU size for SFP ports to 4000.\n"
            "   The device has been optimized for Jumbo Frames\n"
	    "   to lower overhead.\n");
	UHD_MSG(status) << base_message.str();

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

	// Warning for preference to set the MTU size to 3600 to support Jumbo Frames
        boost::format base_message (
            "\nCrimson Warning:\n"
            "   Please set the MTU size for SFP ports to 4000 \n"
            "   The device has been optimized for Jumbo Frames\n"
	    "   to lower overhead.\n");
	UHD_MSG(status) << base_message.str();

	// TODO firmware support for other otw_format, cpu_format
	return tx_streamer::sptr(new crimson_tng_tx_streamer(this->_addr, this->_tree, args.channels, &this->_udp_mutex, &this->_async_comm, &this->_async_mutex));
}
