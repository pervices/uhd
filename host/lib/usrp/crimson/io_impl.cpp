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

#include "crimson_impl.hpp"
#include "crimson_fw_common.h"
#include <uhd/utils/log.hpp>
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
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

class crimson_rx_streamer : public uhd::rx_streamer {
public:
	crimson_rx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels) {
		init_rx_streamer(addr, tree, channels);
	}

	crimson_rx_streamer(device_addr_t addr, property_tree::sptr tree) {
		init_rx_streamer(addr, tree, std::vector<size_t>(1, 0));
	}

	~crimson_rx_streamer() {
		const fs_path mb_path   = "/mboards/0";
		const fs_path link_path = mb_path / "rx_link";
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
		_32_align(&vb2);
		_32_align(&vb3);

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
		metadata.has_time_spec = true;		// valid for Crimson
		return (nbytes / 4) - 5;		// removed the 5 VITA 32-bit words
	}

	void issue_stream_cmd(const stream_cmd_t &stream_cmd) {
		const fs_path mb_path   = "/mboards/0";
		const fs_path link_path = mb_path / "rx_link";

		for (unsigned int i = 0; i < _channels.size(); i++) {
			std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			// send off the stream command
			if (stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_START_CONTINUOUS) {
				_tree->access<std::string>(link_path / "Channel_"+ch / "stream").set("1");
			} else if (stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS) {
				_tree->access<std::string>(link_path / "Channel_"+ch / "stream").set("0");
			} else if (stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE) {
				// unimplemented
			} else {
				// unimplemented
			}
		}
	}

private:
	// init function, common to both constructors
	void init_rx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels) {
		// save the tree
		_tree = tree;
		_channels = channels;
		_prev_frame = 0;
		_start_ticks = 0;

		// get the property root path
		const fs_path mb_path   = "/mboards/0";
		const fs_path link_path = mb_path / "rx_link";

		// if no channels specified, default to channel 1 (0)
		_channels = _channels.empty() ? std::vector<size_t>(1, 0) : _channels;

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
			usleep(600000);

			// vita enable
			tree->access<std::string>(link_path / "Channel_"+ch / "vita_en").set("1");

			// connect to UDP port
			_udp_stream.push_back(uhd::transport::udp_stream::make_rx_stream( ip_addr, udp_port));
		}
	}

	// helper function to convert 8-bit allignment ==> 32-bit allignment
	void _32_align(uint32_t* data) {
		*data   = (*data & 0x000000ff) << 24
		        | (*data & 0x0000ff00) << 8
		        | (*data & 0x00ff0000) >> 8
		        | (*data & 0xff000000) >> 24;
	}

	std::vector<uhd::transport::udp_stream::sptr> _udp_stream;
	std::vector<size_t> _channels;
	property_tree::sptr _tree;
	size_t _prev_frame;
	size_t _pay_len;
	double _rate;
	uint64_t _start_ticks;
};

class crimson_tx_streamer : public uhd::tx_streamer {
public:
	crimson_tx_streamer(device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels, boost::mutex* udp_mutex_add, std::vector<int>* async_comm,  boost::mutex* async_mutex) {
		init_tx_streamer(addr, tree, channels, udp_mutex_add, async_comm, async_mutex);
	}

	crimson_tx_streamer(device_addr_t addr, property_tree::sptr tree, boost::mutex* udp_mutex_add, std::vector<int>* async_comm,  boost::mutex* async_mutex) {
		init_tx_streamer(addr, tree, std::vector<size_t>(1, 0), udp_mutex_add, async_comm, async_mutex);
	}

	~crimson_tx_streamer() {
		num_instances--;
		disable_fc();	// Wait for thread to finish
		delete _flowcontrol_thread;

		const fs_path mb_path   = "/mboards/0";
		const fs_path prop_path = mb_path / "tx_link";
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

	size_t send(
        	const buffs_type &buffs,
        	const size_t nsamps_per_buff,
        	const tx_metadata_t &metadata,
        	const double timeout = 0.1)
	{
		size_t samp_sent = 0;
		size_t remaining_bytes[_channels.size()];
		size_t nbytes_per_buff = nsamps_per_buff << 2;	// multiply by 4
		for (unsigned int i = 0; i < _channels.size(); i++) {
			remaining_bytes[i] =  nbytes_per_buff;
		}

		// Timeout
		time_spec_t timeout_lapsed = time_spec_t::get_system_time() + time_spec_t(timeout);
		bool do_multi_instance_sleep = false;

		while (samp_sent < (nsamps_per_buff * _channels.size())) {			// All Samples for all channels must be sent
			// send to each connected stream data in buffs[i]
			for (unsigned int i = 0; i < _channels.size(); i++) {			// buffer to read in data plus room for VITA

				// Skip Channel if Nothing left to send
				if (remaining_bytes[i] == 0) continue;
				size_t ret = 0;
				size_t next_pkt_size = CRIMSON_MAX_MTU;
				size_t samp_ptr_offset = nbytes_per_buff - remaining_bytes[i];
				do_multi_instance_sleep = false;

				// update sample rate if we don't know the sample rate
				setup_steadystate(i);

				if (_en_fc) {
					if ( _overflow_flag[i] || (time_spec_t::get_system_time() < _last_time[i]) ) {
						update_samplerate(i);
						if (time_spec_t::get_system_time() < _last_time[i])	continue;	// if time still remaining
					}
				}

				// Maximize the packet size unless not enough samples remaining
				if (remaining_bytes[i] < CRIMSON_MAX_MTU) {
					next_pkt_size = remaining_bytes[i];
					do_multi_instance_sleep = (num_instances > 1);
				}

				//Send data (byte operation)
				ret += _udp_stream[i] -> stream_out(buffs[i] + samp_ptr_offset, next_pkt_size);

				//update last_time with when it was supposed to have been sent:
				if (_en_fc) _last_time[i] = _last_time[i]+time_spec_t(0, (double)(next_pkt_size / 4.0) / (double)_samp_rate[i]);
				else _last_time[i] = time_spec_t::get_system_time();

				remaining_bytes[i] -= ret;
				samp_sent += (ret >> 2);	// divide by 4
			}

			// Sleep after channel's Final packet if multiple instance are active
			if (do_multi_instance_sleep) {
				boost::this_thread::sleep(boost::posix_time::microseconds(1));
			}

			// Exit if Timeout has lapsed
			if (time_spec_t::get_system_time() > timeout_lapsed)
				return samp_sent / _channels.size();
		}

		return samp_sent / _channels.size();
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
			for ( size_t c = 0; c < _channels.size(); c++ ) {
				_samp_rate[c] = _samp_rate_usr;
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
		_flow_iface = crimson_iface::make(
				udp_simple::make_connected(
					  addr["addr"]
					, BOOST_STRINGIZE(CRIMSON_FLOW_CNTRL_UDP_PORT)
				)
		);

		// get the property root path
		const fs_path mb_path   = "/mboards/0";
		const fs_path prop_path = mb_path / "tx_link";

		// if no channels specified, default to channel 1 (0)
		_channels = _channels.empty() ? std::vector<size_t>(1, 0) : _channels;

		//Set up mutex variables
		_udp_mutex_add = udp_mutex_add;
		_async_comm = async_comm;
		_async_mutex = async_mutex;

		//Set up constants
		_max_clock_ppm_error = 100;

		// Sample rate for the Instance (What User Has Set)
		_samp_rate_usr = 0;

		for (unsigned int i = 0; i < _channels.size(); i++) {
			std::string ch       = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
			std::string udp_port = tree->access<std::string>(prop_path / "Channel_"+ch / "port").get();
			std::string iface    = tree->access<std::string>(prop_path / "Channel_"+ch / "iface").get();
			std::string ip_addr  = tree->access<std::string>( mb_path / "link" / iface / "ip_addr").get();
			_pay_len = tree->access<int>(mb_path / "link" / iface / "pay_len").get();

			// power on the channel
			tree->access<std::string>(mb_path / "tx" / "Channel_"+ch / "pwr").set("1");
			usleep(600000);

			// vita disable
			tree->access<std::string>(prop_path / "Channel_"+ch / "vita_en").set("0");

			// connect to UDP port
			_udp_stream.push_back(uhd::transport::udp_stream::make_tx_stream(ip_addr, udp_port));

			// initialize controlled sample rate
			_samp_rate.push_back(0);

			// initialize the _last_time
			_last_time.push_back(time_spec_t(0.0));

			// initialise FIFO Steady State Targets
			_buffer_alert.push_back(false);
			_underflow_flag.push_back(true);	// This is set the steady state
			_overflow_flag.push_back(false);
		}

		//Set up initial flow control variables
		_fifo_level_perc = 80;
		_flow_running=true;
		_en_fc=true;
		_flowcontrol_thread = new boost::thread(init_flowcontrol, this);
		num_instances++;
	}

	 // Flow Control (should be called once on seperate thread)
	static void init_flowcontrol(crimson_tx_streamer* txstream) {

		//Get flow control updates x times a second
		uint32_t wait = 1000/CRIMSON_UPDATE_PER_SEC;
		txstream->_flow_running = true;
		uint8_t samp_rate_update_ctr = 4;
		double new_samp_rate;

		{	// Immediately request for of Sample Rate
			std::string ch = boost::lexical_cast<std::string>((char)(txstream->_channels[0] + 65));
			// Access time found to be between 1.3ms to 2.2ms
			new_samp_rate = txstream->_tree->access<double>("/mboards/0/tx_dsps/Channel_"+ch+"/rate/value").get();

			txstream->_flowcontrol_mutex.lock();

			if (new_samp_rate < CRIMSON_SS_FIFOLVL_THRESHOLD)
				txstream->_fifo_level_perc = 50;
			else
				txstream->_fifo_level_perc = 80;
			txstream->_samp_rate_usr = new_samp_rate;
			for ( size_t c = 0; c < txstream->_channels.size(); c++ ) {
				txstream->_samp_rate[c] = txstream->_samp_rate_usr;
			}

			txstream->_flowcontrol_mutex.unlock();
		}

		try {
				boost::this_thread::sleep(boost::posix_time::milliseconds(wait));
		} catch (boost::thread_interrupted&) {
			return;
		}

		while(true) {

			//Sleep for desired time
			// Catch Interrupts to Exit here
			try {
				if (samp_rate_update_ctr < CRIMSON_SAMP_RATE_REFRESH_CTR_MAX) {
					boost::this_thread::sleep( boost::posix_time::milliseconds(wait) );
				} else {	// Reduce wait by the approx time (~2ms) it takes for sample rate updates
					boost::this_thread::sleep( boost::posix_time::milliseconds(wait - 2) );
				}
			} catch (boost::thread_interrupted&) {
				return;
			}

			// Get Sample Rate Information if update counter threshold has been reached

			if (samp_rate_update_ctr < CRIMSON_SAMP_RATE_REFRESH_CTR_MAX) {		// Sample Rate will get updated every fifth loop
				samp_rate_update_ctr++;
			} else {
				std::string ch = boost::lexical_cast<std::string>((char)(txstream->_channels[0] + 65));
				// Access time found to be between 1.3ms to 2.2ms
				new_samp_rate = txstream->_tree->access<double>("/mboards/0/tx_dsps/Channel_"+ch+"/rate/value").get();

				samp_rate_update_ctr = 0;
			}

			//get data under mutex lock
			txstream->_udp_mutex_add->lock();
			txstream->_flow_iface -> poke_str("Read fifo");
			std::string buff_read = txstream->_flow_iface -> peek_str();

			txstream->_udp_mutex_add->unlock();

			// remove the "flow," at the beginning of the string
			buff_read.erase(0, 5);

			//Prevent multiple access
			txstream->_flowcontrol_mutex.lock();

			// Update Sample Rates
			if (samp_rate_update_ctr == 0) {
				if (new_samp_rate != txstream->_samp_rate_usr) {
					if (new_samp_rate < CRIMSON_SS_FIFOLVL_THRESHOLD)
						txstream->_fifo_level_perc = 50;
					else
						txstream->_fifo_level_perc = 80;
					txstream->_samp_rate_usr = new_samp_rate;
					for ( size_t c = 0; c < txstream->_channels.size(); c++ ) {
						txstream->_samp_rate[c] = txstream->_samp_rate_usr;
					}
				}
			}

			// read in each fifo level, ignore() will skip the commas
			std::stringstream ss(buff_read);
			for (int j = 0; j < 4; j++) {
				ss >> txstream->_fifo_lvl[j];
				ss.ignore();
			}

			//increment buffer count to say we have data
			for ( size_t j = 0; j < txstream->_channels.size(); j++ ) {
				txstream->_buffer_alert[j] = true;	// For coordinating sample rate updates
			}
			txstream->_async_mutex->lock();

			//If under run, alert controller
			for ( size_t ch = 0; ch < txstream->_channels.size(); ch++ ) {				// Alert send when FIFO level is < 20% (~13106)
				if ( txstream->_fifo_lvl[txstream->_channels[ch]] < CRIMSON_UNDERFLOW_LIMIT ) {
					txstream->_underflow_flag[ch] = true;
				} else if (txstream->_fifo_lvl[txstream->_channels[ch]] > CRIMSON_OVERFLOW_LIMIT ) {	// Alert send when FIFO level > 90% (~58979)
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
		if(_flowcontrol_mutex.try_lock()) {
			if(_buffer_alert[channel]) {
				// calculate the error - aim for steady state fifo level percentage
				double f_update = ((CRIMSON_BUFF_SIZE*_fifo_level_perc/100)- _fifo_lvl[_channels[channel]]) / (CRIMSON_BUFF_SIZE);
				//apply correction
				_samp_rate[channel]=_samp_rate[channel]+(2*f_update*_samp_rate[channel])/10000000;

				//Limit the correction
				//Maximum correction is a half buffer per second (a buffer element is 2 samples).
				double max_corr = _samp_rate_usr * (_max_clock_ppm_error/1000000);
				if (max_corr > CRIMSON_BUFF_SIZE) max_corr=CRIMSON_BUFF_SIZE;
				if (_samp_rate[channel] > (_samp_rate_usr + max_corr)) {
					_samp_rate[channel] = _samp_rate_usr + max_corr;
				} else if(_samp_rate[channel] < (_samp_rate_usr - max_corr)) {
					_samp_rate[channel] = _samp_rate_usr - max_corr;
				}

				//Adjust last time to try and correct to 50%
				//The adjust is 1/20th as that is the update period
				if (_fifo_lvl[_channels[channel]] > (CRIMSON_BUFF_SIZE*_fifo_level_perc/100)){
					time_spec_t lvl_adjust = time_spec_t(0,
							((_fifo_lvl[_channels[channel]]-(CRIMSON_BUFF_SIZE*_fifo_level_perc/100))*2/20) / (double)_samp_rate[channel]);
					_last_time[channel] += lvl_adjust;
				}else{
					time_spec_t lvl_adjust = time_spec_t(0,
							(((CRIMSON_BUFF_SIZE*_fifo_level_perc/100)-_fifo_lvl[_channels[channel]])*2/20) / (double)_samp_rate[channel]);
					_last_time[channel] -= lvl_adjust;
				}

				// Handle OverFlow Alerts
				if (_overflow_flag[channel]) {
					time_spec_t delay_buffer_ss = time_spec_t(0, ((_fifo_level_perc / 2)/100*(double)(CRIMSON_BUFF_SIZE)) / (double)_samp_rate[channel]);
					_last_time[channel] += delay_buffer_ss;

					_overflow_flag[channel] = false;
				}

				//Buffer is now handled
				_buffer_alert[channel] = false;
			}
			_flowcontrol_mutex.unlock();
		}
	}

	void setup_steadystate(size_t i) {	// i is the channel assignment
		if (_samp_rate[i] == 0) {
			//Get sample rate
			if (_samp_rate_usr == 0) {
				std::string ch = boost::lexical_cast<std::string>((char)(_channels[i] + 65));
				_samp_rate_usr = _tree->access<double>("/mboards/0/tx_dsps/Channel_"+ch+"/rate/value").get();
			}

			//Set the user set sample rate to refer to later
			_samp_rate[i] = _samp_rate_usr;

			// Set FIFO level steady state target accordingly
			if (_samp_rate[i] < CRIMSON_SS_FIFOLVL_THRESHOLD)
				_fifo_level_perc = 50;
			else
				_fifo_level_perc = 80;
		}

		if (_underflow_flag[i]) {	// Handle UnderFlow if it occurs
			//Adjust sample rate to fill up buffer in first half second
			//we do this by setting the "last time " data was sent to be half a buffers worth in the past
			//each element in the buffer is 1 samples worth
			time_spec_t past_buffer_ss = time_spec_t(0, (_fifo_level_perc/100*(double)(CRIMSON_BUFF_SIZE)) / (double)_samp_rate[i]);
			_last_time[i] = time_spec_t::get_system_time()-past_buffer_ss;

			_underflow_flag[i] = !_en_fc;
		}
	}

	// helper function to swap bytes, within 32-bits
	void _32_align(uint32_t* data) {
		*data = (*data & 0x000000ff) << 24
		      | (*data & 0x0000ff00) << 8
		      | (*data & 0x00ff0000) >> 8
		      | (*data & 0xff000000) >> 24;
	}

	std::vector<uhd::transport::udp_stream::sptr> _udp_stream;
	std::vector<size_t> _channels;
	boost::thread *_flowcontrol_thread;
	std::vector<double> _samp_rate;
	double				_samp_rate_usr;
	std::vector<bool> _underflow_flag;
	std::vector<bool> _overflow_flag;
	std::vector<bool> _buffer_alert;
	std::vector<time_spec_t> _last_time;
	property_tree::sptr _tree;
	size_t _pay_len;
	uhd::wb_iface::sptr _flow_iface;
	boost::mutex _flowcontrol_mutex;
	double _fifo_lvl[4];
	bool _flow_running;
	boost::mutex* _udp_mutex_add;
	boost::mutex* _async_mutex;
	std::vector<int>* _async_comm;
	double _fifo_level_perc;
	double _max_clock_ppm_error;
	bool _en_fc;
	static size_t num_instances;

	//debug
	time_spec_t _timer_tofreerun;
};
size_t crimson_tx_streamer::num_instances = 0;

/***********************************************************************
 * Async Data
 **********************************************************************/
// async messages are currently disabled and are deprecated according to UHD
bool crimson_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    return false;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr crimson_impl::get_rx_stream(const uhd::stream_args_t &args){
	// Crimson currently only supports cpu_format of "sc16" (complex<int16_t>) stream
	if (strcmp(args.cpu_format.c_str(), "sc16") != 0 && strcmp(args.cpu_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON Stream only supports cpu_format of \
			\"sc16\" complex<int16_t>" << std::endl;
	}

	// Crimson currently only supports (over the wire) otw_format of "sc16" - Q16 I16 if specified
	if (strcmp(args.otw_format.c_str(), "sc16") != 0 && strcmp(args.otw_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON Stream only supports otw_format of \
			\"sc16\" Q16 I16" << std::endl;
	}

	// TODO firmware support for other otw_format, cpu_format
	return rx_streamer::sptr(new crimson_rx_streamer(this->_addr, this->_tree, args.channels));
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr crimson_impl::get_tx_stream(const uhd::stream_args_t &args){
	// Crimson currently only supports cpu_format of "sc16" (complex<int16_t>) stream
	if (strcmp(args.cpu_format.c_str(), "sc16") != 0 && strcmp(args.cpu_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON Stream only supports cpu_format of \
			\"sc16\" complex<int16_t>" << std::endl;
	}

	// Crimson currently only supports (over the wire) otw_format of "sc16" - Q16 I16 if specified
	if (strcmp(args.otw_format.c_str(), "sc16") != 0 && strcmp(args.otw_format.c_str(), "") != 0 ) {
		UHD_MSG(error) << "CRIMSON Stream only supports otw_format of \
			\"sc16\" Q16 I16" << std::endl;
	}

	// TODO firmware support for other otw_format, cpu_format
	return tx_streamer::sptr(new crimson_tx_streamer(this->_addr, this->_tree, args.channels, &this->_udp_mutex, &this->_async_comm, &this->_async_mutex));
}
