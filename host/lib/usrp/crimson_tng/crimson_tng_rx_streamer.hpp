
/*
 * crimson_tng_rx_streamer.hpp
 *
 *  Created on: 28 Jun 2017
 *      Author: cfri
 */

#ifndef HOST_LIB_USRP_CRIMSON_TNG_CRIMSON_TNG_RX_STREAMER_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_CRIMSON_TNG_RX_STREAMER_HPP_

#include <iostream>
#include <queue>
#include <vector>

#include "uhd/device.hpp"
#include "uhd/property_tree.hpp"
#include "uhd/stream.hpp"
#include "uhd/transport/udp_stream.hpp"

namespace uhd {

class crimson_tng_rx_streamer : public uhd::rx_streamer {
public:
	typedef boost::shared_ptr<crimson_tng_rx_streamer> sptr;

	crimson_tng_rx_streamer( device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels )
	:
		_prev_frame( 0 ),
		_pay_len( 0 ),
		_rate( 0.0 ),
		_start_ticks( 0 ),
		_dev( NULL )
	{
		init_rx_streamer( addr, tree, channels );
	}

	crimson_tng_rx_streamer(device_addr_t addr, property_tree::sptr tree) {
		init_rx_streamer( addr, tree, std::vector<size_t>(1, 0) );
	}

	~crimson_tng_rx_streamer() {
		fini_rx_streamer();
	}

    size_t get_num_channels(void) const;
    size_t get_max_num_samps(void) const;
    size_t recv(
        const buffs_type &buffs,
        const size_t nsamps_per_buff,
        rx_metadata_t &metadata,
        const double timeout,
        const bool one_packet
    );
    void issue_stream_cmd( const stream_cmd_t &stream_cmd );

	void update_fifo_metadata( rx_metadata_t &meta, size_t n_samples );

	void set_device( uhd::device *dev ) {
		_dev = dev;
	}

private:
	void init_rx_streamer(
		device_addr_t addr,
		property_tree::sptr tree,
		std::vector<size_t> channels
	);
	void fini_rx_streamer();

	time_spec_t get_time_now();

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

	uhd::device *_dev;
};

} /* namespace uhd */

#endif /* HOST_LIB_USRP_CRIMSON_TNG_CRIMSON_TNG_RX_STREAMER_HPP_ */
