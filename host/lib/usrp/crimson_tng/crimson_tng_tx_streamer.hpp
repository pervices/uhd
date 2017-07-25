#ifndef HOST_LIB_USRP_CRIMSON_TNG_CRIMSON_TNG_TX_STREAMER_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_CRIMSON_TNG_TX_STREAMER_HPP_

#include <queue>
#include <vector>

#include "uhd/device.hpp"
#include "uhd/property_tree.hpp"
#include "uhd/stream.hpp"
#include "uhd/transport/udp_stream.hpp"
#include "uhd/transport/vrt_if_packet.hpp"

#include "flow_control.hpp"

#ifndef DEBUG_TX
#define DEBUG_TX 1
#endif

namespace uhd {

class crimson_tng_tx_streamer: public tx_streamer {

public:

	typedef boost::shared_ptr<crimson_tng_tx_streamer> sptr;

	crimson_tng_tx_streamer( device_addr_t addr, property_tree::sptr tree, std::vector<size_t> channels )
	:
		_pay_len( 0 ),
		_max_clock_ppm_error( 0.0 ),
		_dev( NULL ),
		_first_send( true ),
		_sob_arg( 0.0 )
	{
		init_tx_streamer( addr, tree, channels );
	}

	crimson_tng_tx_streamer( device_addr_t addr, property_tree::sptr tree ) {
		init_tx_streamer( addr, tree, std::vector<size_t>(1, 0) );
	}

	~crimson_tng_tx_streamer() {
		fini_tx_streamer();
	}

    size_t get_num_channels() const;
    size_t get_max_num_samps() const;
    size_t send(
        const buffs_type &buffs,
        const size_t nsamps_per_buff,
        const tx_metadata_t &metadata,
        const double timeout
    );
    bool recv_async_msg(
		async_metadata_t &async_metadata,
		double timeout
	);

	static void compose_vrt49_packet(
		const tx_metadata_t &metadata,
		const size_t mtu_bytes,
		const size_t remaining_samples,
		const size_t buf_len,
		const size_t sample_count,
		uint32_t *buf,
		transport::vrt::if_packet_info_t &ifo
	);

	time_spec_t get_time_now();

	void set_device( uhd::device *dev );

	void on_buffer_level_read( const std::vector<size_t> & buffer_levels );

private:
	// init function, common to both constructors
	void init_tx_streamer(
		device_addr_t addr,
		property_tree::sptr tree,
		std::vector<size_t> channels
	);

	void fini_tx_streamer();

	std::vector<int> _udp_socket;
	std::vector<size_t> _channels;
	std::vector<size_t> _sample_count;
	property_tree::sptr _tree;
	size_t _pay_len;
	std::vector<size_t> _if_mtu;
	std::vector<uhd::flow_control::sptr> _flow_control;
	double _max_clock_ppm_error;
	uint32_t _vita_hdr_buf[ 16 ];
	bool _first_send;
	double _sob_arg;

	/// Store results of time diff in _crimson_tng_impl object
	uhd::device *_dev;
};

} /* namespace uhd */

#endif /* HOST_LIB_USRP_CRIMSON_TNG_CRIMSON_TNG_TX_STREAMER_HPP_ */
