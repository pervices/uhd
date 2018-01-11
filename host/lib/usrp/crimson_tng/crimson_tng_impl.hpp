//
// Copyright 2014-2015 Per Vices Corporation
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

#ifndef INCLUDED_CRIMSON_TNG_IMPL_HPP
#define INCLUDED_CRIMSON_TNG_IMPL_HPP

#include <set>
#include <vector>
#include <thread>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/multi_crimson_tng.hpp"

#include "uhd/transport/udp_zero_copy.hpp"

#include "crimson_tng_iface.hpp"
#include "crimson_tng_impl.hpp"
#include "pidc.hpp"

namespace uhd {
namespace usrp {

#pragma pack(push,1)
struct time_diff_req {
	uint64_t header; // 1 for time diff
	int64_t tv_sec;
	int64_t tv_tick;
};
#pragma pack(pop)

#pragma pack(push,1)
struct time_diff_uoflow {
	uint64_t uflow;
	uint64_t oflow;
};
#pragma pack(pop)

#pragma pack(push,1)
struct time_diff_resp {
	int64_t tv_sec;
	int64_t tv_tick;
	uint16_t fifo[ CRIMSON_TNG_TX_CHANNELS ];
	struct time_diff_uoflow uoflow[ CRIMSON_TNG_TX_CHANNELS ];
};
#pragma pack(pop)

#pragma pack(push,1)
struct rx_sob_req {
    uint64_t header;
    int64_t tv_sec;    // when the SoB should take place
    int64_t tv_psec;   // when the SoB should take place (ps)
};
#pragma pack(pop)

}
}

#include "crimson_tng_tx_streamer.hpp"

namespace uhd {
namespace usrp {

class crimson_tng_impl : public uhd::device
{
public:
    // shared pointer to the Crimson device
    typedef boost::shared_ptr<crimson_tng_impl> sptr;

    // This is the core constructor to be called when a crimson_tng device is found
    crimson_tng_impl(const uhd::device_addr_t &);
    ~crimson_tng_impl(void);

    // pointers to the streams for the device
    // these functions are defined in io_impl.cpp
    virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);

    // UHD legacy support
    virtual bool recv_async_msg(uhd::async_metadata_t &async_metadata, double timeout = 0.1);

    uhd::device_addr_t _addr;

    uhd::time_spec_t get_time_now() {
    	double diff = time_diff_get();
    	return time_spec_t::get_system_time() + diff;
    }
    inline double time_diff_get() { return _time_diff; }
    inline void time_diff_set( double time_diff ) { _time_diff = time_diff; }
    bool time_diff_converged();
    void start_bm();
    void stop_bm();

    void bm_listener_add( uhd::crimson_tng_tx_streamer *listener );
    void bm_listener_rem( uhd::crimson_tng_tx_streamer *listener );

    void send_rx_sob_req( const rx_sob_req & req );
    static void make_rx_sob_req_packet( const uhd::time_spec_t & ts, const size_t channel, uhd::usrp::rx_sob_req & pkt );

private:
    // helper functions to wrap send and recv as get and set
    std::string get_string(std::string req);
    void set_string(const std::string pre, std::string data);

    // wrapper for type <double> through the ASCII Crimson interface
    double get_double(std::string req);
    void set_double(const std::string pre, double data);

    // wrapper for type <bool> through the ASCII Crimson interface
    bool get_bool(std::string req);
    void set_bool(const std::string pre, bool data);

    // wrapper for type <int> through the ASCII Crimson interface
    int get_int(std::string req);
    void set_int(const std::string pre, int data);

    // wrapper for type <mboard_eeprom_t> through the ASCII Crimson interface
    uhd::usrp::mboard_eeprom_t get_mboard_eeprom(std::string req);
    void set_mboard_eeprom(const std::string pre, uhd::usrp::mboard_eeprom_t data);

    // wrapper for type <dboard_eeprom_t> through the ASCII Crimson interface
    uhd::usrp::dboard_eeprom_t get_dboard_eeprom(std::string req);
    void set_dboard_eeprom(const std::string pre, uhd::usrp::dboard_eeprom_t data);

    // wrapper for type <sensor_value_t> through the ASCII Crimson interface
    uhd::sensor_value_t get_sensor_value(std::string req);
    void set_sensor_value(const std::string pre, uhd::sensor_value_t data);

    // wrapper for type <meta_range_t> through the ASCII Crimson interface
    uhd::meta_range_t get_meta_range(std::string req);
    void set_meta_range(const std::string pre, uhd::meta_range_t data);

    // wrapper for type <complex<double>> through the ASCII Crimson interface
    std::complex<double>  get_complex_double(std::string req);
    void set_complex_double(const std::string pre, std::complex<double> data);

    // wrapper for type <stream_cmd_t> through the ASCII Crimson interface
    uhd::stream_cmd_t get_stream_cmd(std::string req);
    void set_stream_cmd(const std::string pre, uhd::stream_cmd_t data);

    // wrapper for type <time_spec_t> through the ASCII Crimson interface
    uhd::time_spec_t get_time_spec(std::string req);
    void set_time_spec(const std::string pre, uhd::time_spec_t data);

    // set arbitrary crimson properties from dev_addr_t using mappings of the form "crimson:key" => "val"
    void set_properties_from_addr();

    // private pointer to the UDP interface, this is the path to send commands to Crimson
    uhd::crimson_tng_iface::sptr _iface;
    std::mutex _iface_lock;

    std::vector<uhd::transport::udp_zero_copy::sptr> rx_if;

	/**
	 * Clock Domain Synchronization Objects
	 */

	/// UDP endpoint that receives our Time Diff packets
	uhd::transport::udp_simple::sptr _time_diff_iface;
	/** PID controller that rejects differences between Crimson's clock and the host's clock.
	 *  -> The Set Point of the controller (the desired input) is the desired error between the clocks - zero!
	 *  -> The Process Variable (the measured value), is error between the clocks, as computed by Crimson.
	 *  -> The Control Variable of the controller (the output) is the required compensation for the host
	 *     such that the error is forced to zero.
	 *     => Crimson Time Now := Host Time Now + CV
	 */
	uhd::pidc _time_diff_pidc;
    double _time_diff;
	bool _time_diff_converged;
	uhd::time_spec_t _streamer_start_time;
    void time_diff_send( const uhd::time_spec_t & crimson_now );
    bool time_diff_recv( time_diff_resp & tdr );
    void time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now );
    void fifo_update_process( const time_diff_resp & tdr );

    /**
     * Buffer Management Objects
     */

    std::set<uhd::crimson_tng_tx_streamer *> _bm_listeners;

	// N.B: the _bm_thread is also used for clock domain synchronization
	// N.B: the _bm_iface was removed in favour of using the _time_diff_iface
	std::thread _bm_thread;
	std::mutex _bm_thread_mutex;
	bool _bm_thread_needed;
	bool _bm_thread_running;
	bool _bm_thread_should_exit;
	static void bm_thread_fn( crimson_tng_impl *dev );
	bool is_bm_thread_needed();

	/**
	 * RX Streamer Objects
	 */
	std::vector<size_t> _rx_channels;
	uhd::stream_cmd_t _stream_cmd;
	std::vector<size_t> _stream_cmd_samples_remaining;
	std::vector<boost::weak_ptr<uhd::rx_streamer>> rx_streamers;
	double update_rx_samp_rate( const size_t & chan_i, const double & rate );
};

}
}

#endif /* INCLUDED_CRIMSON_TNG_IMPL_HPP */
