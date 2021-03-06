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

#ifndef INCLUDED_CYAN_4R4T_IMPL_HPP
#define INCLUDED_CYAN_4R4T_IMPL_HPP

#include <set>
#include <vector>
#include <thread>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/subdev_spec.hpp"
#include "uhd/utils/pimpl.hpp"
#include "uhd/types/ranges.hpp"
#include "uhd/types/sensors.hpp"

#include "uhd/transport/udp_zero_copy.hpp"

#include "cyan_4r4t_iface.hpp"
#include "../crimson_tng/flow_control.hpp"
#include "../crimson_tng/pidc.hpp"
#include "../crimson_tng/system_time.hpp"

typedef std::pair<uint8_t, uint32_t> user_reg_t;

// Tate has 80 GPIO signals and requires two 64-bit registers
#define NUMBER_OF_GPIO_SIGNALS 80
#define NUMBER_OF_GPIO_REGS 2
#define NUMBER_OF_XG_CONTROL_INTF 4

namespace uhd {
namespace usrp {

#pragma pack(push,1)
struct gpio_burst_req {
	uint64_t header; // Frame 1
	int64_t tv_sec;  // Frame 2
	int64_t tv_psec; // Frame 2
	uint64_t pins[NUMBER_OF_GPIO_REGS];   // Frame N
	uint64_t mask[NUMBER_OF_GPIO_REGS];   // Frame N
};
#pragma pack(pop)

#pragma pack(push,1)
struct time_diff_req {
	uint64_t header;
	int64_t tv_sec;
	int64_t tv_tick;
};
#pragma pack(pop)

#pragma pack(push,1)
struct time_diff_resp {
	int64_t tv_sec;
	int64_t tv_tick;
};
#pragma pack(pop)

#pragma pack(push,1)
struct rx_stream_cmd {
    uint64_t header;   // 0x10000 for RX SoB
    int64_t tv_sec;    // when the SoB should take place
    int64_t tv_psec;   // when the SoB should take place (ps)
    uint64_t nsamples;
};
#pragma pack(pop)

}
}

namespace uhd {
namespace usrp {

class cyan_4r4t_impl : public uhd::device
{
public:
    // shared pointer to the Crimson device
    typedef boost::shared_ptr<cyan_4r4t_impl> sptr;

    // This is the core constructor to be called when a cyan_4r4t device is found
    cyan_4r4t_impl(const uhd::device_addr_t &);
    ~cyan_4r4t_impl(void);

    // pointers to the streams for the device
    // these functions are defined in io_impl.cpp
    virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);

    bool recv_async_msg(uhd::async_metadata_t &, double);

    uhd::device_addr_t device_addr;

    uhd::time_spec_t get_time_now() {
        double diff = time_diff_get();
        return uhd::get_system_time() + diff;
    }

    std::mutex _time_diff_mutex;

    inline double time_diff_get() {
        std::lock_guard<std::mutex> _lock( _time_diff_mutex );
        return _time_diff;
    }
    inline void time_diff_set( double time_diff ) {
        std::lock_guard<std::mutex> _lock( _time_diff_mutex );
        _time_diff = time_diff;
    }

    bool time_diff_converged();
    void start_bm();
    void stop_bm();

    void send_rx_stream_cmd_req( const rx_stream_cmd & req );
    static void make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, const uhd::time_spec_t & now, const size_t channel, uhd::usrp::rx_stream_cmd & pkt );

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

    user_reg_t get_user_reg(std::string req);
    void send_gpio_burst_req(const gpio_burst_req& req);

    void set_user_reg(const std::string key, user_reg_t value);

    // set arbitrary crimson properties from dev_addr_t using mappings of the form "crimson:key" => "val"
    void set_properties_from_addr();

    // private pointer to the UDP interface, this is the path to send commands to Crimson
    //uhd::cyan_4r4t_iface::sptr _iface;
    std::mutex _iface_lock;

	/**
	 * Clock Domain Synchronization Objects
	 */

	/// UDP endpoint that receives our Time Diff packets
    std::array<uhd::transport::udp_simple::sptr, NUMBER_OF_XG_CONTROL_INTF> _time_diff_iface;
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
    void time_diff_send( const uhd::time_spec_t & crimson_now , int xg_intf);
    bool time_diff_recv( time_diff_resp & td );
    bool time_diff_recv( time_diff_resp & tdr, int xg_intf );
    void time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now );
    void fifo_update_process( const time_diff_resp & tdr );

    /**
     * Buffer Management Objects
     */

	// N.B: the _bm_thread is also used for clock domain synchronization
	// N.B: the _bm_iface was removed in favour of using the _time_diff_iface
	std::thread _bm_thread;
	std::mutex _bm_thread_mutex;
	bool _bm_thread_needed;
	bool _bm_thread_running;
	bool _bm_thread_should_exit;

    time_spec_t _command_time;

	static void bm_thread_fn( cyan_4r4t_impl *dev );
	bool is_bm_thread_needed();

    struct mb_container_type{
        cyan_4r4t_iface::sptr iface;
        std::vector<boost::weak_ptr<uhd::rx_streamer> > rx_streamers;
        std::vector<boost::weak_ptr<uhd::tx_streamer> > tx_streamers;
        std::vector<uhd::transport::zero_copy_if::sptr> rx_dsp_xports;
        std::vector<uhd::transport::zero_copy_if::sptr> tx_dsp_xports;
        std::vector<uhd::transport::udp_simple::sptr> fifo_ctrl_xports;
        // radio control core sort of like magnesium (maybe plutonium? only if it's organic)
        size_t rx_chan_occ, tx_chan_occ;
        mb_container_type(void): rx_chan_occ(0), tx_chan_occ(0){}
    };
    uhd::dict<std::string, mb_container_type> _mbc;

    UHD_PIMPL_DECL(io_impl) _io_impl;
    void io_init(void);
    //void update_tick_rate(const double rate);
    void update_rx_samp_rate(const std::string & mb, const size_t chan, const double rate);
    void update_tx_samp_rate(const std::string & mb, const size_t chan, const double rate);
    void update_rates(void);
    //update spec methods are coercers until we only accept db_name == A
    void update_rx_subdev_spec(const std::string &, const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const std::string &, const uhd::usrp::subdev_spec_t &);
    double set_tx_dsp_freq(const std::string &, const double);
    uhd::meta_range_t get_tx_dsp_freq_range(const std::string &);
    void update_clock_source(const std::string &, const std::string &);
    void program_stream_dest(uhd::transport::zero_copy_if::sptr &, const uhd::stream_args_t &);

    uhd::tune_result_t set_rx_freq(const uhd::tune_request_t &tune_request, size_t chan = 0);
    double get_rx_freq(size_t chan = 0);
    uhd::tune_result_t set_tx_freq(const uhd::tune_request_t &tune_request, size_t chan = 0);
    double get_tx_freq(size_t chan = 0);

    static void get_tx_endpoint( uhd::property_tree::sptr tree, const size_t & chan, std::string & ip_addr, uint16_t & udp_port, std::string & sfp );
};

}
}

#endif /* INCLUDED_CYAN_4R4T_IMPL_HPP */
