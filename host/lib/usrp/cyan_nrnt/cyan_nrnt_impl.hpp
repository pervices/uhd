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

#ifndef INCLUDED_CYAN_NRNT_IMPL_HPP
#define INCLUDED_CYAN_NRNT_IMPL_HPP

#include <set>
#include <vector>
#include <thread>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/multi_usrp.hpp"

#include "uhd/transport/udp_zero_copy.hpp"

#include "cyan_nrnt_iface.hpp"
#include "../crimson_tng/pidc.hpp"
#include <uhdlib/utils/system_time.hpp>
#include <uhd/transport/bounded_buffer.hpp>

#define NUMBER_OF_XG_CONTROL_INTF 4

typedef std::pair<uint8_t, uint32_t> user_reg_t;

namespace uhd {
namespace usrp {

#pragma pack(push,1)
struct gpio_burst_req {
	uint64_t header; // Frame 1
	int64_t tv_sec;  // Frame 2
	int64_t tv_psec; // Frame 2
	uint64_t pins;   // Frame 3
	uint64_t mask;   // Frame 3
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

class cyan_nrnt_impl : public uhd::device
{
public:
    // shared pointer to the Crimson device
    typedef std::shared_ptr<cyan_nrnt_impl> sptr;

    // This is the core constructor to be called when a cyan_nrnt device is found
    cyan_nrnt_impl(const uhd::device_addr_t &);
    ~cyan_nrnt_impl(void);

    // pointers to the streams for the device
    // these functions are defined in cyan_nrnt_io_impl.cpp
    virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);

    bool recv_async_msg_deprecated_warning = false;
    std::shared_ptr<uhd::transport::bounded_buffer<async_metadata_t>> _async_msg_fifo;
    bool recv_async_msg(uhd::async_metadata_t &, double);

    uhd::device_addr_t device_addr;

    uhd::time_spec_t get_time_now();
    bool time_diff_converged();
    void wait_for_time_diff_converged();
    // Note: this must start false since get_time_now gets called when initializing the state tree, before the bm thread even starts
    std::atomic<bool> time_resync_requested = false;

    inline double time_diff_get() {
        return _time_diff;
    }
    inline void time_diff_set( double time_diff ) {
        _time_diff = time_diff;
    }

    void start_bm();
    void stop_bm();

    void send_rx_stream_cmd_req( const rx_stream_cmd & req );

    void send_rx_stream_cmd_req( const rx_stream_cmd & req, int xg_intf );
    static void make_rx_stream_cmd_packet( const uhd::stream_cmd_t & cmd, const size_t channel, uhd::usrp::rx_stream_cmd & pkt );

private:
    std::string rx_link_root(const size_t channel, const size_t mboard = 0);
    std::string tx_link_root(const size_t channel, const size_t mboard = 0);
    std::string tx_dsp_root(const size_t channel, const size_t mboard = 0);

    // The buffer size in number of samples
    int64_t max_buffer_level;
    // The number to multiply get buffer level requests by to get the actual buffer level in number of samples
    int64_t buffer_level_multiple;

    int nsamps_multiple_rx;

    // The maximum smaple rate of the unit
    double max_sample_rate;

    // Over the wire format, int bits per half of the iq pair. i.e. 16 = sc16
    int otw_rx;
    std::string otw_rx_s;
    int otw_tx;
    std::string otw_tx_s;

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
    //uhd::cyan_nrnt_iface::sptr _iface;
    std::mutex _iface_lock;

	/**
	 * Clock Domain Synchronization Objects
	 */

	/// UDP endpoint that receives our Time Diff packets
	std::array<uhd::transport::udp_simple::sptr,NUMBER_OF_XG_CONTROL_INTF> _time_diff_iface;
	/** PID controller that rejects differences between Crimson's clock and the host's clock.
	 *  -> The Set Point of the controller (the desired input) is the desired error between the clocks - zero!
	 *  -> The Process Variable (the measured value), is error between the clocks, as computed by Crimson.
	 *  -> The Control Variable of the controller (the output) is the required compensation for the host
	 *     such that the error is forced to zero.
	 *     => Crimson Time Now := Host Time Now + CV
	 */
	uhd::pidc _time_diff_pidc;
    std::atomic<double> _time_diff;
	bool _time_diff_converged;
	uhd::time_spec_t _streamer_start_time;
    void time_diff_send( const uhd::time_spec_t & crimson_now );
    void time_diff_send( const uhd::time_spec_t & crimson_now , int xg_intf);
    bool time_diff_recv( time_diff_resp & tdr );
    bool time_diff_recv( time_diff_resp & tdr, int xg_intf );
    void time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now );
    void fifo_update_process( const time_diff_resp & tdr );

    /**
     * Buffer Management Objects
     */

	// N.B: the _bm_thread is also used for clock domain synchronization
	// N.B: the _bm_iface was removed in favour of using the _time_diff_iface
	std::thread _bm_thread;
	std::atomic<bool> _bm_thread_needed;
	std::atomic<bool> _bm_thread_running;
	std::atomic<bool> _bm_thread_should_exit;;

    time_spec_t _command_time;

	static void bm_thread_fn( cyan_nrnt_impl *dev );

    struct mb_container_type{
        cyan_nrnt_iface::sptr iface;
        std::vector<std::weak_ptr<uhd::rx_streamer> > rx_streamers;
        std::vector<std::weak_ptr<uhd::tx_streamer> > tx_streamers;
        std::vector<uhd::transport::zero_copy_if::sptr> rx_dsp_xports;
        std::vector<uhd::transport::zero_copy_if::sptr> tx_dsp_xports;
        std::vector<uhd::transport::udp_simple::sptr> fifo_ctrl_xports;
        // radio control core sort of like magnesium (maybe plutonium? only if it's organic)
        size_t rx_chan_occ, tx_chan_occ;
        mb_container_type(void): rx_chan_occ(0), tx_chan_occ(0){}
    };
    uhd::dict<std::string, mb_container_type> _mbc;

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

    std::vector<bool> is_tx_sfp_cached;
    std::vector<std::string> tx_sfp_cache;
    std::string get_tx_sfp( size_t chan );
    
    std::vector<bool> is_tx_ip_cached;
    std::vector<std::string> tx_ip_cache;
    std::string get_tx_ip( size_t chan );
    
    std::vector<bool> is_tx_fc_cached;
    std::vector<uint16_t> tx_fc_cache;
    uint16_t get_tx_fc_port( size_t chan );
    
    std::vector<bool> is_tx_udp_port_cached;
    std::vector<uint16_t> tx_udp_port_cache;
    uint16_t get_tx_udp_port( size_t chan );
    
    void get_tx_endpoint( const size_t & chan, std::string & ip_addr, uint16_t & udp_port, std::string & sfp );
    
    int64_t get_tx_buff_scale();

    int get_rx_xg_intf(int channel);

    int get_rx_jesd_num(int channel);
    void set_tx_gain(double gain, const std::string &name, size_t chan);
    double get_tx_gain(const std::string &name, size_t chan);
    void set_rx_gain(double gain, const std::string &name, size_t chan);
    double get_rx_gain(const std::string &name, size_t chan);
    void request_resync_time_diff();

    // Checks it an ip address can be pinged
    void ping_check(std::string sfp, std::string ip);
    // Verifies the MTU is set to the correct value
    void mtu_check(std::string sfp, std::string ip);

    // Samples per second being using per channel
    std::vector<double> tx_sfp_throughput_used;
    // Used to check if a tx channel's rate should be counted towards the max rate check
    std::shared_ptr<std::vector<bool>> tx_channel_in_use;

    // SFP link speed in bits per second
    double link_rate_cache = 0;
    double get_link_rate();

    bool tx_rate_warning_printed = false;
    void tx_rate_check(size_t ch, double rate_samples);
};

}
}

#endif /* INCLUDED_CYAN_NRNT_IMPL_HPP */
