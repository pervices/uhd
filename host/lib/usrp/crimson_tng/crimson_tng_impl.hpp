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
#include <atomic>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/multi_usrp.hpp"

#include "uhd/transport/udp_zero_copy.hpp"

#include "crimson_tng_fw_common.h"
#include <uhdlib/usrp/common/pv_iface.hpp>
#include <uhdlib/usrp/common/clock_sync.hpp>
#include <uhdlib/usrp/common/stream_cmd_issuer.hpp>
#include "crimson_tng_io_impl.hpp"
#include "../../transport/flow_control.hpp"
#include "pidc.hpp"

#include <uhdlib/utils/system_time.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <immintrin.h>

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

}
}

namespace uhd {
namespace usrp {

class crimson_tng_impl : public uhd::device
{
public:
    static constexpr uint_fast8_t NUMBER_OF_XG_CONTROL_INTF = 2;

    // Cache line size
    // Assume it is 64, which is the case for virtually all AMD64 systems
    static constexpr uint_fast8_t CACHE_LINE_SIZE = 64;

    // This is the core constructor to be called when a crimson_tng device is found
    crimson_tng_impl(const uhd::device_addr_t &);
    ~crimson_tng_impl(void);

    static device_addrs_t crimson_tng_find_with_addr(const device_addr_t &hint);
    static device_addrs_t crimson_tng_find(const device_addr_t &hint_);

    // pointers to the streams for the device
    // these functions are defined in crimson_tng_io_impl.cpp
    virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);

    bool recv_async_msg_deprecated_warning = false;
    std::shared_ptr<uhd::transport::bounded_buffer<async_metadata_t>> _async_msg_fifo;
    bool recv_async_msg(uhd::async_metadata_t &, double);

    uhd::device_addr_t device_addr;

    uhd::time_spec_t get_time_now();

    void start_bm();
    void stop_bm();

    void start_pps_dtc();
    void stop_pps_dtc();

private:
    std::string rx_link_root(const size_t channel, const size_t mboard = 0);
    std::string tx_link_root(const size_t channel, const size_t mboard = 0);
    std::string tx_dsp_root(const size_t channel, const size_t mboard = 0);

    // Changing the band results in the gain being reset. These are used to decide if a warning should be printed to let the user know
    bool gain_reset_warning_printed = false;
    std::vector<int> rx_gain_is_set;
    std::vector<int> last_set_rx_band;
    std::vector<int> tx_gain_is_set;
    std::vector<int> last_set_tx_band;

    // wrapper for type <stream_cmd_t> through the SFP ports
    void set_stream_cmd(const std::string pre, uhd::stream_cmd_t data);

    static void detect_pps(crimson_tng_impl *dev);

    void set_command_time(const std::string key, uhd::time_spec_t value);
    void send_gpio_burst_req(const gpio_burst_req& req);
    void set_user_reg(const std::string key, user_reg_t value);

    // set arbitrary crimson properties from dev_addr_t using mappings of the form "crimson:key" => "val"
    void set_properties_from_addr();

    // Mutexes for controlling control (not data) send/receives each SFP port
    std::shared_ptr<std::mutex> _sfp_control_mutex[NUMBER_OF_XG_CONTROL_INTF];

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
    static constexpr size_t padded_pidc_tcl_size = (size_t) ceil(sizeof(uhd::pidc_tl) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    uhd::pidc* const _time_diff_pidc;

    // device_clock_sync_info is the main location used to store clock sync info
    // streamer_clock_sync_info contains the location to copy clock sync info to be shared with streamers
    std::shared_ptr<clock_sync_shared_info> device_clock_sync_info;

	uhd::time_spec_t _streamer_start_time;
    void time_diff_send( const uhd::time_spec_t & crimson_now );
    bool time_diff_recv( time_diff_resp & tdr );
    // Resets the PID controller managing time diffs
    void reset_time_diff_pid();
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
	std::atomic<bool> _bm_thread_should_exit;

    std::thread _pps_thread;
    std::atomic<bool> _pps_thread_needed;
    std::atomic<bool> _pps_thread_running;
    std::atomic<bool> _pps_thread_should_exit;

    time_spec_t _command_time;

    double _max_rate;

    // Tick rate used for Crimson timestamps
    double _master_tick_rate;
    double _tick_period_ns;

    // Minimum valid lo
    double _min_lo;
    double _max_lo;

    double _lo_stepsize;

	static void bm_thread_fn( crimson_tng_impl *dev );

    struct mb_container_type{
        pv_iface::sptr iface;
        // TODO: see if removing rx_streamers and tx_streamers is viable
        std::vector<std::weak_ptr<uhd::usrp::crimson_tng_recv_packet_streamer>> rx_streamers;
        std::vector<std::weak_ptr<uhd::usrp::crimson_tng_send_packet_streamer>> tx_streamers;
        std::vector<uhd::transport::udp_simple::sptr> fifo_ctrl_xports;
    };
    mb_container_type _mbc;

    // Inform the streamer corresponding to a channel what their sample rate is
    void update_rx_samp_rate(const size_t chan, const double rate);
    void update_tx_samp_rate(const size_t chan, const double rate);
    // Inform all streamers what the current sample rate is for their respective channel
    void update_rates(void);
    //update spec methods are coercers until we only accept db_name == A
    void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    double set_tx_dsp_freq(const std::string &, const double);
    uhd::meta_range_t get_tx_dsp_freq_range(const std::string &);
    void update_clock_source(const std::string &, const std::string &);
    void program_stream_dest(uhd::transport::zero_copy_if::sptr &, const uhd::stream_args_t &);

    bool is_high_band( const meta_range_t &dsp_range, const double freq, double bw );

    // Calculate and set frequency
    double choose_lo_shift( double target_freq, double dsp_bw, double user_bw  );
    tune_result_t tune_xx_subdev_and_dsp( const double xx_sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, const tune_request_t &tune_request, int* gain_is_set, int* last_set_band, size_t chan );

    uhd::tune_result_t set_rx_freq(const uhd::tune_request_t &tune_request, size_t chan = 0);
    double get_rx_freq(size_t chan = 0);
    uhd::tune_result_t set_tx_freq(const uhd::tune_request_t &tune_request, size_t chan = 0);
    double get_tx_freq(size_t chan = 0);

    std::string get_tx_sfp( size_t chan );

    std::string get_rx_sfp( size_t chan );

    std::string get_tx_ip( size_t chan );
    uint16_t get_tx_fc_port( size_t chan );
    uint16_t get_tx_udp_port( size_t chan );
    static void get_tx_endpoint( uhd::property_tree::sptr tree, const size_t & chan, std::string & ip_addr, uint16_t & udp_port, std::string & sfp );
    void set_tx_gain(double gain, const std::string &name, size_t chan);
    double get_tx_gain(const std::string &name, size_t chan);
    
    int64_t get_tx_buff_scale();

    void set_rx_gain(double gain, const std::string &name, size_t chan);
    
    double get_rx_gain(const std::string &name, size_t chan);

    // Set/get the sample rates, also updates the streamers to have the new rate
    void set_rx_rate(double rate, size_t chan) override;
    double get_rx_rate(size_t chan) override;
    void set_tx_rate(double rate, size_t chan) override;
    double get_tx_rate(size_t chan) override;

    void set_time_now(const time_spec_t& time_spec, size_t mboard) override;
    void request_resync_time_diff();

    // Checks if an ip address can be pinged
    void ping_check(std::string sfp, std::string ip);
    // Records if an sfp port has already had it's ping check performed
    bool ping_check_completed[NUMBER_OF_XG_CONTROL_INTF]{};

    // Samples per second being using per channel
    std::vector<double> tx_sfp_throughput_used;
    // Used to check if a tx channel's rate should be counted towards the max rate check
    std::shared_ptr<std::vector<bool>> tx_channel_in_use;

    // SFP link speed in bits per second
    double link_rate_cache = 0;
    double get_link_rate();

    bool tx_rate_warning_printed = false;
    void tx_rate_check(size_t ch, double rate_samples);

    // Samples per second being using per channel
    std::vector<double> rx_sfp_throughput_used;
    // Used to check if a rx channel's rate should be counted towards the max rate check
    std::shared_ptr<std::vector<bool>> rx_channel_in_use;
    bool rx_rate_warning_printed = false;
    void rx_rate_check(size_t ch, double rate_samples);

    int64_t ticks_to_nsecs( int64_t tv_tick );
    int64_t nsecs_to_ticks( int64_t tv_nsec );
    void make_time_diff_packet( time_diff_req & pkt, time_spec_t ts );

    bool clock_sync_desired = false;

    // Used to rx start/stop stream commands
    std::vector<stream_cmd_issuer> rx_stream_cmd_issuer;
};

}
}

#endif /* INCLUDED_CRIMSON_TNG_IMPL_HPP */
