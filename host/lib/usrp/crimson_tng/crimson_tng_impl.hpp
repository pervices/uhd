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
#include <sys/file.h>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/multi_usrp.hpp"

#include "uhd/transport/udp_zero_copy.hpp"

#include "crimson_tng_fw_common.h"
#include "../pv_device/pv_device_impl.hpp"
#include "crimson_tng_io_impl.hpp"
#include "../../transport/flow_control.hpp"

namespace uhd {
namespace usrp {

class crimson_tng_impl : public pv_device_impl
{
public:
    static constexpr uint_fast8_t NUMBER_OF_XG_CONTROL_INTF = 2;



    // This is the core constructor to be called when a crimson_tng device is found
    crimson_tng_impl(const uhd::device_addr_t &);
    ~crimson_tng_impl(void);

    static device_addrs_t crimson_tng_find_with_addr(const device_addr_t &hint);
    static device_addrs_t crimson_tng_find(const device_addr_t &hint_);

    // pointers to the streams for the device
    // these functions are defined in crimson_tng_io_impl.cpp
    virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);

    bool recv_async_msg(uhd::async_metadata_t &, double);

private:
    std::string product_name_c;

    void detect_pps_loop() override;
    std::string get_log_id() const override;
    std::string get_prop_prefix() const override;
    std::vector<stream_cmd_issuer>& get_rx_stream_cmd_issuer() override;
    pv_iface::sptr get_mb_iface() override;


private:

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
    double choose_lo_shift( double target_freq, double dsp_bw, double user_bw, bool is_tx  );
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
    void set_time_initiated(int64_t planned_time_s) override;
    void set_time_finished() override;

    // Checks if an ip address can be pinged
    // Return true if ping succeeded
    bool ping_check(std::string sfp, std::string ip);
    // Prevents multiple threads from attempting the ping check at once
    std::mutex ping_mutex;
    // Records if an sfp port has already had it's ping check performed
    uint8_t ping_check_completed[NUMBER_OF_XG_CONTROL_INTF] ={0};
    // Records if ping succeeded
    uint8_t sfp_working[NUMBER_OF_XG_CONTROL_INTF] ={0};

    double get_link_rate();
    void tx_rate_check(size_t ch, double rate_samples);
    void rx_rate_check(size_t ch, double rate_samples);

    // Full tx is a mode where tx channels C and D operate at full rate, but rx channels C and D are disabled
    // Update the error message in get_rx_stream if the number of rx channels ever changes
    bool is_full_tx = false;
};

}
}

#endif /* INCLUDED_CRIMSON_TNG_IMPL_HPP */
