//
// Copyright 2014-2015 Per Vices Corporation
// Copyright 2022-2023 Per Vices Corporation
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
#include <atomic>
#include <sys/file.h>
#include <filesystem>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/multi_usrp.hpp"

#include "uhd/transport/udp_zero_copy.hpp"

#include "cyan_nrnt_fw_common.h"
#include "../pv_device/pv_device_impl.hpp"
#include "cyan_nrnt_io_impl.hpp"

namespace uhd {
namespace usrp {

class cyan_nrnt_impl : public pv_device_impl
{
public:
    static constexpr uint_fast8_t NUMBER_OF_XG_CONTROL_INTF = 4;



    // This is the core constructor to be called when a cyan_nrnt device is found
    cyan_nrnt_impl(const uhd::device_addr_t &, const bool use_dpdk, double freq_range_stop = CYAN_NRNT_FREQ_RANGE_STOP);
    ~cyan_nrnt_impl(void);

    static device_addrs_t cyan_nrnt_find_with_addr(const device_addr_t &hint);
    static device_addrs_t cyan_nrnt_find(const device_addr_t &hint_);

    // pointers to the streams for the device
    // these functions are defined in cyan_nrnt_io_impl.cpp
    virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);

    bool recv_async_msg(uhd::async_metadata_t &, double);

private:

    // Flag to indicate the unit is a 3G unit being operated in 1G mode
    int flag_use_3g_as_1g;

    void detect_pps_loop() override;
    std::string get_log_id() const override;
    std::string get_prop_prefix() const override;
    std::vector<stream_cmd_issuer>& get_rx_stream_cmd_issuer() override;
    pv_iface::sptr get_mb_iface() override;


private:

    // Maximum frequency of the highest band
    double _freq_range_stop;

    static void bm_thread_fn( cyan_nrnt_impl *dev );

    struct mb_container_type{
        pv_iface::sptr iface;
        // TODO: see if removing rx_streamers and tx_streamers is viable
        std::vector<std::weak_ptr<uhd::usrp::cyan_nrnt_recv_packet_streamer>> rx_streamers;
        std::vector<std::weak_ptr<uhd::usrp::cyan_nrnt_send_packet_streamer>> tx_streamers;
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

    // Calculate and set frequency
    double choose_lo_shift( double target_freq, int band, property_tree::sptr dsp_subtree, bool is_tx, size_t chan );
    tune_result_t tune_xx_subdev_and_dsp( const double xx_sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, const tune_request_t &tune_request, int* gain_is_set, int* last_set_band, size_t chan );

    uhd::tune_result_t set_rx_freq(const uhd::tune_request_t &tune_request, size_t chan = 0);
    double get_rx_freq(size_t chan = 0);
    uhd::tune_result_t set_tx_freq(const uhd::tune_request_t &tune_request, size_t chan = 0);
    double get_tx_freq(size_t chan = 0);

    std::vector<bool> is_tx_sfp_cached;
    std::vector<std::string> tx_sfp_cache;
    std::string get_tx_sfp( size_t chan );

    std::vector<bool> is_rx_sfp_cached;
    std::vector<std::string> rx_sfp_cache;
    std::string get_rx_sfp( size_t chan );

    std::vector<bool> is_tx_ip_cached;
    std::vector<std::string> tx_ip_cache;
    std::string get_tx_ip( size_t chan );

    std::vector<bool> is_tx_fc_cached;
    std::vector<uint16_t> tx_fc_cache;
    uint16_t get_tx_fc_port( size_t chan );

    std::vector<bool> is_tx_udp_port_cached;
    std::vector<uint16_t> tx_udp_port_cache;
    uint16_t get_tx_udp_port( size_t chan );

    std::vector<bool> is_tx_baseband_only;

    void get_tx_endpoint( const size_t & chan, std::string & ip_addr, uint16_t & udp_port, std::string & sfp );

    int64_t get_tx_buff_scale();

    int get_rx_xg_intf(int channel);

    int get_rx_jesd_num(int channel);
    void set_tx_gain(double gain, const std::string &name, size_t chan);
    double get_tx_gain(const std::string &name, size_t chan);
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

    // Stores the deault rate rx boards operate at so that is doesn't need to be asked every time it is accessed
    std::vector<double> rx_rfe_rate_cache;

    const bool _use_dpdk;
};

}
}

#endif /* INCLUDED_CYAN_NRNT_IMPL_HPP */
