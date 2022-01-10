//
// Copyright 2014 Per Vices Corporation
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

#ifndef INCLUDED_UHD_USRP_MULTI_CRIMSON_TNG_HPP
#define INCLUDED_UHD_USRP_MULTI_CRIMSON_TNG_HPP

#include <uhd/config.hpp>
#include <uhd/device.hpp>
#include <uhd/deprecated.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/types/tune_result.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/dboard_iface.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <complex>
#include <string>
#include <vector>
#include <uhd/stream.hpp>
#include <uhd/usrp/multi_usrp.hpp>

namespace uhd{ namespace usrp{

/*!
 * The Crimson device class:
 *
 * This is the Crimson API interface that is used for Per Vices' Crimson products.
 * It contains all the base class functionality that the Ettus' USRP has, including
 * additional features. The standard Crimson features 4 independent RX chains and
 * 4 independent TX chains. This interface will also allow the use of multiple Crimson
 * units.
 *
 * Remember to map multiple Crimson units accordingly as per notes in multi_usrp.hpp.
 *
 * The notes for each function derived from the base class multi_usrp is defined in
 * the file multi_usrp.hpp. Notes for Crimson specific functions are located here.
 *
 *  A lot of the features that the multi_usrp API offers is not supported in Crimson
 *  hardware because USRP allows the user to purchase different daughter boards and have
 *  multiple mboards as well. However, Crimson is fixed to its single 1 mboard and 2 dboard,
 *  where a dboard has 4 TX chains and the other has 4 RX chains.
 *
 *  Crimson is thus represented through the UHD driver as 1 fixed mboard, and 8 dboard,
 *  where each dboard represents 1 RF chain.
 */

class multi_crimson_tng : public multi_usrp{
public:
    typedef std::shared_ptr<multi_crimson_tng> sptr;

    multi_crimson_tng(const device_addr_t &addr);
    ~multi_crimson_tng(void);
    device::sptr get_device(void);
    dict<std::string, std::string> get_usrp_rx_info(size_t chan = 0);
    dict<std::string, std::string> get_usrp_tx_info(size_t chan = 0);

    /*******************************************************************
     * Mboard methods
     ******************************************************************/
    void set_master_clock_rate(double rate, size_t mboard = 0);
    double get_master_clock_rate(size_t mboard = 0);
    std::string get_pp_string(void);
    std::string get_mboard_name(size_t mboard = 0);
    time_spec_t get_time_now(size_t mboard = 0);
    time_spec_t get_time_last_pps(size_t mboard = 0);
    void set_time_now(const time_spec_t &time_spec, size_t mboard = 0);
    void set_time_next_pps(const time_spec_t &time_spec, size_t mboard = 0);
    void set_time_unknown_pps(const time_spec_t &time_spec);
    bool get_time_synchronized(void);
    void set_command_time(const time_spec_t &time_spec, size_t mboard = 0);
    void clear_command_time(size_t mboard = 0);
    void issue_stream_cmd(const stream_cmd_t &stream_cmd, size_t chan = 0);
    void set_clock_config(const clock_config_t &clock_config, size_t mboard = 0);
    void set_time_source(const std::string &source, const size_t mboard = 0);
    std::string get_time_source(const size_t mboard = 0);
    std::vector<std::string> get_time_sources(const size_t mboard = 0);
    void set_clock_source(const std::string &source, const size_t mboard = 0);
    std::string get_clock_source(const size_t mboard = 0);
    std::vector<std::string> get_clock_sources(const size_t mboard = 0);
    void set_clock_source_out(const bool enb, const size_t mboard = 0);
    void set_time_source_out(const bool enb, const size_t mboard = 0);
    size_t get_num_mboards(void);
    sensor_value_t get_mboard_sensor(const std::string &name, size_t mboard = 0);
    std::vector<std::string> get_mboard_sensor_names(size_t mboard = 0);
    void set_user_register(const boost::uint8_t addr, const boost::uint32_t data, size_t mboard = 0);

    /*******************************************************************
     * RX methods
     ******************************************************************/
    rx_streamer::sptr get_rx_stream(const stream_args_t &args) ;
    void set_rx_subdev_spec(const subdev_spec_t &spec, size_t mboard = 0);
    subdev_spec_t get_rx_subdev_spec(size_t mboard = 0);
    size_t get_rx_num_channels(void);
    std::string get_rx_subdev_name(size_t chan = 0);
    void set_rx_rate(double rate, size_t chan = 0);
    double get_rx_rate(size_t chan = 0);
    meta_range_t get_rx_rates(size_t chan = 0);
    tune_result_t set_rx_freq(const tune_request_t &tune_request, size_t chan = 0);
    double get_rx_freq(size_t chan = 0);
    freq_range_t get_rx_freq_range(size_t chan = 0);
    freq_range_t get_fe_rx_freq_range(size_t chan = 0);
    std::vector<std::string> get_rx_lo_names(size_t chan = 0);
    void set_rx_lo_source(const std::string &src, const std::string &name = ALL_LOS, size_t chan = 0);
    const std::string get_rx_lo_source(const std::string &name = ALL_LOS, size_t chan = 0);
    std::vector<std::string> get_rx_lo_sources(const std::string &name = ALL_LOS, size_t chan = 0);
    void set_rx_lo_export_enabled(bool enabled, const std::string &name = ALL_LOS, size_t chan = 0);
    bool get_rx_lo_export_enabled(const std::string &name = ALL_LOS, size_t chan = 0);
    double set_rx_lo_freq(double freq, const std::string &name, size_t chan = 0);
    double get_rx_lo_freq(const std::string &name, size_t chan = 0);
    freq_range_t get_rx_lo_freq_range(const std::string &name, size_t chan = 0);
    void set_rx_gain(double gain, const std::string &name, size_t chan = 0);
    void set_normalized_rx_gain(double gain, size_t chan = 0);
    void set_rx_agc(bool enable, size_t chan = 0);
    void set_rx_gain(double gain, size_t chan = 0){
        return this->set_rx_gain(gain, "", chan);
    }
    double get_rx_gain(const std::string &name, size_t chan = 0);
    double get_normalized_rx_gain(size_t chan = 0);
    double get_rx_gain(size_t chan = 0){
        return this->get_rx_gain("", chan);
    }
    gain_range_t get_rx_gain_range(const std::string &name, size_t chan = 0);
    gain_range_t get_rx_gain_range(size_t chan = 0){
        return this->get_rx_gain_range("", chan);
    }
    std::vector<std::string> get_rx_gain_names(size_t chan = 0);
    void set_rx_antenna(const std::string &ant, size_t chan = 0);
    std::string get_rx_antenna(size_t chan = 0);
    std::vector<std::string> get_rx_antennas(size_t chan = 0);
    void set_rx_bandwidth(double bandwidth, size_t chan = 0);
    double get_rx_bandwidth(size_t chan = 0);
    meta_range_t get_rx_bandwidth_range(size_t chan = 0);
    dboard_iface::sptr get_rx_dboard_iface(size_t chan = 0);
    sensor_value_t get_rx_sensor(const std::string &name, size_t chan = 0);
    std::vector<std::string> get_rx_sensor_names(size_t chan = 0);
    void set_rx_dc_offset(const bool enb, size_t chan = 0);
    void set_rx_dc_offset(const std::complex<double> &offset, size_t chan = 0);
    void set_rx_iq_balance(const bool enb, size_t chan);
    void set_rx_iq_balance(const std::complex<double> &offset, size_t chan = 0);
    std::vector<std::string> get_rx_gain_profile_names(const size_t chan = 0);
    void set_rx_gain_profile(const std::string& profile, const size_t chan = 0);
    std::string get_rx_gain_profile(const size_t chan = 0);


    /*******************************************************************
     * TX methods
     ******************************************************************/
    tx_streamer::sptr get_tx_stream(const stream_args_t &args) ;
    void set_tx_subdev_spec(const subdev_spec_t &spec, size_t mboard = 0);
    subdev_spec_t get_tx_subdev_spec(size_t mboard = 0);
    size_t get_tx_num_channels(void);
    std::string get_tx_subdev_name(size_t chan = 0);
    void set_tx_rate(double rate, size_t chan = 0);
    double get_tx_rate(size_t chan = 0);
    meta_range_t get_tx_rates(size_t chan = 0);
    tune_result_t set_tx_freq(const tune_request_t &tune_request, size_t chan = 0);
    double get_tx_freq(size_t chan = 0);
    freq_range_t get_tx_freq_range(size_t chan = 0);
    freq_range_t get_fe_tx_freq_range(size_t chan = 0);
    void set_tx_gain(double gain, const std::string &name, size_t chan = 0);
    void set_normalized_tx_gain(double gain, size_t chan = 0);
    void set_tx_gain(double gain, size_t chan = 0){
        return this->set_tx_gain(gain, "", chan);
    }
    double get_tx_gain(const std::string &name, size_t chan = 0);
    double get_tx_gain(size_t chan = 0){
        return this->get_tx_gain("", chan);
    }
    double get_normalized_tx_gain(size_t chan = 0);
    gain_range_t get_tx_gain_range(const std::string &name, size_t chan = 0);
    gain_range_t get_tx_gain_range(size_t chan = 0){
        return this->get_tx_gain_range("", chan);
    }
    std::vector<std::string> get_tx_gain_names(size_t chan = 0);
    void set_tx_antenna(const std::string &ant, size_t chan = 0);
    std::string get_tx_antenna(size_t chan = 0);
    std::vector<std::string> get_tx_antennas(size_t chan = 0);
    void set_tx_bandwidth(double bandwidth, size_t chan = 0);
    double get_tx_bandwidth(size_t chan = 0);
    meta_range_t get_tx_bandwidth_range(size_t chan = 0);
    dboard_iface::sptr get_tx_dboard_iface(size_t chan = 0);
    sensor_value_t get_tx_sensor(const std::string &name, size_t chan = 0);
    std::vector<std::string> get_tx_sensor_names(size_t chan = 0);
    void set_tx_dc_offset(const std::complex<double> &offset, size_t chan = 0);
    void set_tx_iq_balance(const std::complex<double> &offset, size_t chan = 0);
    std::vector<std::string> get_tx_lo_names(size_t chan = 0);
    void set_tx_lo_source(
            const std::string &src,
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    );
    const std::string get_tx_lo_source(
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    );
    std::vector<std::string> get_tx_lo_sources(
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    );
    void set_tx_lo_export_enabled(
            const bool enabled,
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    );
    bool get_tx_lo_export_enabled(
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    );
    double set_tx_lo_freq(
            const double freq,
            const std::string &name,
            const size_t chan=0
    );
    double get_tx_lo_freq(
            const std::string &name,
            const size_t chan=0
    );
    freq_range_t get_tx_lo_freq_range(
            const std::string &name,
            const size_t chan=0
    );
    std::vector<std::string> get_tx_gain_profile_names(const size_t chan = 0);
    void set_tx_gain_profile(const std::string& profile, const size_t chan = 0);
    std::string get_tx_gain_profile(const size_t chan = 0);

    /*******************************************************************
     * GPIO methods
     ******************************************************************/
    void set_gpio_attr(const std::string &bank, const std::string &attr, const std::string &value, const uint32_t mask = 0xffffffff, const size_t mboard = 0);
    uint32_t get_gpio_attr(const std::string &bank, const std::string &attr, const size_t mboard = 0);
    std::vector<std::string> get_gpio_string_attr(const std::string &bank, const std::string &attr, const size_t mboard = 0);
    // not supported on Crimson
    std::vector<std::string> get_gpio_banks(const size_t mboard = 0);
    // not supported on Crimson
    void set_gpio_attr(const std::string &bank, const std::string &attr, const boost::uint32_t value, const boost::uint32_t mask = 0xffffffff, const size_t mboard = 0);
    // no supported on Crimson

    /*******************************************************************
     * Register IO methods
     ******************************************************************/
    std::vector<std::string> enumerate_registers(const size_t mboard = 0);
    void write_register(const std::string &path, const uint32_t field, const uint64_t value, const size_t mboard = 0);
    uint64_t read_register(const std::string &path, const uint32_t field, const size_t mboard = 0);

    /*******************************************************************
     * Filter API methods
     ******************************************************************/

    std::vector<std::string> get_filter_names(const std::string &search_mask = "");
    filter_info_base::sptr get_filter(const std::string &path);
    void set_filter(const std::string &path, filter_info_base::sptr filter);

    /*******************************************************************
     * Crimson methods
     ******************************************************************/

    static tune_result_t tune_lo_and_dsp( const double xx_sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, const tune_request_t &tune_request );
    static double choose_dsp_nco_shift( double target_freq, property_tree::sptr dsp_subtree );

private:
    // Pointer to the Crimson device
    device::sptr _dev;

    // Crimson does not support a tree/file-system structure on the SoC for properties
    property_tree::sptr _tree;

    // pointer to the streamer that is being used to send commands
    tx_streamer::sptr _stream;

    // get the string representation of the channel: 1 -> "chan1"
    std::string chan_to_string(size_t chan);
    // channel: 1 -> A, 2 -> B....
    std::string chan_to_alph(size_t chan);

    // get the root path
    fs_path mb_root(const size_t mboard);
    fs_path rx_rf_fe_root(const size_t chan);
    fs_path rx_dsp_root(const size_t chan);
    fs_path rx_link_root(const size_t chan);
    fs_path tx_rf_fe_root(const size_t chan);
    fs_path tx_dsp_root(const size_t chan);
    fs_path tx_link_root(const size_t chan);
    fs_path cm_root();
};

}}
#endif
