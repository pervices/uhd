//
// Copyright 2010-2016 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/convert.hpp>
#include <uhd/exception.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/eeprom.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/usrp/dboard_id.hpp>
#include <uhd/usrp/gpio_defs.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/gain_group.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/soft_register.hpp>
#include <uhdlib/rfnoc/rfnoc_device.hpp>
#include <uhdlib/usrp/gpio_defs.hpp>
#include <uhdlib/usrp/multi_usrp_utils.hpp>
#include <uhdlib/types/trigger_options_helpers.hpp>
#include <uhdlib/usrp/pv_device/pv_device.hpp>
#include <uhdlib/usrp/pv_device/pv_device_fw_common.hpp>
#include <boost/algorithm/string.hpp>
#include <format>
#include <algorithm>
#include <bitset>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <concepts>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <assert.h>

#include <iostream>

namespace uhd { namespace rfnoc {

//! Factory function for RFNoC devices specifically
uhd::usrp::multi_usrp::sptr make_rfnoc_device(
    uhd::rfnoc::detail::rfnoc_device::sptr rfnoc_device,
    const uhd::device_addr_t& dev_addr);

}} /* namespace uhd::rfnoc */

using namespace uhd;
using namespace uhd::usrp;

const size_t multi_usrp::ALL_MBOARDS    = size_t(~0);
const size_t multi_usrp::ALL_CHANS      = size_t(~0);
const std::string multi_usrp::ALL_GAINS = "";
const std::string multi_usrp::ALL_LOS   = "all";

UHD_INLINE std::string string_vector_to_string(
    std::vector<std::string> values, std::string delimiter = std::string(" "))
{
    std::string out = "";
    for (std::vector<std::string>::iterator iter = values.begin(); iter != values.end();
         iter++) {
        out += (iter != values.begin() ? delimiter : "") + *iter;
    }
    return out;
}

#define THROW_GAIN_NAME_ERROR(name, chan, dir)                                   \
    throw uhd::exception::runtime_error(                                         \
        std::format(                                                          \
             "{}: gain \"{}\" not found for channel {}.\nAvailable gains: {}\n", \
            UHD_FUNCTION, name, chan,                                         \
            string_vector_to_string(get_##dir##_gain_names(chan))));

/***********************************************************************
 * Helper methods
 **********************************************************************/

static void freq_error_check(
    double target_freq, double actual_freq, const std::string& xx)
{
    // Threshold to print a warning message
    static constexpr double warn_threshold = 1000; // 1kHz
    static constexpr double error_threshold = 250e6; // 100MHz

    const double diff = std::abs(target_freq - actual_freq);

    // The different between the actual and is extreme, through an error
    // Technically this is different behaviour than upstream, but if someone encounters this they are doing something wrong
    if(diff > error_threshold) {
        std::string message = std::format(
                   "Extreme difference between target and actual when setting {} center frequency:\n"
                   "Target center frequency: {} MHz\n"
                   "Actual center frequency: {} MHz\n",
                   xx, (target_freq / 1e6), (actual_freq / 1e6));
        UHD_LOG_ERROR("MULTI_USRP", message)
        throw uhd::runtime_error(message);

    } else if (diff > warn_threshold) {
        UHD_LOGGER_WARNING("MULTI_USRP")
            << std::format(
                   "Error while attempting to set the requested {} center frequency:\n"
                   "Target center frequency: {} MHz\n"
                   "Actual center frequency: {} MHz\n",
                   xx, (target_freq / 1e6), (actual_freq / 1e6));
    }
}

/***********************************************************************
 * Trigger helper functions
 **********************************************************************/

/***********************************************************************
 * Gain helper functions
 **********************************************************************/
static double get_gain_value(property_tree::sptr subtree){
    return subtree->access<double>("value").get();
}

static void set_gain_value(property_tree::sptr subtree, const double gain){
    subtree->access<double>("value").set(gain);
}

static meta_range_t get_gain_range(property_tree::sptr subtree){
    return subtree->access<meta_range_t>("range").get();
}

static gain_fcns_t make_gain_fcns_from_subtree(property_tree::sptr subtree){
    gain_fcns_t gain_fcns;
    gain_fcns.get_range = std::bind(&get_gain_range, subtree);
    gain_fcns.get_value = std::bind(&get_gain_value, subtree);
    gain_fcns.set_value = std::bind(&set_gain_value, subtree, std::placeholders::_1);
    return gain_fcns;
}

/***********************************************************************
 * Tune Helper Functions
 **********************************************************************/
static const double RX_SIGN = +1.0;
static const double TX_SIGN = -1.0;

/***********************************************************************
 * Multi USRP Implementation
 **********************************************************************/
class multi_usrp_impl : public multi_usrp
{
public:
    multi_usrp_impl(device::sptr dev) : _dev(dev)
    {
        _tree = _dev->get_tree();
    }

    device::sptr get_device(void) override
    {
        return _dev;
    }

    uhd::property_tree::sptr get_tree() const override
    {
        return _tree;
    }

    std::vector<std::string> get_filter_names(const std::string &search_mask)
    {
        std::vector<std::string> names;
        //the odd and inefficient structure of this code is to keep its behaviour as close as possible to the original
        for(size_t n = 0; n <get_rx_num_channels(); n++) {
            std::vector<std::string> ch_names = get_rx_filter_names(n);
            for(size_t i = 0; i < ch_names.size(); i++) {
                if((search_mask.empty()) || ch_names[i].find(search_mask) != std::string::npos) {
                    names.push_back(ch_names[i]);
                }
            }
        }
        for(size_t n = 0; n <get_tx_num_channels(); n++) {
            std::vector<std::string> ch_names = get_tx_filter_names(n);
            for(size_t i = 0; i < ch_names.size(); i++) {
                if((search_mask.empty()) || ch_names[i].find(search_mask) != std::string::npos) {
                    names.push_back(ch_names[i]);
                }
            }
        }
        return names;
    }

    filter_info_base::sptr get_filter(const std::string &path)
    {
        // Okay to ignore deprecated warning here since this function is itself deprecated
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        std::vector<std::string> possible_names = get_filter_names("");
        #pragma GCC diagnostic pop
        std::vector<std::string>::iterator it;
        it = find(possible_names.begin(), possible_names.end(), path);
        if (it == possible_names.end()) {
            throw uhd::runtime_error("Attempting to get non-existing filter: "+path);
        }

        return _tree->access<filter_info_base::sptr>(path / "value").get();
    }

    void set_filter(const std::string &path, filter_info_base::sptr filter)
    {
        // Okay to ignore deprecated warning here since this function is itself deprecated
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        std::vector<std::string> possible_names = get_filter_names("");
        #pragma GCC diagnostic pop
        std::vector<std::string>::iterator it;
        it = find(possible_names.begin(), possible_names.end(), path);
        if (it == possible_names.end()) {
            throw uhd::runtime_error("Attempting to set non-existing filter: "+path);
        }

        _tree->access<filter_info_base::sptr>(path / "value").set(filter);
    }

    dict<std::string, std::string> get_usrp_rx_info(size_t chan){

        mboard_chan_pair mcp = rx_chan_to_mcp(chan);

        dict<std::string, std::string> usrp_info;
        const auto mb_eeprom =
            _tree->access<mboard_eeprom_t>(mb_root(mcp.mboard) / "eeprom").get();
        usrp_info["module_serial"] =
            mb_eeprom.get("module_serial", mb_eeprom.get("serial", "n/a"));
        usrp_info["mboard_id"] =
            _tree->access<std::string>(mb_root(mcp.mboard) / "name").get();
        usrp_info["mboard_name"]   = mb_eeprom.get("name", "n/a");
        usrp_info["mboard_serial"] = mb_eeprom["serial"];
        usrp_info["rx_subdev_name"] =
            _tree->access<std::string>(rx_rf_fe_root(chan) / "name").get();
        usrp_info["rx_subdev_spec"] =
            _tree->access<subdev_spec_t>(mb_root(mcp.mboard) / "rx_subdev_spec")
                .get()
                .to_string();
        usrp_info["rx_antenna"] =
            _tree->access<std::string>(rx_rf_fe_root(chan) / "antenna" / "value").get();
        if (_tree->exists(
                rx_rf_fe_root(chan).branch_path().branch_path() / "rx_eeprom")) {
            const auto db_eeprom =
                _tree
                    ->access<dboard_eeprom_t>(
                        rx_rf_fe_root(chan).branch_path().branch_path() / "rx_eeprom")
                    .get();
            usrp_info["rx_serial"] = db_eeprom.serial;
            usrp_info["rx_id"]     = db_eeprom.id.to_pp_string();
        }
        if (_tree->exists(rx_rf_fe_root(chan) / "ref_power/key")) {
            usrp_info["rx_ref_power_key"] =
                _tree->access<std::string>(rx_rf_fe_root(chan) / "ref_power/key").get();
        }
        if (_tree->exists(rx_rf_fe_root(chan) / "ref_power/serial")) {
            usrp_info["rx_ref_power_serial"] =
                _tree->access<std::string>(rx_rf_fe_root(chan) / "ref_power/serial")
                    .get();
        }
        return usrp_info;
    }

    dict<std::string, std::string> get_usrp_tx_info(size_t chan){
        mboard_chan_pair mcp = tx_chan_to_mcp(chan);
        dict<std::string, std::string> usrp_info;
        const auto mb_eeprom =
            _tree->access<mboard_eeprom_t>(mb_root(mcp.mboard) / "eeprom").get();
        usrp_info["module_serial"] =
            mb_eeprom.get("module_serial", mb_eeprom.get("serial", "n/a"));
        usrp_info["mboard_id"] =
            _tree->access<std::string>(mb_root(mcp.mboard) / "name").get();
        usrp_info["mboard_name"]   = mb_eeprom.get("name", "n/a");
        usrp_info["mboard_serial"] = mb_eeprom["serial"];
        usrp_info["tx_subdev_name"] =
            _tree->access<std::string>(tx_rf_fe_root(chan) / "name").get();
        usrp_info["tx_subdev_spec"] =
            _tree->access<subdev_spec_t>(mb_root(mcp.mboard) / "tx_subdev_spec")
                .get()
                .to_string();
        usrp_info["tx_antenna"] =
            _tree->access<std::string>(tx_rf_fe_root(chan) / "antenna" / "value").get();
        if (_tree->exists(
                tx_rf_fe_root(chan).branch_path().branch_path() / "tx_eeprom")) {
            const auto db_eeprom =
                _tree
                    ->access<dboard_eeprom_t>(
                        tx_rf_fe_root(chan).branch_path().branch_path() / "tx_eeprom")
                    .get();
            usrp_info["tx_serial"] = db_eeprom.serial;
            usrp_info["tx_id"]     = db_eeprom.id.to_pp_string();
        }
        if (_tree->exists(tx_rf_fe_root(chan) / "ref_power/key")) {
            usrp_info["tx_ref_power_key"] =
                _tree->access<std::string>(tx_rf_fe_root(chan) / "ref_power/key").get();
        }
        if (_tree->exists(tx_rf_fe_root(chan) / "ref_power/serial")) {
            usrp_info["tx_ref_power_serial"] =
                _tree->access<std::string>(tx_rf_fe_root(chan) / "ref_power/serial")
                    .get();
        }
        return usrp_info;
    }

    /*******************************************************************
     * Mboard methods
     ******************************************************************/
    void set_master_clock_rate(double rate, size_t mboard){
        if (mboard != ALL_MBOARDS) {
            if (_tree->exists(mb_root(mboard) / "auto_tick_rate")
                and _tree->access<bool>(mb_root(mboard) / "auto_tick_rate").get()) {
                _tree->access<bool>(mb_root(mboard) / "auto_tick_rate").set(false);
                UHD_LOGGER_INFO("MULTI_USRP")
                    << "Setting master clock rate selection to 'manual'.";
            }
            _tree->access<double>(mb_root(mboard) / "tick_rate").set(rate);
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_master_clock_rate(rate, m);
        }
    }

    double get_master_clock_rate(size_t mboard){
        return _tree->access<double>(mb_root(mboard) / "tick_rate").get();
    }

    meta_range_t get_master_clock_rate_range(const size_t mboard) override
    {
        if (_tree->exists(mb_root(mboard) / "tick_rate/range")) {
            return _tree->access<meta_range_t>(
                mb_root(mboard) / "tick_rate/range"
            ).get();
        }
        // The USRP may not have a range defined, in which case we create a
        // fake range with a single value:
        const double tick_rate = get_master_clock_rate(mboard);
        return meta_range_t(
            tick_rate,
            tick_rate,
            0
        );
    }

    std::string get_pp_string(void) override
    {
        std::string buff = std::format("{} USRP:\n"
                                             "  Device: {}\n",
                               ((get_num_mboards() > 1) ? "Multi" : "Single"),
                               (_tree->access<std::string>("/name").get()));
        for (size_t m = 0; m < get_num_mboards(); m++) {
            buff += std::format("  Mboard {}: {}\n", m,
                        (_tree->access<std::string>(mb_root(m) / "name").get()));
        }

        //----------- rx side of life ----------------------------------
        for (size_t m = 0, chan = 0; m < get_num_mboards(); m++) {
            for (; chan < (m + 1) * get_rx_subdev_spec(m).size(); chan++) {
                buff +=
                    std::format("  RX Channel: {}\n"
                                  "    RX DSP: {}\n"
                                  "    RX Dboard: {}\n"
                                  "    RX Subdev: {}\n",
                    chan, rx_dsp_root(chan).leaf(),
                    rx_rf_fe_root(chan).branch_path().branch_path().leaf(),
                    (_tree->access<std::string>(rx_rf_fe_root(chan) / "name").get()));
            }
        }

        //----------- tx side of life ----------------------------------
        for (size_t m = 0, chan = 0; m < get_num_mboards(); m++) {
            for (; chan < (m + 1) * get_tx_subdev_spec(m).size(); chan++) {
                buff +=
                    std::format("  TX Channel: {}\n"
                                  "    TX DSP: {}\n"
                                  "    TX Dboard: {}\n"
                                  "    TX Subdev: {}\n",
                    chan, tx_dsp_root(chan).leaf(),
                    tx_rf_fe_root(chan).branch_path().branch_path().leaf(),
                    (_tree->access<std::string>(tx_rf_fe_root(chan) / "name").get()));
            }
        }

        return buff;
    }

    std::string get_mboard_name(size_t mboard) override
    {
        return _tree->access<std::string>(mb_root(mboard) / "name").get();
    }

    // TODO see if this should get the current time instead
    // Currently this only checks the
    time_spec_t get_time_now(size_t mboard = 0) override
    {
        (void) mboard;
        return get_device()->get_time_now();
    }

    // The time seconds time increments on pps, to the last pps pulse occured on the last full second
    time_spec_t get_time_last_pps(size_t mboard = 0) override
    {
        _tree->access<std::string>(mb_root(mboard) / "gps_time").set("1");
        int64_t secs = std::strtol(_tree->access<std::string>(mb_root(mboard) / "gps_time").get().c_str(), nullptr, 10);

        return time_spec_t(secs, 0);
    }

    void set_time_now(const time_spec_t& time_spec, size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            this->get_device()->set_time_initiated(time_spec.get_full_secs());
            this->get_device()->set_time_now(time_spec, mboard);
            this->get_device()->set_time_finished();
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_time_now(time_spec, m);
        }
    }

    void set_time_next_pps(const time_spec_t& time_spec, size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            this->get_device()->set_time_initiated(time_spec.get_full_secs());
            _tree->access<time_spec_t>(mb_root(mboard) / "time/pps").set(time_spec);
            this->get_device()->set_time_finished();
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_time_next_pps(time_spec, m);
        }
    }

    void set_time_unknown_pps(const time_spec_t& time_spec) override
    {
        // Gets the time of the last pps
        time_spec_t time_start_last_pps = get_time_last_pps();
        auto end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(1100);

        // Waits for next pps
        while (time_start_last_pps == get_time_last_pps(0)) {
            if (std::chrono::steady_clock::now() > end_time) {
                throw uhd::runtime_error("Board 0 may not be getting a PPS signal!\n"
                                         "No PPS detected within the time interval.\n"
                                         "See the application notes for your device.\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Sets time at next pps. Done after waiting for previous pps to ensure all channels are set within the same pps cycle
        set_time_next_pps(time_spec, ALL_MBOARDS);

        // Waits for next pps
        time_start_last_pps = get_time_last_pps(0);
        end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(1100);
        while (time_start_last_pps == get_time_last_pps(0)) {
            if (std::chrono::steady_clock::now() > end_time) {
                throw uhd::runtime_error("Board 0 may not be getting a PPS signal!\n"
                                         "No PPS detected within the time interval.\n"
                                         "See the application notes for your device.\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Print warning if mboards are out out sync
        for (size_t m = 1; m < get_num_mboards(); m++) {
            time_spec_t time_0 = this->get_time_now(0);
            time_spec_t time_i = this->get_time_now(m);
            if (time_i < time_0
                or (time_i - time_0)
                       > time_spec_t(0.01)) { // 10 ms: greater than time to issue the get time request but not too big
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << std::format(
                           "Detected time deviation between board {} and board 0.\n"
                           "Board 0 time is {} seconds.\n"
                           "Board {} time is {} seconds.\n",
                           m, time_0.get_real_secs(), m, time_i.get_real_secs());
            }
        }
    }

    bool get_time_synchronized(void) override
    {
        for (size_t m = 1; m < get_num_mboards(); m++) {
            time_spec_t time_0 = this->get_time_now(0);
            time_spec_t time_i = this->get_time_now(m);
            if (time_i < time_0 or (time_i - time_0) > time_spec_t(0.01))
                return false;
        }
        return true;
    }

    void set_command_time(const time_spec_t &time_spec, size_t mboard){
        if (mboard != ALL_MBOARDS) {
            if (not _tree->exists(mb_root(mboard) / "time/cmd")) {
                throw uhd::not_implemented_error(
                    "timed command feature not implemented on this hardware");
            }
            _tree->access<time_spec_t>(mb_root(mboard) / "time/cmd").set(time_spec);
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_command_time(time_spec, m);
        }
    }

    void clear_command_time(size_t mboard){
        if (mboard != ALL_MBOARDS) {
            _tree->access<time_spec_t>(mb_root(mboard) / "time/cmd")
                .set(time_spec_t(0.0));
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            clear_command_time(m);
        }
    }

    void issue_stream_cmd(const stream_cmd_t &stream_cmd, size_t chan){
        if (chan != ALL_CHANS) {
            _tree->access<stream_cmd_t>(rx_dsp_root(chan) / "stream_cmd").set(stream_cmd);
            return;
        }
        for (size_t c = 0; c < get_rx_num_channels(); c++) {
            issue_stream_cmd(stream_cmd, c);
        }
    }


    void set_time_source(const std::string& source, const size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            _tree->access<std::string>(mb_root(mboard) / "time_source" / "value").set(source);
            std::string actual = _tree->access<std::string>(mb_root(mboard) / "time_source" / "value").get();

            if(actual == source) {
                // Success, no further action required
                return;
            }
            else if(actual == "internal" && source == "external") {
                UHD_LOG_ERROR("MULTI_USRP", "External PSS not detected, fall back to internal PPS triggered. "
                    "Catch std::runtime_error if you are okay with falling back to internal PPS");
                throw std::runtime_error("External PPS not detected");
            }
            else if(actual.starts_with("Invalid time source")) {

                // Get the list of valid sources and convert it to a string from a vector of strings
                std::vector<std::string> sources_list = get_time_sources(mboard);
                std::string sources_string;
                if(sources_list.size() > 0) {
                    sources_string = sources_list[0];
                }
                for(size_t i = 1; i < sources_list.size(); i++) {
                    sources_string += ", " + sources_list[i];
                }

                UHD_LOG_ERROR("MULTI_USRP",
                    std::format("Invalid time source provided: \"{}\"\n"
                        "Valid time sources are: {}",
                        source, sources_string
                    )
                );

                throw std::invalid_argument("Invalid time source provided: " + source);

            } else {
                UHD_LOG_ERROR("MULTI_USRP", "Unknown error when setting PPS source")
                throw std::runtime_error("Unknown error when setting PPS source");
            }
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            this->set_time_source(source, m);
        }
    }

    std::string get_time_source(const size_t mboard) override
    {

        const auto time_source_path = mb_root(mboard) / "time_source/value";
        if (_tree->exists(time_source_path)) {
            return _tree->access<std::string>(time_source_path).get();
        } else if (_tree->exists(mb_root(mboard) / "sync_source/value")) {
            auto sync_source =
                _tree->access<device_addr_t>(mb_root(mboard) / "sync_source" / "value")
                    .get();
            if (sync_source.has_key("time_source")) {
                return sync_source.get("time_source");
            }
        }
        throw uhd::runtime_error("Cannot query time_source on this device!");
    }

    std::vector<std::string> get_time_sources(const size_t mboard) override
    {
        const auto time_source_path = mb_root(mboard) / "time_source/options";
        if (_tree->exists(time_source_path)) {
            return _tree->access<std::vector<std::string>>(time_source_path).get();
        } else if (_tree->exists(mb_root(mboard) / "sync_source/options")) {
            const auto sync_sources = get_sync_sources(mboard);
            std::vector<std::string> time_sources;
            for (const auto& sync_source : sync_sources) {
                if (sync_source.has_key("time_source")) {
                    time_sources.push_back(sync_source.get("time_source"));
                }
            }
        }
        throw uhd::runtime_error("Cannot query time_source on this device!");
    }

    void set_clock_source(const std::string& source, const size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            const auto clock_source_path = mb_root(mboard) / "clock_source/value";
            const auto sync_source_path  = mb_root(mboard) / "sync_source/value";
            std::string result;

            if (_tree->exists(clock_source_path)) {
                _tree->access<std::string>(clock_source_path).set(source);
                result = _tree->access<std::string>(clock_source_path).get();
            } else if (_tree->exists(sync_source_path)) {
                auto sync_source = _tree->access<device_addr_t>(sync_source_path).get();
                sync_source["clock_source"] = source;
                _tree->access<device_addr_t>(sync_source_path).set(sync_source);
                result = sync_source.get("time_source");
            } else {
                throw uhd::runtime_error("Can't set clock source on this device.");
            }

            if(result == "Error Unlocked PLL with External Reference") {
                UHD_LOGGER_ERROR("MULTI_USRP")  << "PLL unlocked while using external reference. Verify if the external reference is connected";
            } else if(result.substr(0, source.size()) != source) {
                UHD_LOGGER_ERROR("MULTI_USRP")  << std::format( "Unable to set clock source. The program attempted to set it to {} but it returned: {}", source, result);
            }
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            this->set_clock_source(source, m);
        }
    }

    std::string get_clock_source(const size_t mboard) override
    {
        const auto clock_source_path = mb_root(mboard) / "clock_source/value";
        if (_tree->exists(clock_source_path)) {
            return _tree->access<std::string>(mb_root(mboard) / "clock_source" / "value")
                .get();
        } else if (_tree->exists(mb_root(mboard) / "sync_source/value")) {
            auto sync_source =
                _tree->access<device_addr_t>(mb_root(mboard) / "sync_source" / "value")
                    .get();
            if (sync_source.has_key("clock_source")) {
                return sync_source.get("clock_source");
            }
        }
        throw uhd::runtime_error("Cannot query clock_source on this device!");
    }

    // Set the expected frequency of an external reference clock
    void set_clock_reference_freq(int freq) override
    {
        try {
            _tree->access<int>("/mboards/0/time_source/freq").set(freq);
        } catch (uhd::lookup_error& e) {
            UHD_LOGGER_ERROR("MULTI_USRP") << "This device does not support changing the external reference clock's frequency" << std::endl;
        }
    }

    void set_sync_source(const std::string& clock_source,
        const std::string& time_source,
        const size_t mboard) override
    {
        device_addr_t sync_args;
        sync_args["clock_source"] = clock_source;
        sync_args["time_source"]  = time_source;
        set_sync_source(sync_args, mboard);
    }

    void set_sync_source(const device_addr_t& sync_source, const size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            const auto sync_source_path = mb_root(mboard) / "sync_source/value";
            if (_tree->exists(sync_source_path)) {
                _tree->access<device_addr_t>(sync_source_path).set(sync_source);
            } else if (_tree->exists(mb_root(mboard) / "clock_source/value")
                       and _tree->exists(mb_root(mboard) / "time_source/value")
                       and sync_source.has_key("clock_source")
                       and sync_source.has_key("time_source")) {
                const std::string clock_source = sync_source["clock_source"];
                const std::string time_source  = sync_source["time_source"];
                set_clock_source(clock_source, mboard);
                set_time_source(time_source, mboard);
            } else {
                throw uhd::runtime_error("Can't set sync source on this device.");
            }
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            this->set_sync_source(sync_source, m);
        }

    }

    device_addr_t get_sync_source(const size_t mboard) override
    {
        const auto sync_source_path = mb_root(mboard) / "sync_source/value";
        if (_tree->exists(sync_source_path)) {
            return _tree->access<device_addr_t>(sync_source_path).get();
        }
        // If this path is not there, we fall back to the oldschool method and
        // convert to a new-fangled sync source dictionary
        const std::string clock_source = get_clock_source(mboard);
        const std::string time_source  = get_time_source(mboard);
        device_addr_t sync_source;
        sync_source["clock_source"] = clock_source;
        sync_source["time_source"]  = time_source;
        return sync_source;
    }

    std::vector<device_addr_t> get_sync_sources(const size_t mboard) override
    {
        const auto sync_source_path = mb_root(mboard) / "sync_source/options";
        if (_tree->exists(sync_source_path)) {
            return _tree->access<std::vector<device_addr_t>>(sync_source_path).get();
        }
        // If this path is not there, we fall back to the oldschool method and
        // convert to a new-fangled sync source dictionary
        const auto clock_sources = get_clock_sources(mboard);
        const auto time_sources  = get_time_sources(mboard);
        std::vector<device_addr_t> sync_sources;
        for (const auto& clock_source : clock_sources) {
            for (const auto& time_source : time_sources) {
                device_addr_t sync_source;
                sync_source["clock_source"] = clock_source;
                sync_source["time_source"]  = time_source;
                sync_sources.push_back(sync_source);
            }
        }

        return sync_sources;
    }

    std::vector<std::string> get_clock_sources(const size_t mboard) override
    {
        const auto clock_source_path = mb_root(mboard) / "clock_source/options";
        if (_tree->exists(clock_source_path)) {
            return _tree->access<std::vector<std::string>>(clock_source_path).get();
        } else if (_tree->exists(mb_root(mboard) / "sync_source/options")) {
            const auto sync_sources = get_sync_sources(mboard);
            std::vector<std::string> clock_sources;
            for (const auto& sync_source : sync_sources) {
                if (sync_source.has_key("clock_source")) {
                    clock_sources.push_back(sync_source.get("clock_source"));
                }
            }
        }
        throw uhd::runtime_error("Cannot query clock_source on this device!");
    }

    void set_clock_source_out(const bool enb, const size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            if (_tree->exists(mb_root(mboard) / "clock_source" / "output")) {
                _tree->access<bool>(mb_root(mboard) / "clock_source" / "output").set(enb);
            } else {
                throw uhd::runtime_error(
                    "multi_usrp::set_clock_source_out - not supported on this device");
            }
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            this->set_clock_source_out(enb, m);
        }
    }
    void set_time_source_out(const bool enb, const size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            if (_tree->exists(mb_root(mboard) / "time_source" / "output")) {
                _tree->access<bool>(mb_root(mboard) / "time_source" / "output").set(enb);
            } else {
                throw uhd::runtime_error(
                    "multi_usrp::set_time_source_out - not supported on this device");
            }
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            this->set_time_source_out(enb, m);
        }
    }

    size_t get_num_mboards(void) override
    {
        return _tree->list("/mboards").size();
    }

    sensor_value_t get_mboard_sensor(const std::string& name, size_t mboard) override
    {
        return _tree->access<sensor_value_t>(mb_root(mboard) / "sensors" / name).get();
    }

    std::vector<std::string> get_mboard_sensor_names(size_t mboard) override
    {
        if (_tree->exists(mb_root(mboard) / "sensors")) {
            return _tree->list(mb_root(mboard) / "sensors");
        }
        return {};
    }

    void set_user_register(
        const uint8_t addr, const uint32_t data, size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            typedef std::pair<uint8_t, uint32_t> user_reg_t;
            _tree->access<user_reg_t>(mb_root(mboard) / "user/regs")
                .set(user_reg_t(addr, data));
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_user_register(addr, data, m);
        }
    }

    wb_iface::sptr get_user_settings_iface(const size_t chan) override
    {
        const auto user_settings_path = rx_rf_fe_root(chan) / "user_settings" / "iface";

        if (_tree->exists(user_settings_path)) {
            return _tree->access<wb_iface::sptr>(user_settings_path).get();
        }
        UHD_LOG_WARNING(
            "MULTI_USRP", "Attempting to read back non-existent user settings iface!");
        return nullptr;
    }

    uhd::rfnoc::radio_control& get_radio_control(const size_t) override
    {
        throw uhd::not_implemented_error(
            "get_radio_control() not supported on this device!");
    }

    uhd::extension::extension::sptr get_extension(
        const direction_t, const size_t) override
    {
        throw uhd::not_implemented_error("get_extension() not supported on this device!");
    }

    /*******************************************************************
     * RX methods
     ******************************************************************/
    rx_streamer::sptr get_rx_stream(const stream_args_t& args) override
    {
        stream_args_t args_ = args;
        if (!args.args.has_key("spp")) {
            for (auto chan : args.channels) {
                if (_rx_spp.count(chan)) {
                    args_.args.set("spp", std::to_string(_rx_spp.at(chan)));
                    break;
                }
            }
        }
        return this->get_device()->get_rx_stream(args_);

    }

    void set_rx_subdev_spec(const subdev_spec_t& spec, size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            _tree->access<subdev_spec_t>(mb_root(mboard) / "rx_subdev_spec").set(spec);
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_rx_subdev_spec(spec, m);
        }
    }

    subdev_spec_t get_rx_subdev_spec(size_t mboard) override
    {
        subdev_spec_t spec =
            _tree->access<subdev_spec_t>(mb_root(mboard) / "rx_subdev_spec").get();
        if (spec.empty()) {
            try {
                const std::string db_name =
                    _tree->list(mb_root(mboard) / "dboards").at(0);
                const std::string fe_name =
                    _tree->list(mb_root(mboard) / "dboards" / db_name / "rx_frontends")
                        .at(0);
                spec.push_back(subdev_spec_pair_t(db_name, fe_name));
                _tree->access<subdev_spec_t>(mb_root(mboard) / "rx_subdev_spec")
                    .set(spec);
            } catch (const std::exception& e) {
                throw uhd::index_error(
                    std::format("multi_usrp::get_rx_subdev_spec({}) failed to make "
                                      "default spec - {}",
                        mboard, e.what()));
            }
            UHD_LOGGER_INFO("MULTI_USRP")
                << "Selecting default RX front end spec: " << spec.to_pp_string();
        }
        return spec;
    }

    size_t get_rx_num_channels(void) override
    {

        if(_dev->is_num_rx_channels_set) return _dev->num_rx_channels;
        else {
            size_t sum = 0;
            for (size_t m = 0; m < get_num_mboards(); m++) {
                sum += get_rx_subdev_spec(m).size();
            }
            _dev->is_num_rx_channels_set = true;
            _dev->num_rx_channels = sum;
            return _dev->num_rx_channels;
        }
    }

    size_t get_rx_radio_channel(size_t chan = 0) override
    {
        return rx_chan_to_mcp(chan).chan;
    }

    std::string get_rx_subdev_name(size_t chan) override
    {
        return _tree->access<std::string>(rx_rf_fe_root(chan) / "name").get();
    }

    void set_rx_rate(double rate, size_t chan) override
    {
        get_device()->set_rx_rate(rate, chan);
    }

    void set_rx_spp(const size_t spp, const size_t chan = ALL_CHANS) override
    {
        _rx_spp[chan] = spp;
    }

    double get_rx_rate(size_t chan){
        return get_device()->get_rx_rate(chan);
    }

    meta_range_t get_rx_rates(size_t chan) override
    {
        return _tree->access<meta_range_t>(rx_dsp_root(chan) / "rate" / "range").get();
    }

    tune_result_t set_rx_freq(const tune_request_t &tune_request, size_t chan){
        // If any mixer is driven by an external LO the daughterboard assumes that no CORDIC correction is
        // necessary. Since the LO might be sourced from another daughterboard which would normally apply a
        // cordic correction a manual DSP tune policy should be used to ensure identical configurations across
        // daughterboards.
        if (tune_request.dsp_freq_policy == tune_request.POLICY_AUTO and
            tune_request.rf_freq_policy  == tune_request.POLICY_AUTO)
        {
            for (size_t c = 0; c < get_rx_num_channels(); c++) {
                const bool external_all_los = _tree->exists(rx_rf_fe_root(chan) / "los" / ALL_LOS)
                                                && get_rx_lo_source(ALL_LOS, c) == "external";
                if (external_all_los) {
                    UHD_LOGGER_WARNING("MULTI_USRP")
                            << "At least one channel is using an external LO."
                            << "Using a manual DSP frequency policy is recommended to ensure "
                            << "the same frequency shift on all channels.";
                    break;
                }
            }
        }
        tune_result_t actual_freq = get_device()->set_rx_freq(tune_request, chan);
        freq_error_check(tune_request.target_freq, actual_freq.actual_rf_freq - actual_freq.actual_dsp_freq, "RX" + std::to_string(chan));
        return actual_freq;
    }

    // Anytime GNU Radio overflows it will try to create a take an get the center frequency
    // Repeated call to get_rx_freq are a side affect of GNU Radio's error logging when overflows occur and not the root problem
    double get_rx_freq(size_t chan) override
    {
        return get_device()->get_rx_freq(chan);
    }


    freq_range_t get_rx_freq_range(size_t chan) override
    {
        return make_overall_tune_range(
            _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "freq" / "range").get(),
            _tree->access<meta_range_t>(rx_dsp_root(chan) / "freq" / "range").get(),
            this->get_rx_bandwidth(chan));
    }

    freq_range_t get_fe_rx_freq_range(size_t chan) override
    {
        return _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "freq" / "range").get();
    }

    /**************************************************************************
     * LO controls
     *************************************************************************/
    std::vector<std::string> get_rx_lo_names(size_t chan = 0) override
    {
        (void) chan; // Reference chan here to avoid unused parameter warning.
        std::vector<std::string> lo_names;
        // Upstream would return the list from the <rx_rf_fe_root>/los path if it exists, but for PV devices
        // user-facing LO operations are not implemented, so just return the empty vector.
        return lo_names;
    }

    void set_rx_lo_source(const std::string& src,
        const std::string& name = ALL_LOS,
        size_t chan             = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                if (_tree->exists(rx_rf_fe_root(chan) / "los" / ALL_LOS)) {
                    // Special value ALL_LOS support atomically sets the source for all
                    // LOs
                    _tree
                        ->access<std::string>(
                            rx_rf_fe_root(chan) / "los" / ALL_LOS / "source" / "value")
                        .set(src);
                } else {
                    for (const std::string& n :
                        _tree->list(rx_rf_fe_root(chan) / "los")) {
                        this->set_rx_lo_source(src, n, chan);
                    }
                }
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    _tree
                        ->access<std::string>(
                            rx_rf_fe_root(chan) / "los" / name / "source" / "value")
                        .set(src);
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            if (not(src == "internal" and name == ALL_LOS)) {
                throw uhd::runtime_error(
                    "This device only supports setting internal source on all LOs");
            }
        }
    }

    const std::string get_rx_lo_source(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                // Special value ALL_LOS support atomically sets the source for all LOs
                return _tree
                    ->access<std::string>(
                        rx_rf_fe_root(chan) / "los" / ALL_LOS / "source" / "value")
                    .get();
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<std::string>(
                            rx_rf_fe_root(chan) / "los" / name / "source" / "value")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // If the daughterboard doesn't expose it's LO(s) then it can only be internal
            return "internal";
        }
    }

    std::vector<std::string> get_rx_lo_sources(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                if (_tree->exists(rx_rf_fe_root(chan) / "los" / ALL_LOS)) {
                    // Special value ALL_LOS support atomically sets the source for all
                    // LOs
                    return _tree
                        ->access<std::vector<std::string>>(
                            rx_rf_fe_root(chan) / "los" / ALL_LOS / "source" / "options")
                        .get();
                } else {
                    return std::vector<std::string>();
                }
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<std::vector<std::string>>(
                            rx_rf_fe_root(chan) / "los" / name / "source" / "options")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // If the daughterboard doesn't expose it's LO(s) then it can only be internal
            return std::vector<std::string>(1, "internal");
        }
    }

    void set_rx_lo_export_enabled(
        bool enabled, const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                if (_tree->exists(rx_rf_fe_root(chan) / "los" / ALL_LOS)) {
                    // Special value ALL_LOS support atomically sets the source for all
                    // LOs
                    _tree->access<bool>(rx_rf_fe_root(chan) / "los" / ALL_LOS / "export")
                        .set(enabled);
                } else {
                    for (const std::string& n :
                        _tree->list(rx_rf_fe_root(chan) / "los")) {
                        this->set_rx_lo_export_enabled(enabled, n, chan);
                    }
                }
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    _tree->access<bool>(rx_rf_fe_root(chan) / "los" / name / "export")
                        .set(enabled);
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            if (not(enabled == false and name == ALL_LOS)) {
                throw uhd::runtime_error("This device only supports setting LO export "
                                         "enabled to false on all LOs");
            }
        }
    }

    bool get_rx_lo_export_enabled(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                // Special value ALL_LOS support atomically sets the source for all LOs
                return _tree
                    ->access<bool>(rx_rf_fe_root(chan) / "los" / ALL_LOS / "export")
                    .get();
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<bool>(rx_rf_fe_root(chan) / "los" / name / "export")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // If the daughterboard doesn't expose it's LO(s), assume it cannot export
            return false;
        }
    }

    double set_rx_lo_freq(
        double freq, const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                throw uhd::runtime_error(
                    "LO frequency must be set for each stage individually");
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    _tree
                        ->access<double>(
                            rx_rf_fe_root(chan) / "los" / name / "freq" / "value")
                        .set(freq);
                    return _tree
                        ->access<double>(
                            rx_rf_fe_root(chan) / "los" / name / "freq" / "value")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            throw uhd::runtime_error(
                "This device does not support manual configuration of LOs");
        }
    }

    double get_rx_lo_freq(const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                throw uhd::runtime_error(
                    "LO frequency must be retrieved for each stage individually");
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<double>(
                            rx_rf_fe_root(chan) / "los" / name / "freq" / "value")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // Return actual RF frequency if the daughterboard doesn't expose it's LO(s)
            return _tree->access<double>(rx_rf_fe_root(chan) / "freq" / "value").get();
        }
    }

    freq_range_t get_rx_lo_freq_range(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                throw uhd::runtime_error(
                    "LO frequency range must be retrieved for each stage individually");
            } else {
                if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<freq_range_t>(
                            rx_rf_fe_root(chan) / "los" / name / "freq" / "range")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // Return the actual RF range if the daughterboard doesn't expose it's LO(s)
            return _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "freq" / "range")
                .get();
        }
    }

    void set_rx_delay(size_t channel, int i_delay, int q_delay) {
        std::string delay_arg = std::to_string(i_delay) + " " + std::to_string(q_delay);
        _tree->access<std::string>(rx_dsp_root(channel) + "/delay_iq").set(delay_arg);
    }

    void set_tx_delay(size_t channel, int i_delay, int q_delay) {
        std::string delay_arg = std::to_string(i_delay) + " " + std::to_string(q_delay);
        _tree->access<std::string>(tx_dsp_root(channel) + "/delay_iq").set(delay_arg);
    }

    std::vector<std::string> get_tx_lo_names(const size_t chan = 0) override
    {
        return get_device()->get_tx_lo_names(chan);
    }

    void set_tx_lo_source(const std::string& src,
        const std::string& name = ALL_LOS,
        const size_t chan       = 0) override
    {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                if (_tree->exists(tx_rf_fe_root(chan) / "los" / ALL_LOS)) {
                    // Special value ALL_LOS support atomically sets the source
                    // for all LOs
                    _tree
                        ->access<std::string>(
                            tx_rf_fe_root(chan) / "los" / ALL_LOS / "source" / "value")
                        .set(src);
                } else {
                    for (const auto& n : _tree->list(tx_rf_fe_root(chan) / "los")) {
                        this->set_tx_lo_source(src, n, chan);
                    }
                }
            } else {
                if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                    _tree
                        ->access<std::string>(
                            tx_rf_fe_root(chan) / "los" / name / "source" / "value")
                        .set(src);
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            if (not(src == "internal" and name == ALL_LOS)) {
                throw uhd::runtime_error(
                    "This device only supports setting internal source on all LOs");
            }
        }
    }

    const std::string get_tx_lo_source(
        const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                return _tree
                    ->access<std::string>(
                        tx_rf_fe_root(chan) / "los" / name / "source" / "value")
                    .get();
            } else {
                throw uhd::runtime_error("Could not find LO stage " + name);
            }
        } else {
            // If the daughterboard doesn't expose its LO(s) then it can only
            // be internal
            return "internal";
        }
    }

    std::vector<std::string> get_tx_lo_sources(
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    ) {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                if (_tree->exists(tx_rf_fe_root(chan) / "los" / ALL_LOS)) {
                    // Special value ALL_LOS support atomically sets the source
                    // for all LOs
                    return _tree
                        ->access<std::vector<std::string>>(
                            tx_rf_fe_root(chan) / "los" / ALL_LOS / "source" / "options")
                        .get();
                } else {
                    return std::vector<std::string>();
                }
            } else {
                if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<std::vector<std::string>>(
                            tx_rf_fe_root(chan) / "los" / name / "source" / "options")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // If the daughterboard doesn't expose its LO(s) then it can only
            // be internal
            return std::vector<std::string>(1, "internal");
        }
    }

    void set_tx_lo_export_enabled(const bool enabled,
        const std::string& name = ALL_LOS,
        const size_t chan       = 0) override
    {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                if (_tree->exists(tx_rf_fe_root(chan) / "los" / ALL_LOS)) {
                    // Special value ALL_LOS support atomically sets the source for all
                    // LOs
                    _tree->access<bool>(tx_rf_fe_root(chan) / "los" / ALL_LOS / "export")
                        .set(enabled);
                } else {
                    for (const std::string& n :
                        _tree->list(tx_rf_fe_root(chan) / "los")) {
                        this->set_tx_lo_export_enabled(enabled, n, chan);
                    }
                }
            } else {
                if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                    _tree->access<bool>(tx_rf_fe_root(chan) / "los" / name / "export")
                        .set(enabled);
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            if (not(enabled == false and name == ALL_LOS)) {
                throw uhd::runtime_error("This device only supports setting LO export "
                                         "enabled to false on all LOs");
            }
        }
    }

    bool get_tx_lo_export_enabled(
        const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                return _tree->access<bool>(tx_rf_fe_root(chan) / "los" / name / "export")
                    .get();
            } else {
                throw uhd::runtime_error("Could not find LO stage " + name);
            }
        } else {
            // If the daughterboard doesn't expose its LO(s), assume it cannot
            // export
            return false;
        }
    }

    double set_tx_lo_freq(const double freq,
        const std::string& name = ALL_LOS,
        const size_t chan       = 0) override
    {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                throw uhd::runtime_error("LO frequency must be set for each "
                                         "stage individually");
            } else {
                if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<double>(
                            tx_rf_fe_root(chan) / "los" / name / "freq" / "value")
                        .set(freq)
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            throw uhd::runtime_error("This device does not support manual "
                                     "configuration of LOs");
        }
    }

    double get_tx_lo_freq(
            const std::string &name = ALL_LOS,
            const size_t chan = 0
    ) {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                throw uhd::runtime_error("LO frequency must be retrieved for "
                                         "each stage individually");
            } else {
                if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<double>(
                            tx_rf_fe_root(chan) / "los" / name / "freq" / "value")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // Return actual RF frequency if the daughterboard doesn't expose
            // its LO(s)
            return _tree->access<double>(tx_rf_fe_root(chan) / "freq" / " value").get();
        }
    }

    freq_range_t get_tx_lo_freq_range(
        const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            if (name == ALL_LOS) {
                throw uhd::runtime_error("LO frequency range must be retrieved "
                                         "for each stage individually");
            } else {
                if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
                    return _tree
                        ->access<freq_range_t>(
                            tx_rf_fe_root(chan) / "los" / name / "freq" / "range")
                        .get();
                } else {
                    throw uhd::runtime_error("Could not find LO stage " + name);
                }
            }
        } else {
            // Return the actual RF range if the daughterboard doesn't expose
            // its LO(s)
            return _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "freq" / "range")
                .get();
        }
    }

    /**************************************************************************
     * Gain control
     *************************************************************************/
    void set_rx_gain(double gain, const std::string& name, size_t chan) override
    {
        try {
            get_device()->set_rx_gain(gain, name, chan);
        } catch (uhd::key_error&) {
            THROW_GAIN_NAME_ERROR(name, chan, rx);
        }
    }


    void set_rx_gain_profile(const std::string& profile, const size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_rf_fe_root(chan) / "gains/all/profile/value")) {
                _tree->access<std::string>(
                         rx_rf_fe_root(chan) / "gains/all/profile/value")
                    .set(profile);
            }
        } else {
            for (size_t c = 0; c < get_rx_num_channels(); c++) {
                if (_tree->exists(rx_rf_fe_root(c) / "gains/all/profile/value")) {
                    _tree
                        ->access<std::string>(
                            rx_rf_fe_root(chan) / "gains/all/profile/value")
                        .set(profile);
                }
            }
        }
    }

    std::string get_rx_gain_profile(const size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_rf_fe_root(chan) / "gains/all/profile/value")) {
                return _tree
                    ->access<std::string>(rx_rf_fe_root(chan) / "gains/all/profile/value")
                    .get();
            }
        } else {
            throw uhd::runtime_error("Can't get RX gain profile from "
                                     "all channels at once!");
        }
        return "";
    }

    std::vector<std::string> get_rx_gain_profile_names(const size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_rf_fe_root(chan) / "gains/all/profile/options")) {
                return _tree->access<std::vector<std::string>>(
                    rx_rf_fe_root(chan) / "gains/all/profile/options"
                ).get();
            }
        } else {
            throw uhd::runtime_error("Can't get RX gain profile names from "
                                     "all channels at once!");
        }
        return std::vector<std::string>();
    }

    void set_normalized_rx_gain(double gain, size_t chan = 0) override
    {
        if (gain > 1.0 || gain < 0.0) {
            throw uhd::runtime_error("Normalized gain out of range, "
                                     "must be in [0, 1].");
        }
        const gain_range_t gain_range = get_rx_gain_range(ALL_GAINS, chan);
        const double abs_gain =
            (gain * (gain_range.stop() - gain_range.start())) + gain_range.start();
        set_rx_gain(abs_gain, ALL_GAINS, chan);
    }

    void set_rx_agc(bool enable, size_t chan = 0) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_rf_fe_root(chan) / "gain" / "agc" / "enable")) {
                _tree->access<bool>(rx_rf_fe_root(chan) / "gain" / "agc" / "enable")
                    .set(enable);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "AGC is not available on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_rx_num_channels(); c++) {
            this->set_rx_agc(enable, c);
        }

    }

    double get_rx_gain(const std::string& name, size_t chan) override
    {
        try {
            return get_device()->get_rx_gain(name, chan);
        } catch (uhd::key_error &) {
            THROW_GAIN_NAME_ERROR(name,chan,rx);
        }
    }

    double get_normalized_rx_gain(size_t chan) override
    {
        gain_range_t gain_range = get_rx_gain_range(ALL_GAINS, chan);
        double gain_range_width = gain_range.stop() - gain_range.start();
        // In case we have a device without a range of gains:
        if (gain_range_width == 0.0) {
            return 0;
        }
        double norm_gain =
            (get_rx_gain(ALL_GAINS, chan) - gain_range.start()) / gain_range_width;
        // Avoid rounding errors:
        if (norm_gain > 1.0)
            return 1.0;
        if (norm_gain < 0.0)
            return 0.0;
        return norm_gain;
    }

    gain_range_t get_rx_gain_range(const std::string& name, size_t chan) override
    {
        if(name != CURRENT_BAND_GAINS && name != ALL_GAINS) {
            THROW_GAIN_NAME_ERROR(name, chan, tx);
        }
        // Sets the range to get it to update to the current band
        _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "gain" / "range").set(meta_range_t(0.0, 0.0, 0.0));
        // Gets the current range
        return _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "gain" / "range").get();
    }

    std::vector<std::string> get_rx_gain_names(size_t chan) override
    {
        (void) chan;
        return {std::string(CURRENT_BAND_GAINS)};
    }

    /**************************************************************************
     * RX Power control
     *************************************************************************/
    bool has_rx_power_reference(const size_t chan) override
    {
        return _tree->exists(rx_rf_fe_root(chan) / "ref_power/value");
    }

    void set_rx_power_reference(const double power_dbm, const size_t chan = 0) override
    {
        const auto power_ref_path = rx_rf_fe_root(chan) / "ref_power/value";
        if (!_tree->exists(power_ref_path)) {
            throw uhd::not_implemented_error(
                "set_rx_power_reference() not available for this device and channel");
        }
        _tree->access<double>(power_ref_path).set(power_dbm);
    }

    double get_rx_power_reference(const size_t chan = 0) override
    {
        const auto power_ref_path = rx_rf_fe_root(chan) / "ref_power/value";
        if (!_tree->exists(power_ref_path)) {
            throw uhd::not_implemented_error(
                "get_rx_power_reference() not available for this device and channel");
        }
        return _tree->access<double>(power_ref_path).get();
    }

    meta_range_t get_rx_power_range(const size_t chan) override
    {
        const auto power_ref_path = rx_rf_fe_root(chan) / "ref_power/range";
        if (!_tree->exists(power_ref_path)) {
            throw uhd::not_implemented_error(
                "get_rx_power_range() not available for this device and channel");
        }
        return _tree->access<meta_range_t>(power_ref_path).get();
    }

    void set_rx_antenna(const std::string& ant, size_t chan) override
    {
        _tree->access<std::string>(rx_rf_fe_root(chan) / "antenna" / "value").set(ant);
    }

    std::string get_rx_antenna(size_t chan) override
    {
        return _tree->access<std::string>(rx_rf_fe_root(chan) / "antenna" / "value")
            .get();

    }

    std::vector<std::string> get_rx_antennas(size_t chan) override
    {
        return _tree
            ->access<std::vector<std::string>>(
                rx_rf_fe_root(chan) / "antenna" / "options")
            .get();
    }

    void set_rx_bandwidth(double bandwidth, size_t chan) override
    {
        _tree->access<double>(rx_rf_fe_root(chan) / "bandwidth" / "value").set(bandwidth);
    }

    double get_rx_bandwidth(size_t chan) override
    {
        return _tree->access<double>(rx_rf_fe_root(chan) / "bandwidth" / "value").get();
    }

    meta_range_t get_rx_bandwidth_range(size_t chan) override
    {
        return _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "bandwidth" / "range")
            .get();

    }

    dboard_iface::sptr get_rx_dboard_iface(size_t chan) override
    {
        return _tree
            ->access<dboard_iface::sptr>(
                rx_rf_fe_root(chan).branch_path().branch_path() / "iface")
            .get();
    }

    sensor_value_t get_rx_sensor(const std::string& name, size_t chan) override
    {
        return _tree->access<sensor_value_t>(rx_rf_fe_root(chan) / "sensors" / name)
            .get();

    }

    std::vector<std::string> get_rx_sensor_names(size_t chan) override
    {

        std::vector<std::string> sensor_names;
        if (_tree->exists(rx_rf_fe_root(chan) / "sensors")) {
            sensor_names = _tree->list(rx_rf_fe_root(chan) / "sensors");
        }
        return sensor_names;
    }

    void set_rx_dc_offset(const bool enb, size_t chan) override
    {

        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_fe_root(chan) / "dc_offset" / "enable")) {
                _tree->access<bool>(rx_fe_root(chan) / "dc_offset" / "enable").set(enb);
            } else if (_tree->exists(rx_rf_fe_root(chan) / "dc_offset" / "enable")) {
                /*For B2xx devices the dc-offset correction is implemented in the rf
                 * front-end*/
                _tree->access<bool>(rx_rf_fe_root(chan) / "dc_offset" / "enable")
                    .set(enb);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "Setting DC offset compensation is not possible on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_rx_num_channels(); c++) {
            this->set_rx_dc_offset(enb, c);
        }
    }

    void set_rx_dc_offset(const std::complex<double>& offset, size_t chan) override
    {

        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_fe_root(chan) / "dc_offset" / "value")) {
                _tree
                    ->access<std::complex<double>>(
                        rx_fe_root(chan) / "dc_offset" / "value")
                    .set(offset);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "Setting DC offset is not possible on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_rx_num_channels(); c++) {
            this->set_rx_dc_offset(offset, c);
        }
    }

    meta_range_t get_rx_dc_offset_range(size_t chan) override
    {
        if (_tree->exists(rx_fe_root(chan) / "dc_offset" / "range")) {
            return _tree
                ->access<uhd::meta_range_t>(rx_fe_root(chan) / "dc_offset" / "range")
                .get();
        } else {
            UHD_LOGGER_WARNING("MULTI_USRP")
                << "This device does not support querying the RX DC offset range.";
            return meta_range_t(0.0, 0.0);
        }
    }

    void set_rx_iq_balance(const bool enb, size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_rf_fe_root(chan) / "iq_balance" / "enable")) {
                _tree->access<bool>(rx_rf_fe_root(chan) / "iq_balance" / "enable")
                    .set(enb);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP") << "Setting IQ imbalance compensation "
                                                    "is not possible on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_rx_num_channels(); c++) {
            this->set_rx_iq_balance(enb, c);
        }
    }

    void set_rx_iq_balance(const std::complex<double>& offset, size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(rx_fe_root(chan) / "iq_balance" / "value")) {
                _tree
                    ->access<std::complex<double>>(
                        rx_fe_root(chan) / "iq_balance" / "value")
                    .set(offset);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "Setting IQ balance is not possible on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_rx_num_channels(); c++) {
            this->set_rx_iq_balance(offset, c);
        }
    }

    std::vector<std::string> get_rx_filter_names(const size_t chan) override
    {
        if (chan >= get_rx_num_channels()) {
            throw uhd::index_error("Attempting to get non-existent RX filter names");
        }
        std::vector<std::string> ret;

        if (_tree->exists(rx_rf_fe_root(chan) / "filters")) {
            std::vector<std::string> names = _tree->list(rx_rf_fe_root(chan) / "filters");
            for (size_t i = 0; i < names.size(); i++) {
                std::string name = rx_rf_fe_root(chan) / "filters" / names[i];
                ret.push_back(name);
            }
        }
        if (_tree->exists(rx_dsp_root(chan) / "filters")) {
            std::vector<std::string> names = _tree->list(rx_dsp_root(chan) / "filters");
            for (size_t i = 0; i < names.size(); i++) {
                std::string name = rx_dsp_root(chan) / "filters" / names[i];
                ret.push_back(name);
            }
        }

        return ret;
    }

    uhd::filter_info_base::sptr get_rx_filter(
        const std::string& name, const size_t chan) override
    {
        std::vector<std::string> possible_names = get_rx_filter_names(chan);
        std::vector<std::string>::iterator it;
        it = find(possible_names.begin(), possible_names.end(), name);
        if (it == possible_names.end()) {
            throw uhd::runtime_error("Attempting to get non-existing filter: " + name);
        }

        return _tree->access<filter_info_base::sptr>(fs_path(name) / "value").get();
    }

    void set_rx_filter(const std::string& name,
        uhd::filter_info_base::sptr filter,
        const size_t chan) override
    {
        std::vector<std::string> possible_names = get_rx_filter_names(chan);
        std::vector<std::string>::iterator it;
        it = find(possible_names.begin(), possible_names.end(), name);
        if (it == possible_names.end()) {
            throw uhd::runtime_error("Attempting to set non-existing filter: " + name);
        }

        _tree->access<filter_info_base::sptr>(fs_path(name) / "value").set(filter);
    }

    std::string get_tx_sfp(size_t chan) override
    {
        return get_device()->get_tx_sfp(chan);
    }

    std::string get_tx_ip(size_t chan) override
    {
        return get_device()->get_tx_ip(chan);
    }

    uint16_t get_tx_fc_port(size_t chan) override
    {
        return get_device()->get_tx_fc_port(chan);
    }

    uint16_t get_tx_udp_port(size_t chan) override
    {
        return get_device()->get_tx_udp_port(chan);
    }

    std::vector<std::string> get_tx_filter_names(const size_t chan) override
    {
        if (chan >= get_tx_num_channels()) {
            throw uhd::index_error("Attempting to get non-existent TX filter names");
        }
        std::vector<std::string> ret;

        if (_tree->exists(tx_rf_fe_root(chan) / "filters")) {
            std::vector<std::string> names = _tree->list(tx_rf_fe_root(chan) / "filters");
            for (size_t i = 0; i < names.size(); i++) {
                std::string name = tx_rf_fe_root(chan) / "filters" / names[i];
                ret.push_back(name);
            }
        }
        if (_tree->exists(rx_dsp_root(chan) / "filters")) {
            std::vector<std::string> names = _tree->list(tx_dsp_root(chan) / "filters");
            for (size_t i = 0; i < names.size(); i++) {
                std::string name = tx_dsp_root(chan) / "filters" / names[i];
                ret.push_back(name);
            }
        }

        return ret;
    }

    uhd::filter_info_base::sptr get_tx_filter(
        const std::string& name, const size_t chan) override
    {
        std::vector<std::string> possible_names = get_tx_filter_names(chan);
        std::vector<std::string>::iterator it;
        it = find(possible_names.begin(), possible_names.end(), name);
        if (it == possible_names.end()) {
            throw uhd::runtime_error("Attempting to get non-existing filter: " + name);
        }

        return _tree->access<filter_info_base::sptr>(fs_path(name) / "value").get();
    }

    void set_tx_filter(const std::string& name,
        uhd::filter_info_base::sptr filter,
        const size_t chan) override
    {
        std::vector<std::string> possible_names = get_tx_filter_names(chan);

        std::vector<std::string>::iterator it;
        it = find(possible_names.begin(), possible_names.end(), name);
        if (it == possible_names.end()) {
            throw uhd::runtime_error("Attempting to set non-existing filter: " + name);
        }

        _tree->access<filter_info_base::sptr>(fs_path(name) / "value").set(filter);
    }

    /*******************************************************************
     * TX methods
     ******************************************************************/
    tx_streamer::sptr get_tx_stream(const stream_args_t& args) override
    {
        return this->get_device()->get_tx_stream(args);
    }

    void set_tx_subdev_spec(const subdev_spec_t& spec, size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            _tree->access<subdev_spec_t>(mb_root(mboard) / "tx_subdev_spec").set(spec);
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_tx_subdev_spec(spec, m);
        }
    }

    subdev_spec_t get_tx_subdev_spec(size_t mboard) override
    {
        subdev_spec_t spec =
            _tree->access<subdev_spec_t>(mb_root(mboard) / "tx_subdev_spec").get();
        if (spec.empty()) {
            try {
                const std::string db_name =
                    _tree->list(mb_root(mboard) / "dboards").at(0);
                const std::string fe_name =
                    _tree->list(mb_root(mboard) / "dboards" / db_name / "tx_frontends")
                        .at(0);
                spec.push_back(subdev_spec_pair_t(db_name, fe_name));
                _tree->access<subdev_spec_t>(mb_root(mboard) / "tx_subdev_spec")
                    .set(spec);
                UHD_LOGGER_INFO("MULTI_USRP") << "Selecting default TX front end spec: " << spec.to_pp_string();
            }
            catch(const std::exception &e)
            {
                UHD_LOGGER_INFO("MULTI_USRP") << "No tx front ends detected";

            }

        }
        return spec;
    }

    size_t get_tx_num_channels(void) override
    {
        if(_dev->is_num_tx_channels_set) return _dev->num_tx_channels;
        else {
            size_t sum = 0;
            for (size_t m = 0; m < get_num_mboards(); m++) {
                sum += get_tx_subdev_spec(m).size();
            }
            _dev->is_num_tx_channels_set = true;
            _dev->num_tx_channels = sum;
            return _dev->num_tx_channels;
        }
    }

    size_t get_tx_radio_channel(size_t chan = 0) override
    {
        return tx_chan_to_mcp(chan).chan;
    }

    std::string get_tx_subdev_name(size_t chan) override
    {
        return _tree->access<std::string>(tx_rf_fe_root(chan) / "name").get();
    }

    void set_tx_rate(double rate, size_t chan) override
    {
        get_device()->set_tx_rate(rate, chan);
    }

    double get_tx_rate(size_t chan) override
    {
        return get_device()->get_tx_rate(chan);
    }

    meta_range_t get_tx_rates(size_t chan) override
    {
        return _tree->access<meta_range_t>(tx_dsp_root(chan) / "rate" / "range").get();
    }

    tune_result_t set_tx_freq(const tune_request_t& tune_request, size_t chan) override
    {
        tune_result_t actual_freq = get_device()->set_tx_freq(tune_request, chan);
        freq_error_check(tune_request.target_freq, actual_freq.actual_rf_freq + actual_freq.actual_dsp_freq, "TX" + std::to_string(chan));
        return actual_freq;
    }

    double get_tx_freq(size_t chan) override
    {
        return get_device()->get_tx_freq(chan);
    }


    freq_range_t get_tx_freq_range(size_t chan) override
    {
        return make_overall_tune_range(
            _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "freq" / "range").get(),
            _tree->access<meta_range_t>(tx_dsp_root(chan) / "freq" / "range").get(),
            this->get_tx_bandwidth(chan));
    }

    freq_range_t get_fe_tx_freq_range(size_t chan) override
    {
        return _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "freq" / "range").get();
    }

    void set_tx_gain(double gain, const std::string& name, size_t chan) override
    {
        get_device()->set_tx_gain(gain, name, chan);
    }

//    void set_tx_gain(double gain, const std::string &name, size_t chan){
//        try {
//            return tx_gain_group(chan)->set_value(gain, name);
//        } catch (uhd::key_error &) {
//            THROW_GAIN_NAME_ERROR(name,chan,tx);
//        }
//    }

    void set_tx_gain_profile(const std::string& profile, const size_t chan){
        if (chan != ALL_CHANS) {
            if (_tree->exists(tx_rf_fe_root(chan) / "gains/all/profile/value")) {
                _tree->access<std::string>(
                         tx_rf_fe_root(chan) / "gains/all/profile/value")
                    .set(profile);
            }
        } else {
            for (size_t c = 0; c < get_tx_num_channels(); c++) {
                if (_tree->exists(tx_rf_fe_root(c) / "gains/all/profile/value")) {
                    _tree
                        ->access<std::string>(
                            tx_rf_fe_root(chan) / "gains/all/profile/value")
                        .set(profile);
                }
            }
        }
    }

    std::string get_tx_gain_profile(const size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(tx_rf_fe_root(chan) / "gains/all/profile/value")) {
                return _tree
                    ->access<std::string>(tx_rf_fe_root(chan) / "gains/all/profile/value")
                    .get();
            }
        } else {
            throw uhd::runtime_error("Can't get TX gain profile from "
                                     "all channels at once!");
        }
        return "";
    }

    std::vector<std::string> get_tx_gain_profile_names(const size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(tx_rf_fe_root(chan) / "gains/all/profile/options")) {
                return _tree
                    ->access<std::vector<std::string>>(
                        tx_rf_fe_root(chan) / "gains/all/profile/options")
                    .get();
            }
        } else {
            throw uhd::runtime_error("Can't get TX gain profile names from "
                                     "all channels at once!");
        }
        return std::vector<std::string>();
    }

    void set_normalized_tx_gain(double gain, size_t chan = 0) override
    {
        if (gain > 1.0 || gain < 0.0) {
            throw uhd::runtime_error("Normalized gain out of range, must be in [0, 1].");
        }
        gain_range_t gain_range = get_tx_gain_range(ALL_GAINS, chan);
        double abs_gain =
            (gain * (gain_range.stop() - gain_range.start())) + gain_range.start();
        set_tx_gain(abs_gain, ALL_GAINS, chan);
    }


    double get_tx_gain(const std::string& name, size_t chan) override
    {

        try {
            return get_device()->get_tx_gain(name, chan);
        } catch (uhd::key_error &) {
            THROW_GAIN_NAME_ERROR(name,chan,rx);
        }
    }

    double get_normalized_tx_gain(size_t chan) override
    {
        gain_range_t gain_range = get_tx_gain_range(ALL_GAINS, chan);
        double gain_range_width = gain_range.stop() - gain_range.start();
        // In case we have a device without a range of gains:
        if (gain_range_width == 0.0) {
            return 0.0;
        }
        double norm_gain =
            (get_tx_gain(ALL_GAINS, chan) - gain_range.start()) / gain_range_width;
        // Avoid rounding errors:
        if (norm_gain > 1.0)
            return 1.0;
        if (norm_gain < 0.0)
            return 0.0;
        return norm_gain;
    }

    gain_range_t get_tx_gain_range(const std::string& name, size_t chan) override
    {
        if(name != CURRENT_BAND_GAINS && name != ALL_GAINS) {
            THROW_GAIN_NAME_ERROR(name, chan, tx);
        }
        // Sets the range to get it to update to the current band
        _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "gain" / "range").set(meta_range_t(0.0, 0.0, 0.0));
        // Gets the current range
        return _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "gain" / "range").get();
    }

    std::vector<std::string> get_tx_gain_names(size_t chan) override
    {
        (void) chan;
        return {std::string(CURRENT_BAND_GAINS)};
    }

    /*!
     * Get the number of samples in the buffer per number reported in packets sent to fif udp port
     */
    int64_t get_tx_buff_scale(){
        return get_device()->get_tx_buff_scale();
    }

    /**************************************************************************
     * TX Power Controls
     *************************************************************************/
    bool has_tx_power_reference(const size_t chan) override
    {
        return _tree->exists(tx_rf_fe_root(chan) / "ref_power/value");
    }

    void set_tx_power_reference(const double power_dbm, const size_t chan = 0) override
    {
        const auto power_ref_path = tx_rf_fe_root(chan) / "ref_power/value";
        if (!_tree->exists(power_ref_path)) {
            throw uhd::not_implemented_error(
                "set_tx_power_reference() not available for this device and channel");
        }
        _tree->access<double>(power_ref_path).set(power_dbm);
    }

    double get_tx_power_reference(const size_t chan = 0) override
    {
        const auto power_ref_path = tx_rf_fe_root(chan) / "ref_power/value";
        if (!_tree->exists(power_ref_path)) {
            throw uhd::not_implemented_error(
                "get_tx_power_reference() not available for this device and channel");
        }
        return _tree->access<double>(power_ref_path).get();
    }

    meta_range_t get_tx_power_range(const size_t chan) override
    {
        const auto power_ref_path = tx_rf_fe_root(chan) / "ref_power/range";
        if (!_tree->exists(power_ref_path)) {
            throw uhd::not_implemented_error(
                "get_tx_power_range() not available for this device and channel");
        }
        return _tree->access<meta_range_t>(power_ref_path).get();
    }

    void set_tx_antenna(const std::string& ant, size_t chan) override
    {
        _tree->access<std::string>(tx_rf_fe_root(chan) / "antenna" / "value").set(ant);
    }

    std::string get_tx_antenna(size_t chan) override
    {
        return _tree->access<std::string>(tx_rf_fe_root(chan) / "antenna" / "value")
            .get();

    }

    std::vector<std::string> get_tx_antennas(size_t chan) override
    {
        return _tree
            ->access<std::vector<std::string>>(
                tx_rf_fe_root(chan) / "antenna" / "options")
            .get();
    }

    void set_tx_bandwidth(double bandwidth, size_t chan) override
    {
        _tree->access<double>(tx_rf_fe_root(chan) / "bandwidth" / "value").set(bandwidth);
    }

    double get_tx_bandwidth(size_t chan) override
    {
        return _tree->access<double>(tx_rf_fe_root(chan) / "bandwidth" / "value").get();
    }

    meta_range_t get_tx_bandwidth_range(size_t chan) override
    {
        return _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "bandwidth" / "range")
            .get();

    }

    dboard_iface::sptr get_tx_dboard_iface(size_t chan) override
    {
        return _tree
            ->access<dboard_iface::sptr>(
                tx_rf_fe_root(chan).branch_path().branch_path() / "iface")
            .get();
    }

    sensor_value_t get_tx_sensor(const std::string& name, size_t chan) override
    {
        return _tree->access<sensor_value_t>(tx_rf_fe_root(chan) / "sensors" / name)
            .get();

    }

    std::vector<std::string> get_tx_sensor_names(size_t chan) override
    {
        std::vector<std::string> sensor_names;
        if (_tree->exists(tx_rf_fe_root(chan) / "sensors")) {
            sensor_names = _tree->list(tx_rf_fe_root(chan) / "sensors");
        }
        return sensor_names;
    }

    void set_tx_dc_offset(const std::complex<double>& offset, size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(tx_fe_root(chan) / "dc_offset" / "value")) {
                _tree
                    ->access<std::complex<double>>(
                        tx_fe_root(chan) / "dc_offset" / "value")
                    .set(offset);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "Setting DC offset is not possible on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_tx_num_channels(); c++) {
            this->set_tx_dc_offset(offset, c);
        }
    }

    meta_range_t get_tx_dc_offset_range(size_t chan) override
    {
        if (_tree->exists(tx_fe_root(chan) / "dc_offset" / "range")) {
            return _tree
                ->access<uhd::meta_range_t>(tx_fe_root(chan) / "dc_offset" / "range")
                .get();
        } else {
            UHD_LOGGER_WARNING("MULTI_USRP")
                << "This device does not support querying the TX DC offset range.";
            return meta_range_t(0.0, 0.0);
        }
    }

    void set_tx_iq_balance(const std::complex<double>& offset, size_t chan) override
    {
        if (chan != ALL_CHANS) {
            if (_tree->exists(tx_fe_root(chan) / "iq_balance" / "value")) {
                _tree
                    ->access<std::complex<double>>(
                        tx_fe_root(chan) / "iq_balance" / "value")
                    .set(offset);
            } else {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "Setting IQ balance is not possible on this device.";
            }
            return;
        }
        for (size_t c = 0; c < get_tx_num_channels(); c++) {
            this->set_tx_iq_balance(offset, c);
        }
    }

    /***********************************************************************
     * Deprecated trigger functions: wrappers for trigger_configure
     **********************************************************************/
    // Both tx and rx trigger configurations set the trigger direction. These are used for a warning
    enum which_rxtx { none, rx, tx };
    which_rxtx last_trigger_dir_set_by = none;
    std::string previous_trigger_dir;

    /*!
     * RX and TX share one trigger port. Warn if rx_trigger_setup and
     * tx_trigger_setup asked for different directions, and record which side
     * set it. Same behaviour as the original functions.
     */
    void warn_on_trigger_dir_conflict(const which_rxtx setting_from, const std::string& trigger_dir)
    {
        const which_rxtx other_side = (setting_from == rx) ? tx : rx;
        if (last_trigger_dir_set_by == other_side) {
            if (previous_trigger_dir != trigger_dir) {
                const char* this_name  = (setting_from == rx) ? "rx" : "tx";
                const char* other_name = (setting_from == rx) ? "tx" : "rx";
                UHD_LOGGER_WARNING("MULTI_USRP") << std::format(
                    "Trigger direction was set to a different value when configuring {}. "
                    "RX and TX use the same trigger port. Previously set to {}, changing it "
                    "to {} while setting up the {} trigger",
                    other_name, previous_trigger_dir, trigger_dir, this_name);
            }
        } else {
            last_trigger_dir_set_by = setting_from;
            previous_trigger_dir    = trigger_dir;
        }
    }

    //! Every channel in [0, num_channels) that is not in `channels`, in ascending order.
    static std::vector<size_t> trigger_other_channels(
        const std::vector<size_t>& channels, const size_t num_channels)
    {
        std::vector<size_t> others;
        for (size_t channel = 0; channel < num_channels; channel++) {
            if (std::find(channels.begin(), channels.end(), channel) == channels.end()) {
                others.push_back(channel);
            }
        }
        return others;
    }

    /*!
     * Adjust a preset's op for the deprecated functions: like the originals,
     * accept whatever the device applies, and keep working with older servers
     * (with a warning).
     */
    static void trigger_make_legacy_op(uhd::trigger_pv::trigger_op_t& op)
    {
        using namespace uhd::trigger_pv;
        op.enable_strict_errors &= ~strict_error_t::coerced_value;
        op.enable_strict_errors &= ~strict_error_t::unsupported_server;
    }

    /*!
     * Fill one entry so trigger_configure() accesses as little of the channel as
     * possible: everything that supports bypass is bypassed, and the fields that
     * depend on those are not relevant, so they are skipped. The channel is hot
     * configured; enable (and TX gating), which have no bypass, are only read.
     */
    static void trigger_fill_bypass(uhd::trigger_pv::trigger_rx_t& chan, const size_t channel)
    {
        using namespace uhd::trigger_pv;
        chan               = trigger_rx_t{};
        chan.channel       = channel;
        chan.hot_configure = true;
        chan.trig_sel      = active_trigger_t::bypass;
        chan.sma_mode      = trigger_mode_t::bypass;
        chan.ufl_mode      = trigger_mode_t::bypass;
        chan.ufl_polarity  = polarity_t::bypass;
        // Not relevant with trig_sel bypassed, so never accessed.
        chan.edge_backoff            = SKIP_IF_NOT_RELEVANT;
        chan.edge_sample_num         = SKIP_IF_NOT_RELEVANT;
        chan.ufl_output_pulse_length = SKIP_IF_NOT_RELEVANT;
        chan.ufl_dir                 = pin_direction_t::must_specify_if_applicable;
        chan.enable                  = GET_INT;
    }
    static void trigger_fill_bypass(uhd::trigger_pv::trigger_tx_t& chan, const size_t channel)
    {
        using namespace uhd::trigger_pv;
        chan               = trigger_tx_t{};
        chan.channel       = channel;
        chan.hot_configure = true;
        chan.trig_sel      = active_trigger_t::bypass;
        chan.sma_mode      = trigger_mode_t::bypass;
        chan.ufl_mode      = trigger_mode_t::bypass;
        chan.ufl_polarity  = polarity_t::bypass;
        // Not relevant with trig_sel bypassed, so never accessed.
        chan.edge_backoff            = SKIP_IF_NOT_RELEVANT;
        chan.edge_sample_num         = SKIP_IF_NOT_RELEVANT;
        chan.ufl_output_pulse_length = SKIP_IF_NOT_RELEVANT;
        chan.ufl_dir                 = pin_direction_t::must_specify_if_applicable;
        chan.enable                  = GET_INT;
        chan.gating                  = gating_t::get;
    }

    /*!
     * In an entry for a channel the deprecated functions configure, replace
     * fields left as get (or GET_INT) with bypass, or with SKIP_IF_NOT_RELEVANT
     * for integer fields that support it, so they are not accessed. The
     * original functions never read them. Fields with no such value (enable,
     * TX gating, sma_override) are left as they are.
     */
    template <typename channel_t>
    static void trigger_bypass_unset_fields(channel_t& chan)
    {
        using namespace uhd::trigger_pv;
        if (chan.trig_sel == active_trigger_t::get) {
            chan.trig_sel = active_trigger_t::bypass;
        }
        if (chan.sma_mode == trigger_mode_t::get) {
            chan.sma_mode = trigger_mode_t::bypass;
        }
        if (chan.ufl_mode == trigger_mode_t::get) {
            chan.ufl_mode = trigger_mode_t::bypass;
        }
        if (chan.ufl_polarity == polarity_t::get) {
            chan.ufl_polarity = polarity_t::bypass;
        }
        for (int64_t* value :
            {&chan.edge_backoff, &chan.edge_sample_num, &chan.ufl_output_pulse_length}) {
            if (*value == GET_INT) {
                *value = SKIP_IF_NOT_RELEVANT;
            }
        }
    }

    /*!
     * Fill args[first], args[first + 1], ... with bypass entries for the
     * channels in `preserved` (see trigger_fill_bypass()).
     */
    template <typename channel_t>
    static void trigger_fill_preserved_bypass(
        std::vector<channel_t>& args, const size_t first, const std::vector<size_t>& preserved)
    {
        for (size_t i = 0; i < preserved.size(); i++) {
            trigger_fill_bypass(args[first + i], preserved[i]);
        }
    }

    /*!
     * Shared body of the deprecated rx_trigger_setup and tx_trigger_setup:
     * configure `channels` with the get_simple_sma_config preset. Every other
     * RX and TX channel is preserved, as the original functions did not touch
     * them. To access the device as little as possible, every other channel
     * is bypassed (see trigger_fill_bypass()), and so are the configured
     * channels' fields that the preset leaves as get (see
     * trigger_bypass_unset_fields()).
     * \return actual number of samples per trigger (0 if channels is empty)
     */
    uint64_t trigger_setup_wrapper(const which_rxtx setting_from,
        const std::vector<size_t>& channels,
        const uint64_t num_samples_per_trigger,
        const std::string& trigger_dir)
    {
        using namespace uhd::trigger_pv;

        if (!std::in_range<int64_t>(num_samples_per_trigger)) {
            throw uhd::value_error(std::format(
                "num_samples_per_trigger {} is too large", num_samples_per_trigger));
        }
        pin_direction_t sma_direction;
        if (trigger_dir == "in") {
            sma_direction = pin_direction_t::in;
        } else if (trigger_dir == "out") {
            sma_direction = pin_direction_t::out;
        } else {
            throw uhd::value_error(std::format(
                "trigger_dir must be \"in\" or \"out\", got \"{}\"", trigger_dir));
        }

        warn_on_trigger_dir_conflict(setting_from, trigger_dir);

        const std::vector<size_t> no_channels;
        const std::vector<size_t>& rx_channels = (setting_from == rx) ? channels : no_channels;
        const std::vector<size_t>& tx_channels = (setting_from == tx) ? channels : no_channels;
        const std::vector<size_t> preserved_rx = trigger_other_channels(rx_channels, get_rx_num_channels());
        const std::vector<size_t> preserved_tx = trigger_other_channels(tx_channels, get_tx_num_channels());

        trigger_op_t op;
        trigger_global_t global;
        std::vector<trigger_rx_t> rx_args(rx_channels.size() + preserved_rx.size());
        std::vector<trigger_tx_t> tx_args(tx_channels.size() + preserved_tx.size());
        get_simple_sma_config(&op, &global, rx_args.data(), rx_args.size(), tx_args.data(),
            tx_args.size(), rx_channels, tx_channels, sma_direction,
            static_cast<int64_t>(num_samples_per_trigger), &preserved_rx, &preserved_tx);
        trigger_make_legacy_op(op);
        // Access as little as possible: the configured channels' remaining get
        // fields and every preserved channel (both directions) are bypassed.
        for (size_t i = 0; i < rx_channels.size(); i++) {
            trigger_bypass_unset_fields(rx_args[i]);
        }
        for (size_t i = 0; i < tx_channels.size(); i++) {
            trigger_bypass_unset_fields(tx_args[i]);
        }
        trigger_fill_preserved_bypass(rx_args, rx_channels.size(), preserved_rx);
        trigger_fill_preserved_bypass(tx_args, tx_channels.size(), preserved_tx);

        // The return value only says whether something was coerced; the actual
        // values are read from the structs below instead.
        (void) trigger_configure(
            &op, &global, rx_args.data(), rx_args.size(), tx_args.data(), tx_args.size());

        if (channels.empty()) {
            return 0;
        }
        // The configured channels come first, and every one was given the same
        // request, so they report the same value.
        if (setting_from == rx) {
            return static_cast<uint64_t>(rx_args[channels.size() - 1].edge_sample_num);
        } else {
            return static_cast<uint64_t>(tx_args[channels.size() - 1].edge_sample_num);
        }
    }

    /*!
     * Shared body of the deprecated rx_trigger_cleanup and tx_trigger_cleanup:
     * stop triggering on `channels` (trig_sel none, and for RX, stream off) and
     * set edge_sample_num back to 0. Other settings are untouched, and every
     * other RX and TX channel is preserved. As in the setup wrapper, fields
     * that are not set and every other channel are bypassed where possible.
     */
    void trigger_cleanup_wrapper(const which_rxtx setting_from, const std::vector<size_t>& channels)
    {
        using namespace uhd::trigger_pv;

        trigger_op_t op;
        trigger_make_legacy_op(op);

        const std::vector<size_t> no_channels;
        const std::vector<size_t>& rx_channels = (setting_from == rx) ? channels : no_channels;
        const std::vector<size_t>& tx_channels = (setting_from == tx) ? channels : no_channels;
        const std::vector<size_t> preserved_rx = trigger_other_channels(rx_channels, get_rx_num_channels());
        const std::vector<size_t> preserved_tx = trigger_other_channels(tx_channels, get_tx_num_channels());

        // Fields not set here are bypassed or skipped below where possible.
        // TX channels are hot configured
        // because the original tx_trigger_cleanup did not power them off.
        std::vector<trigger_rx_t> rx_args(rx_channels.size() + preserved_rx.size());
        std::vector<trigger_tx_t> tx_args(tx_channels.size() + preserved_tx.size());
        for (size_t i = 0; i < rx_channels.size(); i++) {
            trigger_rx_t& chan   = rx_args[i];
            chan.channel         = rx_channels[i];
            // Disabled first, as the original turned stream off before trig_sel.
            chan.hot_configure   = false;
            chan.trig_sel        = active_trigger_t::none;
            chan.edge_sample_num = 0;
            chan.enable          = 0;
        }
        for (size_t i = 0; i < rx_channels.size(); i++) {
            trigger_bypass_unset_fields(rx_args[i]);
        }
        trigger_fill_preserved_bypass(rx_args, rx_channels.size(), preserved_rx);
        for (size_t i = 0; i < tx_channels.size(); i++) {
            trigger_tx_t& chan   = tx_args[i];
            chan.channel         = tx_channels[i];
            chan.hot_configure   = true;
            chan.trig_sel        = active_trigger_t::none;
            chan.edge_sample_num = 0;
        }
        for (size_t i = 0; i < tx_channels.size(); i++) {
            trigger_bypass_unset_fields(tx_args[i]);
        }
        trigger_fill_preserved_bypass(tx_args, tx_channels.size(), preserved_tx);

        (void) trigger_configure(
            &op, nullptr, rx_args.data(), rx_args.size(), tx_args.data(), tx_args.size());

        last_trigger_dir_set_by = none;
    }

    uint64_t tx_trigger_setup(
        std::vector<size_t> channels,
        uint64_t num_samples_per_trigger,
        std::string trigger_dir
    ) {
        return trigger_setup_wrapper(tx, channels, num_samples_per_trigger, trigger_dir);
    }

    void tx_trigger_cleanup(
        std::vector<size_t> channels
    ) {
        trigger_cleanup_wrapper(tx, channels);
    }

    uint64_t rx_trigger_setup(
        std::vector<size_t> channels,
        uint64_t num_samples_per_trigger,
        std::string trigger_dir
    ) {
        return trigger_setup_wrapper(rx, channels, num_samples_per_trigger, trigger_dir);
    }

    void rx_trigger_cleanup(
        std::vector<size_t> channels
    ) {
        trigger_cleanup_wrapper(rx, channels);
    }

    void rx_start_force_stream(std::vector<size_t> channels) {
        if (_tree->exists("/mboards/0/cm/rx/force_stream")) {
            int stream_mask = 0;
            for(size_t n = 0; n <channels.size(); n++) {
                //when seting cm force stream, bit 0 corresponds to chA, bit 1 to chB, etc
                stream_mask |= 1 << channels[n];
            }
            _tree->access<int>("/mboards/0/cm/rx/force_stream").set(stream_mask);
        } else {
            throw uhd::not_implemented_error("Rx force stream not implemented for this device");
        }
    }

    void rx_stop_force_stream(std::vector<size_t> channels) {
        if (_tree->exists("/mboards/0/cm/rx/force_stream")) {
            int active_channels = _tree->access<int>("/mboards/0/cm/rx/force_stream").get();
            int stream_mask = 0;
            for(size_t n = 0; n <channels.size(); n++) {
                //when seting cm force stream, bit 0 corresponds to chA, bit 1 to chB, etc
                stream_mask |= 1 << channels[n];
            }
            stream_mask = active_channels & ~stream_mask;
            _tree->access<int>("/mboards/0/cm/rx/force_stream").set(stream_mask);
        } else {
            throw uhd::not_implemented_error("Rx force stream not implemented for this device");
        }
    }

    // Sets the destination IP and port of rx to match the tx IP and port
    void rx_to_tx(uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<size_t> rx_channels, std::vector<size_t> tx_channels) override {
        assert(rx_channels.size() == tx_channels.size());
        for(size_t n = 0; n < rx_channels.size(); n++) {
            std::string tx_sfp = tx_usrp->get_tree()->access<std::string>(tx_link_root(tx_channels[n]) / "iface").get();
            std::string tx_ip = tx_usrp->get_tree()->access<std::string>("/mboards/0/link" / tx_sfp / "ip_addr").get();
            _tree->access<std::string>(rx_link_root(rx_channels[n]) / "ip_dest").set(tx_ip);
            std::string tx_port = tx_usrp->get_tree()->access<std::string>(tx_link_root(tx_channels[n]) / "port").get();
            _tree->access<std::string>(rx_link_root(rx_channels[n]) / "port").set(tx_port);

            // Required on Crimson (but not Cyan) so the packets contain a vita header
            _tree->access<std::string>(rx_link_root(rx_channels[n]) / "vita_en").set("1");
        }
    }

    // Enables tx force stream (transmit starting when buffer reaches a desired target)
    void tx_start_force_stream(std::vector<size_t> channels)
    override {
        if (_tree->exists("/mboards/0/cm/tx/force_stream")) {
            int stream_mask = 0;
            for(size_t n = 0; n < channels.size(); n++) {
                // Resets the dsp (clearing the buffer)
                // Resetting is done as part of enabling boards from disabled, but not when enabling if already enabled
                _tree->access<double>(tx_dsp_root(channels[n]) / "rstreq").set(1);
                // Enables boards
                _tree->access<std::string>(tx_root(channels[n]) / "pwr").set("1");
                //when seting cm force stream, bit 0 corresponds to chA, bit 1 to chB, etc
                stream_mask |= 1 << channels[n];
            }
            _tree->access<int>("/mboards/0/cm/tx/force_stream").set(stream_mask);
        } else {
            throw uhd::not_implemented_error("Tx force stream not implemented for this device");
        }
    }

    // Disables tx force stream (transmit starting when buffer reaches a desired target)
    void tx_stop_force_stream(std::vector<size_t> channels)
    override {
        if (_tree->exists("/mboards/0/cm/tx/force_stream")) {
            int active_channels = _tree->access<int>("/mboards/0/cm/tx/force_stream").get();
            int new_channel_mask = 0;
            for(size_t n = 0; n < channels.size(); n++) {
                // Disables boards
                _tree->access<std::string>(tx_root(channels[n]) / "pwr").set("0");
                //when seting cm force stream, bit 0 corresponds to chA, bit 1 to chB, etc
                new_channel_mask |= 1 << channels[n];
            }
            int stream_mask = active_channels & ~new_channel_mask;
            _tree->access<int>("/mboards/0/cm/tx/force_stream").set(stream_mask);
        } else {
            throw uhd::not_implemented_error("Tx force stream not implemented for this device");
        }
    }

    /*******************************************************************
     * GPIO methods
     ******************************************************************/
    std::vector<std::string> get_gpio_banks(const size_t mboard) override
    {
        std::vector<std::string> banks;
        if (_tree->exists(mb_root(mboard) / "gpio")) {
            for (const std::string& name : _tree->list(mb_root(mboard) / "gpio")) {
                banks.push_back(name);
            }
        }
        for (const std::string& name : _tree->list(mb_root(mboard) / "dboards")) {
            banks.push_back("RX" + name);
            banks.push_back("TX" + name);
        }
        return banks;
    }

    void set_gpio_attr(const std::string& bank,
        const std::string& attr,
        const uint32_t value,
        const uint32_t mask,
        const size_t mboard) override
    {
        std::vector<std::string> attr_value;
        if (_tree->exists(mb_root(mboard) / "gpio" / bank)) {
            if (_tree->exists(mb_root(mboard) / "gpio" / bank / attr)) {
                const auto attr_type = gpio_atr::gpio_attr_rev_map.at(attr);
                switch (attr_type) {
                    case gpio_atr::GPIO_SRC:
                        throw uhd::runtime_error(
                            "Can't set SRC attribute using integer value!");
                        break;
                    case gpio_atr::GPIO_CTRL:
                    case gpio_atr::GPIO_DDR: {
                        attr_value = _tree
                                         ->access<std::vector<std::string>>(
                                             mb_root(mboard) / "gpio" / bank / attr)
                                         .get();
                        UHD_ASSERT_THROW(attr_value.size() <= 32);
                        std::bitset<32> bit_mask  = std::bitset<32>(mask);
                        std::bitset<32> bit_value = std::bitset<32>(value);
                        for (size_t i = 0; i < bit_mask.size(); i++) {
                            if (bit_mask[i] == 1) {
                                attr_value[i] = gpio_atr::attr_value_map.at(attr_type).at(
                                    bit_value[i]);
                            }
                        }
                        _tree
                            ->access<std::vector<std::string>>(
                                mb_root(mboard) / "gpio" / bank / attr)
                            .set(attr_value);
                    } break;
                    default: {
                        const uint32_t current =
                            _tree->access<uint32_t>(
                                     mb_root(mboard) / "gpio" / bank / attr)
                                .get();
                        const uint32_t new_value = (current & ~mask) | (value & mask);
                        _tree->access<uint32_t>(mb_root(mboard) / "gpio" / bank / attr)
                            .set(new_value);
                    } break;
                }
                return;
            } else {
                throw uhd::runtime_error(
                    std::format("The hardware has no gpio attribute: `{}':\n", attr));
            }
        }
        if (bank.size() > 2 and bank[1] == 'X') {
            const std::string name          = bank.substr(2);
            const dboard_iface::unit_t unit = (bank[0] == 'R') ? dboard_iface::UNIT_RX
                                                               : dboard_iface::UNIT_TX;
            auto iface                      = _tree
                             ->access<dboard_iface::sptr>(
                                 mb_root(mboard) / "dboards" / name / "iface")
                             .get();
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_CTRL))
                iface->set_pin_ctrl(unit, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_DDR))
                iface->set_gpio_ddr(unit, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_OUT))
                iface->set_gpio_out(unit, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_ATR_0X))
                iface->set_atr_reg(
                    unit, gpio_atr::ATR_REG_IDLE, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_ATR_RX))
                iface->set_atr_reg(
                    unit, gpio_atr::ATR_REG_RX_ONLY, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_ATR_TX))
                iface->set_atr_reg(
                    unit, gpio_atr::ATR_REG_TX_ONLY, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_ATR_XX))
                iface->set_atr_reg(
                    unit, gpio_atr::ATR_REG_FULL_DUPLEX, uint16_t(value), uint16_t(mask));
            if (attr == gpio_atr::gpio_attr_map.at(gpio_atr::GPIO_SRC)) {
                throw uhd::runtime_error(
                    "Setting gpio source does not supported in daughter board.");

            }
            return;
        }
        throw uhd::runtime_error(
            std::format("The hardware has no GPIO bank `{}'", bank));
    }

    uint32_t get_gpio_attr(
        const std::string& bank, const std::string& attr, const size_t mboard) override
    {
        std::vector<std::string> str_val;

        if (_tree->exists(mb_root(mboard) / "gpio" / bank)) {
            if (_tree->exists(mb_root(mboard) / "gpio" / bank / attr)) {
                const auto attr_type = gpio_atr::gpio_attr_rev_map.at(attr);
                switch (attr_type) {
                    case gpio_atr::GPIO_SRC:
                        throw uhd::runtime_error(
                            "Can't set SRC attribute using integer value");
                    case gpio_atr::GPIO_CTRL:
                    case gpio_atr::GPIO_DDR: {
                        str_val = _tree
                                      ->access<std::vector<std::string>>(
                                          mb_root(mboard) / "gpio" / bank / attr)
                                      .get();
                        uint32_t val = 0;
                        for (size_t i = 0; i < str_val.size(); i++) {
                            val += usrp::gpio_atr::gpio_attr_value_pair.at(attr).at(
                                       str_val[i])
                                   << i;
                        }
                        return val;
                    }
                    default:
                        return uint32_t(
                            _tree->access<uint32_t>(
                                     mb_root(mboard) / "gpio" / bank / attr)
                                .get());
                }
                return 0;
            } else {
                throw uhd::runtime_error(
                    std::format("The hardware has no gpio attribute: `{}'", attr));
            }
        }
        if (bank.size() > 2 and bank[1] == 'X') {
            const std::string name          = bank.substr(2);
            const dboard_iface::unit_t unit = (bank[0] == 'R') ? dboard_iface::UNIT_RX
                                                               : dboard_iface::UNIT_TX;
            auto iface                      = _tree
                             ->access<dboard_iface::sptr>(
                                 mb_root(mboard) / "dboards" / name / "iface")
                             .get();
            if (attr == "CTRL")
                return iface->get_pin_ctrl(unit);
            if (attr == "DDR")
                return iface->get_gpio_ddr(unit);
            if (attr == "OUT")
                return iface->get_gpio_out(unit);
            if (attr == "ATR_0X")
                return iface->get_atr_reg(unit, gpio_atr::ATR_REG_IDLE);
            if (attr == "ATR_RX")
                return iface->get_atr_reg(unit, gpio_atr::ATR_REG_RX_ONLY);
            if (attr == "ATR_TX")
                return iface->get_atr_reg(unit, gpio_atr::ATR_REG_TX_ONLY);
            if (attr == "ATR_XX")
                return iface->get_atr_reg(unit, gpio_atr::ATR_REG_FULL_DUPLEX);
            if (attr == "READBACK")
                return iface->read_gpio(unit);
        }
        throw uhd::runtime_error(
            std::format("The hardware has no gpio bank `{}'", bank));
    }

    // The next four methods are only for RFNoC devices
    std::vector<std::string> get_gpio_src_banks(const size_t) override

    {
        throw uhd::not_implemented_error(
            "get_gpio_src_banks() not implemented for this motherboard!");
    }

    std::vector<std::string> get_gpio_srcs(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error(
            "get_gpio_srcs() not implemented for this motherboard!");


    }

    std::vector<std::string> get_gpio_src(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error(
            "get_gpio_src() not implemented for this motherboard!");


    }

    void set_gpio_src(
        const std::string&, const std::vector<std::string>&, const size_t) override

    {
        throw uhd::not_implemented_error(
            "set_gpio_src() not implemented for this motherboard!");
    }

    uhd::rfnoc::mb_controller& get_mb_controller(const size_t /*mboard*/) override
    {
        throw uhd::not_implemented_error(
            "get_mb_controller() not supported on this device!");


    }

private:
    device::sptr _dev;
    property_tree::sptr _tree;


    //! Container for spp values set in set_rx_spp()
    std::unordered_map<size_t, size_t> _rx_spp;

    struct mboard_chan_pair
    {
        size_t mboard, chan;
        mboard_chan_pair(void) : mboard(0), chan(0) {}
    };

    mboard_chan_pair rx_chan_to_mcp(size_t chan)
    {
        // Error check for when requesting a channel that does no exist
        // The part about GNU Radio is there because a common mistake in GNU Radio is to have a mismatch between the length of the channel list and the channel count, which GNU Radio does not detect it
        if(chan >= get_rx_num_channels()) {
            throw uhd::index_error("MULTI_USRP: attempted operation involving RX channel " + std::to_string(chan) + " but only " + std::to_string(get_rx_num_channels()) + " channels exist. If you are using GNU Radio Companion ensure that the length of \"Stream channels\" matches \"Num Channels\" in your USRP Source blocks.");
        }

        mboard_chan_pair mcp;
        mcp.chan = chan;
        for (mcp.mboard = 0; mcp.mboard < get_num_mboards(); mcp.mboard++) {
            size_t sss = get_rx_subdev_spec(mcp.mboard).size();
            if (mcp.chan < sss)
                break;
            mcp.chan -= sss;
        }
        if (mcp.mboard >= get_num_mboards()) {
            throw uhd::index_error(
                std::format(
                    "multi_usrp: RX channel {} out of range for configured RX frontends",
                chan));
        }
        return mcp;
    }

    mboard_chan_pair tx_chan_to_mcp(size_t chan)
    {
        // Error check for when requesting a channel that does no exist
        // The part about GNU Radio is there because a common mistake in GNU Radio is to have a mismatch between the length of the channel list and the channel count, which GNU Radio does not detect it
        if(chan >= get_tx_num_channels()) {
            throw uhd::index_error("MULTI_USRP: attempted operation involving TX channel " + std::to_string(chan) + " but only " + std::to_string(get_tx_num_channels()) + " channels exist. If you are using GNU Radio Companion ensure that the length of \"Stream channels\" matches \"Num Channels\" in your USRP Source blocks.");
        }

        mboard_chan_pair mcp;
        mcp.chan = chan;
        for (mcp.mboard = 0; mcp.mboard < get_num_mboards(); mcp.mboard++) {
            size_t sss = get_tx_subdev_spec(mcp.mboard).size();
            if (mcp.chan < sss)
                break;
            mcp.chan -= sss;
        }
        if (mcp.mboard >= get_num_mboards()) {
            throw uhd::index_error(
                std::format(
                    "multi_usrp: TX channel {} out of range for configured TX frontends",
                chan));
        }
        return mcp;
    }

    fs_path mb_root(const size_t mboard)
    {
        try {
            const std::string tree_path = "/mboards/" + std::to_string(mboard);
            if (_tree->exists(tree_path)) {
                return tree_path;
            } else {
                throw uhd::index_error(
                    std::format("multi_usrp::mb_root({}) - path not found", mboard));
            }
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::mb_root({}) - {}", mboard, e.what()));
        }
    }

    // Directory from trigger options that apply globally
    fs_path trigger_root() {
        return mb_root(0) / "trigger";
    }

    fs_path rx_root(const size_t chan)
    {
        return "/mboards/0/rx/" + std::to_string(chan);
    }

    fs_path rx_dsp_root(const size_t chan)
    {
        mboard_chan_pair mcp = rx_chan_to_mcp(chan);

        if (_tree->exists(mb_root(mcp.mboard) / "rx_chan_dsp_mapping")) {
            std::vector<size_t> map = _tree
                                          ->access<std::vector<size_t>>(
                                              mb_root(mcp.mboard) / "rx_chan_dsp_mapping")
                                          .get();
            UHD_ASSERT_THROW(map.size() > mcp.chan);
            mcp.chan = map[mcp.chan];
        }

        try {
            const std::string tree_path = mb_root(mcp.mboard) / "rx_dsps" / mcp.chan;
            if (_tree->exists(tree_path)) {
                return tree_path;
            } else {
                throw uhd::index_error(
                    std::format(
                            "multi_usrp::rx_dsp_root({}) - mcp({}) - path not found",
                        chan, mcp.chan));
            }
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::rx_dsp_root({}) - mcp({}) - {}", chan,
                    mcp.chan, e.what()));
        }
    }

    fs_path tx_root(const size_t chan)
    {
        return "/mboards/0/tx/" + std::to_string(chan);
    }

    fs_path tx_dsp_root(const size_t chan)
    {
        mboard_chan_pair mcp = tx_chan_to_mcp(chan);

        if (_tree->exists(mb_root(mcp.mboard) / "tx_chan_dsp_mapping")) {
            std::vector<size_t> map = _tree
                                          ->access<std::vector<size_t>>(
                                              mb_root(mcp.mboard) / "tx_chan_dsp_mapping")
                                          .get();
            UHD_ASSERT_THROW(map.size() > mcp.chan);
            mcp.chan = map[mcp.chan];
        }
        try {
            const std::string tree_path = mb_root(mcp.mboard) / "tx_dsps" / mcp.chan;
            if (_tree->exists(tree_path)) {
                return tree_path;
            } else {
                throw uhd::index_error(
                    std::format(
                            "multi_usrp::tx_dsp_root({}) - mcp({}) - path not found",
                        chan, mcp.chan));
            }
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::tx_dsp_root({}) - mcp({}) - {}", chan,
                    mcp.chan, e.what()));
        }
    }

    fs_path rx_fe_root(const size_t chan)
    {
        mboard_chan_pair mcp = rx_chan_to_mcp(chan);
        try {
            const subdev_spec_pair_t spec = get_rx_subdev_spec(mcp.mboard).at(mcp.chan);
            return mb_root(mcp.mboard) / "rx_frontends" / spec.db_name;
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::rx_fe_root({}) - mcp({}) - {}", chan,
                    mcp.chan, e.what()));
        }
    }

    fs_path rx_link_root(const size_t chan)
    {
        return "/mboards/0/rx_link/" + std::to_string(chan);
    }

    fs_path tx_fe_root(const size_t chan)
    {
        mboard_chan_pair mcp = tx_chan_to_mcp(chan);
        try {
            const subdev_spec_pair_t spec = get_tx_subdev_spec(mcp.mboard).at(mcp.chan);
            return mb_root(mcp.mboard) / "tx_frontends" / spec.db_name;
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::tx_fe_root({}) - mcp({}) - {}", chan,
                    mcp.chan, e.what()));
        }
    }

    fs_path tx_link_root(const size_t chan)
    {
        return "/mboards/0/tx_link/" + std::to_string(chan);
    }

    fs_path rx_trigger_root(const size_t channel) {
        return rx_root(channel) / "trigger";
    }

    fs_path tx_trigger_root(const size_t channel) {
        return tx_root(channel) / "trigger";
    }

    size_t get_radio_index(const std::string slot_name)
    {

        if (slot_name == "A") {
            return 0;
        } else if (slot_name == "B") {
            return 1;
        } else if (slot_name == "C") {
            return 2;
        } else if (slot_name == "D") {
            return 3;
        } else {
            throw uhd::key_error(
                std::format("[multi_usrp]: radio slot name {} out of supported range.",
                slot_name));
        }
    }

    fs_path rx_rf_fe_root(const size_t chan)
    {
        mboard_chan_pair mcp = rx_chan_to_mcp(chan);
        try {
            const subdev_spec_pair_t spec = get_rx_subdev_spec(mcp.mboard).at(mcp.chan);
            return mb_root(mcp.mboard) / "dboards" / spec.db_name / "rx_frontends"
                   / spec.sd_name;
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::rx_rf_fe_root({}) - mcp({}) - {}", chan,
                    mcp.chan, e.what()));
        }
    }

    fs_path tx_rf_fe_root(const size_t chan)
    {
        mboard_chan_pair mcp = tx_chan_to_mcp(chan);
        try {
            const subdev_spec_pair_t spec = get_tx_subdev_spec(mcp.mboard).at(mcp.chan);
            return mb_root(mcp.mboard) / "dboards" / spec.db_name / "tx_frontends"
                   / spec.sd_name;
        } catch (const std::exception& e) {
            throw uhd::index_error(
                std::format("multi_usrp::tx_rf_fe_root({}) - mcp({}) - {}", chan,
                    mcp.chan, e.what()));
        }
    }

    gain_group::sptr rx_gain_group(size_t chan)
    {
        mboard_chan_pair mcp          = rx_chan_to_mcp(chan);
        const subdev_spec_pair_t spec = get_rx_subdev_spec(mcp.mboard).at(mcp.chan);
        gain_group::sptr gg           = gain_group::make();
        for (const std::string& name :
            _tree->list(mb_root(mcp.mboard) / "rx_codecs" / spec.db_name / "gains")) {
            gg->register_fcns("ADC-" + name,
                make_gain_fcns_from_subtree(_tree->subtree(
                    mb_root(mcp.mboard) / "rx_codecs" / spec.db_name / "gains" / name)),
                0 /* low prio */);
        }
        for (const std::string& name : _tree->list(rx_rf_fe_root(chan) / "gains")) {
            gg->register_fcns(name,
                make_gain_fcns_from_subtree(
                    _tree->subtree(rx_rf_fe_root(chan) / "gains" / name)),
                1 /* high prio */);
        }
        return gg;
    }

    gain_group::sptr tx_gain_group(size_t chan)
    {
        mboard_chan_pair mcp          = tx_chan_to_mcp(chan);
        const subdev_spec_pair_t spec = get_tx_subdev_spec(mcp.mboard).at(mcp.chan);
        gain_group::sptr gg           = gain_group::make();
        for (const std::string& name :
            _tree->list(mb_root(mcp.mboard) / "tx_codecs" / spec.db_name / "gains")) {
            gg->register_fcns("DAC-" + name,
                make_gain_fcns_from_subtree(_tree->subtree(
                    mb_root(mcp.mboard) / "tx_codecs" / spec.db_name / "gains" / name)),
                1 /* high prio */);
        }
        for (const std::string& name : _tree->list(tx_rf_fe_root(chan) / "gains")) {
            gg->register_fcns(name,
                make_gain_fcns_from_subtree(
                    _tree->subtree(tx_rf_fe_root(chan) / "gains" / name)),
                0 /* low prio */);
        }
        return gg;
    }

    // Generic tree setters and getters.
    void set_tree_value(const std::string path, const std::string value) {
        _tree->access<std::string>(path).set(value);
    }
    void set_tree_value(const std::string path, const double value) {
        _tree->access<double>(path).set(value);
    }
    void set_tree_value(const std::string path, const int value) {
        _tree->access<int>(path).set(value);
    }
    void set_tree_value(const std::string path, const time_spec_t value) {
        _tree->access<time_spec_t>(path).set(value);
    }
    void set_tree_value(const std::string path, const bool value) {
        _tree->access<bool>(path).set(value);
    }
    void set_tree_value(const std::string path, const stream_cmd_t value) {
        _tree->access<stream_cmd_t>(path).set(value);
    }

    void get_tree_value(const std::string path, std::string& value) {
        value = _tree->access<std::string>(path).get();
    }
    void get_tree_value(const std::string path, double& value) {
        value = _tree->access<double>(path).get();
    }
    void get_tree_value(const std::string path, int& value) {
        value = _tree->access<int>(path).get();
    }
    void get_tree_value(const std::string path, time_spec_t& value) {
        value = _tree->access<time_spec_t>(path).get();
    }
    void get_tree_value(const std::string path, bool& value) {
        value = _tree->access<bool>(path).get();
    }
    void get_tree_value(const std::string path, stream_cmd_t& value) {
        value = _tree->access<stream_cmd_t>(path).get();
    }

    void dump_tree(const std::string root)
    {
        for(auto& path : _tree->list(root))
        {
            const std::string full = root + "/" + path;
            std::cout << full << std::endl;
            dump_tree(full);
        }
    }

// Per Vices-specific functions. Adding to the end separately from the first public block
// for compatibility and to make clear this was added by us, not upstream.
public:

    void set_tx_lo_power(int lo_power, const std::string &name=ALL_LOS, const size_t chan=0) override {
        // Upstream functions check the "<tx_rf_fe_root>/los" path to see if LO API is enabled.
        // To avoid enabling upstream, we just store the properties for each controllable LO under "tx_rf_fe_root/<LO_NAME>".
        // Otherwise, follow similar flow to upstream LO API functions.
        if (name == ALL_LOS) {
            for (const auto& n: this->get_tx_lo_names(chan)) {
                this->set_tx_lo_power(lo_power, n, chan);
            }
        } else {
            // <LO_NAME> path only exists if the device supports LO control through the API
            if (_tree->exists(tx_rf_fe_root(chan) / name)) {
                _tree->access<int>(tx_rf_fe_root(chan) / name / "lo_pwr").set(lo_power);
                // Warn the user if the value from the device does not match what they tried to set but do not throw error.
                // The device will clip the value if it is out of range, so just let the user know the actual value like we do with rate or frequency.
                int actual_power = this->get_tx_lo_power(name, chan);
                if (actual_power != lo_power) {
                    UHD_LOGGER_WARNING("MULTI_USRP") << std::format(
                        "The hardware does not support the requested LO power on channel {}:\n"
                        "Target LO power: {}\n"
                        "Actual LO power: {}\n",
                        chan, lo_power, actual_power);
                }
            } else {
                throw uhd::runtime_error("Could not find LO stage " + name);
            }
        }
    }

    int get_tx_lo_power(const std::string &name=ALL_LOS, const size_t chan=0) override {
        if (name == ALL_LOS) {
            throw uhd::runtime_error("LO power must be retrieved for each stage individually");
        } else {
            return _tree->access<int>(tx_rf_fe_root(chan) / name / "lo_pwr").get();
        }
    }

private:
    /***********************************************************************
     * trigger_configure implementation
     *
     * trigger_configure() works on deep copies of the caller's structs in the
     * library's layout (the "request"). It runs in two stages:
     *  1. Validation. Every argument is checked before anything is written to
     *     the device. The only device access allowed is reading the current
     *     SMA direction of a toggleable SMA pin (see trigger_plan_sma()), which
     *     happens after all other validation.
     *  2. Apply (trigger_apply()). Settings are written in the order described
     *     by setup_mode_t::standard, and the values actually in effect are
     *     written back into the request.
     * The caller's structs are only written to once both stages succeed.
     *
     * Error convention for trigger_configure and its helpers:
     *  - uhd::value_error: the requested settings are invalid, alone, in
     *    combination, or (for the SMA direction read during validation) given
     *    the device's current settings. Thrown only during validation.
     *  - uhd::index_error: a requested channel does not exist. Thrown only
     *    during validation.
     *  - uhd::io_error: reading or writing a property failed. See
     *    detail::property_get_string() and friends, which also log and rethrow
     *    uhd::lookup_error from servers that are incompatible with this version
     *    of UHD.
     *  - uhd::runtime_error: anything else, e.g. a strict coerced_value error, a
     *    value from the device that cannot be parsed, an unsupported server, or
     *    a check that validation should have made unreachable.
     **********************************************************************/

    //! The SMA and U.FL pins on every Per Vices device have these types.
    using trigger_pin_type_t = uhd::pv_device::trigger_pin_type_t;

    //! U.FL pins are toggleable on every Per Vices device.
    static constexpr trigger_pin_type_t trigger_ufl_pin_type = trigger_pin_type_t::toggleable;

    //! What trigger_apply() should do with one pin's direction.
    struct trigger_pin_plan_t
    {
        //! pin_direction_t::in or ::out to write, or ::get to leave the pin alone
        uhd::trigger_pv::pin_direction_t write = uhd::trigger_pv::pin_direction_t::get;
        //! Read the pin's direction and report it back (the field was get)
        bool read = false;
        //! Value to report back instead of reading, e.g. separate on separate pins
        std::optional<uhd::trigger_pv::pin_direction_t> report;
    };

    //! Library-format state of one trigger_configure() call.
    struct trigger_request_t
    {
        uhd::trigger_pv::trigger_op_t op;
        //! Empty if the caller passed no global settings
        std::optional<uhd::trigger_pv::trigger_global_t> global;
        std::vector<uhd::trigger_pv::trigger_rx_t> rx;
        std::vector<uhd::trigger_pv::trigger_tx_t> tx;
        std::vector<size_t> unspecified_rx;
        std::vector<size_t> unspecified_tx;

        //! What to do with global->sma_direction
        trigger_pin_plan_t sma;
        //! The SMA direction in effect on the device, once known (in, out or separate)
        std::optional<uhd::trigger_pv::pin_direction_t> sma_in_effect;
        //! One element per element of rx / tx: what to do with ufl_dir
        std::vector<trigger_pin_plan_t> rx_ufl;
        std::vector<trigger_pin_plan_t> tx_ufl;

        //! Whether the server has the TX stream property (TRIGGER_OVERHAUL_SERVER_R3_S).
        //! Without it TX channels cannot be disabled without powering down their RF chain,
        //! so they are not disabled.
        bool tx_stream_supported = false;
    };

    /*******************************************************************
     * Small shared helpers
     ******************************************************************/

    //! Per Vices device interface, or throw if this is not a Per Vices device.
    uhd::pv_device& trigger_pv_device()
    {
        auto* const dev = dynamic_cast<uhd::pv_device*>(_dev.get());
        if (dev == nullptr) {
            throw uhd::not_implemented_error(
                "This fork is only intended for Per Vices devices. If you are not using a "
                "Per Vices product use upstream (Ettus) UHD. If you encountered this error "
                "despite using a Per Vices product, recompile to fix possible ABI "
                "compatibility issues.");
        }
        return *dev;
    }

    //! "rx[2] (channel 5)": names one element of a request in error messages.
    template <typename channel_t>
    static std::string trigger_element_name(const channel_t& chan, size_t channel_index)
    {
        constexpr const char* name =
            std::is_same_v<channel_t, uhd::trigger_pv::trigger_rx_t> ? "rx" : "tx";
        return std::format("{}[{}] (channel {})", name, channel_index, chan.channel);
    }

    //! True if trig_sel is a real selection (not get or bypass), so fields that
    //! depend on it can count as relevant.
    static bool trigger_sel_counts(const uhd::trigger_pv::active_trigger_t trig_sel)
    {
        using namespace uhd::trigger_pv;
        return trig_sel != active_trigger_t::get && trig_sel != active_trigger_t::bypass;
    }

    //! True if trig_sel counts (see trigger_sel_counts()) and includes `bit`.
    static bool trigger_sel_includes(
        const uhd::trigger_pv::active_trigger_t trig_sel, const uhd::trigger_pv::active_trigger_t bit)
    {
        return trigger_sel_counts(trig_sel) && has_any(trig_sel & bit);
    }

    //! Relevance of edge_backoff and edge_sample_num (see their documentation).
    template <typename channel_t>
    static bool trigger_edge_input_relevant(const channel_t& chan)
    {
        using namespace uhd::trigger_pv;
        const bool sma_edge = trigger_sel_includes(chan.trig_sel, active_trigger_t::sma_input)
                              && chan.sma_mode == trigger_mode_t::edge;
        const bool ufl_edge = trigger_sel_includes(chan.trig_sel, active_trigger_t::channel_ufl)
                              && chan.ufl_dir == pin_direction_t::in
                              && chan.ufl_mode == trigger_mode_t::edge;
        return sma_edge || ufl_edge;
    }

    //! Relevance of ufl_output_pulse_length (see its documentation).
    template <typename channel_t>
    static bool trigger_ufl_output_relevant(const channel_t& chan)
    {
        using namespace uhd::trigger_pv;
        return trigger_sel_includes(chan.trig_sel, active_trigger_t::channel_ufl)
               && chan.ufl_dir == pin_direction_t::out
               && chan.ufl_mode == trigger_mode_t::edge;
    }

    //! Relevance of sma_output_pulse_length for one channel (see its documentation).
    template <typename channel_t>
    static bool trigger_sma_output_relevant(const channel_t& chan)
    {
        using namespace uhd::trigger_pv;
        return trigger_sel_includes(chan.trig_sel, active_trigger_t::sma_output)
               && chan.sma_mode == trigger_mode_t::edge;
    }

    //! Log a warning unless the caller disabled warnings of this category.
    static void trigger_warn(const uhd::trigger_pv::trigger_op_t& op,
        const uhd::trigger_pv::warning_t category,
        const std::string& message)
    {
        if (!has_any(op.disable_warnings & category)) {
            UHD_LOG_WARNING("MULTI_USRP", message);
        }
    }

    /*!
     * Handle a setting the device applied differently from the request: throw
     * uhd::runtime_error if strict coerced_value errors are enabled, otherwise
     * log a warning unless coerced_value warnings are disabled.
     * Not a uhd::value_error: the setting has already been changed on the device.
     */
    static void trigger_handle_coerced_value(const fs_path& path,
        const std::string& requested,
        const std::string& actual,
        const uhd::trigger_pv::trigger_op_t& op)
    {
        using namespace uhd::trigger_pv;

        const std::string message = std::format(
            "{}: requested {} but the device applied {}", std::string(path), requested, actual);
        if (has_any(op.enable_strict_errors & strict_error_t::coerced_value)) {
            throw uhd::runtime_error(message);
        }
        trigger_warn(op, warning_t::coerced_value, message);
    }

    /*******************************************************************
     * Validation
     ******************************************************************/

    //! Throw uhd::value_error unless `value` is one of an enum's named values:
    //! 0 to max_value (every trigger enum except the flag enums is contiguous),
    //! or E::bypass for enums that have it.
    template <typename E>
    static void trigger_check_enum(const E value, const E max_value, const std::string& name)
    {
        using U = std::underlying_type_t<E>;
        if constexpr (requires { E::bypass; }) {
            if (value == E::bypass) {
                return;
            }
        }
        if (static_cast<U>(value) > static_cast<U>(max_value)) {
            throw uhd::value_error(std::format("{}: invalid value {}", name,
                static_cast<uint64_t>(static_cast<U>(value))));
        }
    }

    //! Throw uhd::value_error unless a reserved field is zero.
    template <typename T>
    static void trigger_check_reserved(const T& reserved, const std::string& name)
    {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&reserved);
        for (size_t i = 0; i < sizeof(T); i++) {
            if (bytes[i] != 0) {
                throw uhd::value_error(std::format("{} must be zero", name));
            }
        }
    }

    /*!
     * Validate an integer field that uses GET_INT and, optionally,
     * SKIP_IF_NOT_RELEVANT as control values.
     *
     * \param value the requested value
     * \param max_value largest regular value (regular values start at 0)
     * \param skip_supported whether the field supports SKIP_IF_NOT_RELEVANT
     * \param relevant whether the field is relevant (only used for SKIP_IF_NOT_RELEVANT)
     * \param relevance why the field is relevant, for the error message
     * \param name the field's name in error messages
     */
    static void trigger_check_integer(const int64_t value,
        const int64_t max_value,
        const bool skip_supported,
        const bool relevant,
        const char* relevance,
        const std::string& name)
    {
        using namespace uhd::trigger_pv;

        if (value == GET_INT) {
            return;
        }
        if (value == SKIP_IF_NOT_RELEVANT) {
            if (!skip_supported) {
                throw uhd::value_error(
                    std::format("{}: SKIP_IF_NOT_RELEVANT is not supported", name));
            }
            if (relevant) {
                throw uhd::value_error(std::format(
                    "{} is SKIP_IF_NOT_RELEVANT, but it is relevant because {}", name, relevance));
            }
            return;
        }
        if (value < 0 || value > max_value) {
            throw uhd::value_error(std::format("{}: value {} is outside the valid range 0 to {}",
                name, value, max_value));
        }
    }

    //! Validate trig_sel on its own (see active_trigger_t).
    static void trigger_check_trig_sel(
        const uhd::trigger_pv::active_trigger_t trig_sel, const std::string& name)
    {
        using namespace uhd::trigger_pv;

        if (trig_sel == active_trigger_t::get || trig_sel == active_trigger_t::bypass) {
            return;
        }
        if (has_any(trig_sel & (active_trigger_t::get | active_trigger_t::bypass))) {
            throw uhd::value_error(std::format(
                "{}.trig_sel: get and bypass cannot be combined with anything else", name));
        }
        const active_trigger_t known =
            active_trigger_t::sma_input | active_trigger_t::channel_ufl | active_trigger_t::sma_output;
        if (has_any(trig_sel & ~known)) {
            throw uhd::value_error(std::format("{}.trig_sel: unknown bits 0x{:x}", name,
                static_cast<uint32_t>(trig_sel & ~known)));
        }
        if (has_any(trig_sel & active_trigger_t::sma_input)
            && has_any(trig_sel & active_trigger_t::sma_output)) {
            throw uhd::value_error(std::format(
                "{}.trig_sel: sma_input and sma_output cannot be combined on the same channel",
                name));
        }
    }

    /*!
     * Validate the direction of a pin whose direction cannot be derived, and
     * work out what trigger_apply() should do with it. Used for the U.FL.
     */
    static trigger_pin_plan_t trigger_plan_underivable_pin(const uhd::trigger_pv::pin_direction_t dir,
        const trigger_pin_type_t pin_type,
        const bool relevant,
        const char* relevance,
        const uhd::trigger_pv::trigger_op_t& op,
        const std::string& name)
    {
        using namespace uhd::trigger_pv;

        trigger_pin_plan_t plan;
        switch (dir) {
            case pin_direction_t::must_specify_if_applicable:
                if (relevant) {
                    throw uhd::value_error(std::format(
                        "{} must be specified because {}", name, relevance));
                }
                return plan;
            case pin_direction_t::get:
                return trigger_plan_fixed_pin_get(pin_type);
            case pin_direction_t::in:
            case pin_direction_t::out:
                return trigger_plan_in_out(dir, pin_type, op, name);
            case pin_direction_t::derive:
                throw uhd::value_error(
                    std::format("{}: derive is not supported for this pin", name));
            case pin_direction_t::separate:
                return trigger_plan_separate(pin_type, name);
        }
        throw uhd::value_error(
            std::format("{}: invalid value {}", name, static_cast<uint32_t>(dir)));
    }

    //! Plan for pin_direction_t::get. Only toggleable pins need a read.
    static trigger_pin_plan_t trigger_plan_fixed_pin_get(const trigger_pin_type_t pin_type)
    {
        using namespace uhd::trigger_pv;

        trigger_pin_plan_t plan;
        switch (pin_type) {
            case trigger_pin_type_t::toggleable:
                plan.read = true;
                break;
            case trigger_pin_type_t::separate:
                plan.report = pin_direction_t::separate;
                break;
            case trigger_pin_type_t::input_only:
                plan.report = pin_direction_t::in;
                break;
            case trigger_pin_type_t::output_only:
                plan.report = pin_direction_t::out;
                break;
        }
        return plan;
    }

    //! Plan for pin_direction_t::in or ::out, apart from conflicts with other fields.
    static trigger_pin_plan_t trigger_plan_in_out(const uhd::trigger_pv::pin_direction_t dir,
        const trigger_pin_type_t pin_type,
        const uhd::trigger_pv::trigger_op_t& op,
        const std::string& name)
    {
        using namespace uhd::trigger_pv;

        const char* const dir_name = (dir == pin_direction_t::in) ? "in" : "out";
        trigger_pin_plan_t plan;
        switch (pin_type) {
            case trigger_pin_type_t::toggleable:
                plan.write = dir;
                return plan;
            case trigger_pin_type_t::separate: {
                const std::string message = std::format(
                    "{} is {}, but this device has separate input and output pins", name, dir_name);
                if (has_any(op.enable_strict_errors & strict_error_t::pedantic_dir)) {
                    throw uhd::value_error(message);
                }
                trigger_warn(op, warning_t::pedantic_dir, message + "; treated as get");
                plan.report = pin_direction_t::separate;
                return plan;
            }
            case trigger_pin_type_t::input_only:
            case trigger_pin_type_t::output_only: {
                const pin_direction_t fixed = (pin_type == trigger_pin_type_t::input_only)
                                                  ? pin_direction_t::in
                                                  : pin_direction_t::out;
                if (dir != fixed) {
                    throw uhd::value_error(std::format(
                        "{} is {}, but this pin can only be the other direction", name, dir_name));
                }
                plan.report = fixed;
                return plan;
            }
        }
        return plan;
    }

    //! Plan for pin_direction_t::separate.
    static trigger_pin_plan_t trigger_plan_separate(
        const trigger_pin_type_t pin_type, const std::string& name)
    {
        using namespace uhd::trigger_pv;

        if (pin_type != trigger_pin_type_t::separate) {
            throw uhd::value_error(std::format(
                "{} is separate, but this device does not have separate input and output pins",
                name));
        }
        trigger_pin_plan_t plan;
        plan.report = pin_direction_t::separate;
        return plan;
    }

    /*!
     * Validate everything in one channel's request that does not involve the
     * SMA direction, and plan its U.FL direction.
     */
    template <typename channel_t>
    static trigger_pin_plan_t trigger_validate_channel(const channel_t& chan,
        const size_t channel_index,
        const uhd::trigger_pv::trigger_op_t& op)
    {
        using namespace uhd::trigger_pv;

        const std::string name = trigger_element_name(chan, channel_index);
        trigger_check_reserved(chan.reserved0, name + ".reserved0");
        trigger_check_reserved(chan.reserved1, name + ".reserved1");
        if constexpr (std::is_same_v<channel_t, trigger_tx_t>) {
            trigger_check_reserved(chan.reserved2, name + ".reserved2");
            trigger_check_enum(chan.gating, gating_t::output, name + ".gating");
        }

        trigger_check_trig_sel(chan.trig_sel, name);
        trigger_check_enum(chan.sma_mode, trigger_mode_t::level, name + ".sma_mode");
        trigger_check_enum(chan.ufl_mode, trigger_mode_t::level, name + ".ufl_mode");
        trigger_check_enum(chan.ufl_polarity, polarity_t::negative, name + ".ufl_polarity");

        constexpr const char* edge_input_relevance =
            "trig_sel includes sma_input with sma_mode set to edge, or channel_ufl with "
            "ufl_dir set to in and ufl_mode set to edge";
        trigger_check_integer(chan.edge_backoff, UINT32_MAX, true,
            trigger_edge_input_relevant(chan), edge_input_relevance, name + ".edge_backoff");
        trigger_check_integer(chan.edge_sample_num, INT64_MAX, true,
            trigger_edge_input_relevant(chan), edge_input_relevance, name + ".edge_sample_num");
        trigger_check_integer(chan.ufl_output_pulse_length, INT64_MAX, true,
            trigger_ufl_output_relevant(chan),
            "trig_sel includes channel_ufl with ufl_dir set to out and ufl_mode set to edge",
            name + ".ufl_output_pulse_length");
        trigger_check_integer(chan.enable, 1, false, false, "", name + ".enable");

        return trigger_plan_underivable_pin(chan.ufl_dir, trigger_ufl_pin_type,
            trigger_sel_includes(chan.trig_sel, active_trigger_t::channel_ufl),
            "trig_sel includes channel_ufl", op, name + ".ufl_dir");
    }

    //! The SMA direction the channels in a request need (see pin_direction_t::derive).
    struct trigger_sma_usage_t
    {
        bool input  = false; //!< At least one channel listens to the SMA input
        bool output = false; //!< At least one channel drives the SMA output
    };

    template <typename channel_t>
    static void trigger_add_sma_usage(
        const std::vector<channel_t>& channels, trigger_sma_usage_t& usage)
    {
        using namespace uhd::trigger_pv;
        for (const channel_t& chan : channels) {
            usage.input  = usage.input || trigger_sel_includes(chan.trig_sel, active_trigger_t::sma_input);
            usage.output = usage.output || trigger_sel_includes(chan.trig_sel, active_trigger_t::sma_output);
        }
    }

    /*!
     * Validate the SMA direction against the request and plan what to do with
     * it. On a toggleable pin whose direction is not being set (no global
     * settings, or sma_direction is get) while channels use the SMA, this reads
     * the current direction: the one device access allowed during validation.
     * Must therefore be called after all other validation.
     */
    void trigger_plan_sma(trigger_request_t& request, const trigger_pin_type_t pin_type)
    {
        using namespace uhd::trigger_pv;

        trigger_sma_usage_t usage;
        trigger_add_sma_usage(request.rx, usage);
        trigger_add_sma_usage(request.tx, usage);

        const bool used = usage.input || usage.output;
        pin_direction_t implied = pin_direction_t::get; // get: no channel uses the SMA
        if (usage.input) {
            implied = pin_direction_t::in;
        } else if (usage.output) {
            implied = pin_direction_t::out;
        }
        const char* const implied_name = (implied == pin_direction_t::in) ? "in" : "out";

        if (pin_type == trigger_pin_type_t::toggleable && usage.input && usage.output) {
            throw uhd::value_error(
                "This device has a single SMA pin whose direction applies to every channel, but "
                "channels in this request use it both as an input and as an output");
        }

        // Reads the current direction during validation and checks the channels against it.
        auto check_against_current = [&](const char* why) {
            const pin_direction_t current = detail::direction_from_device_string(
                detail::property_get_string(*_tree, trigger_root() / "sma_dir"));
            request.sma_in_effect = current;
            if (current != implied) {
                throw uhd::value_error(std::format(
                    "{}, so the existing SMA direction ({}) was kept, but channels in this "
                    "request use the SMA as {}. Set sma_direction to derive to change it",
                    why, detail::to_device_string(current), implied_name));
            }
            return current;
        };

        if (!request.global.has_value()) {
            // The direction is never changed without global settings, but it is
            // still validated.
            if (pin_type == trigger_pin_type_t::toggleable && used) {
                check_against_current("No global settings were given");
            }
            return;
        }

        const pin_direction_t requested = request.global->sma_direction;
        const std::string name          = "global.sma_direction";
        switch (requested) {
            case pin_direction_t::must_specify_if_applicable:
                if (used) {
                    throw uhd::value_error(std::format(
                        "{} must be specified because channels in this request use the SMA", name));
                }
                return;

            case pin_direction_t::get:
                request.sma = trigger_plan_fixed_pin_get(pin_type);
                if (pin_type == trigger_pin_type_t::toggleable && used) {
                    // Read now for validation; the value read is also the result of the get.
                    request.sma.read   = false;
                    request.sma.report = check_against_current("sma_direction is get");
                }
                return;

            case pin_direction_t::in:
            case pin_direction_t::out:
                request.sma = trigger_plan_in_out(requested, pin_type, request.op, name);
                if (pin_type == trigger_pin_type_t::toggleable && used && implied != requested) {
                    throw uhd::value_error(std::format(
                        "{} is {}, but channels in this request use the SMA as {}. Use derive "
                        "to set it from the channels",
                        name, (requested == pin_direction_t::in) ? "in" : "out", implied_name));
                }
                return;

            case pin_direction_t::derive:
                // Returned unchanged. Only toggleable pins have a direction to set.
                if (pin_type == trigger_pin_type_t::toggleable && used) {
                    request.sma.write = implied;
                }
                return;

            case pin_direction_t::separate:
                request.sma = trigger_plan_separate(pin_type, name);
                return;
        }
        throw uhd::value_error(
            std::format("{}: invalid value {}", name, static_cast<uint32_t>(requested)));
    }

    /*!
     * Stage 1: validate the request. See the error convention above.
     * Fills in the plans in `request`.
     */
    /*!
     * Handle an output pulse length on a server older than
     * TRIGGER_OVERHAUL_SERVER_R2_S, which does not have the property.
     * Called after the field has been validated, so a SKIP_IF_NOT_RELEVANT value
     * here is not relevant.
     *  - SKIP_IF_NOT_RELEVANT: nothing to do.
     *  - GET_INT with strict_error_t::unsupported_server enabled: reported as
     *    -ENOSYS without accessing the device.
     *  - Anything else (a value to set, or GET_INT without the flag): uhd::runtime_error.
     */
    static void trigger_check_pulse_length_supported(int64_t& value,
        const uhd::trigger_pv::trigger_op_t& op,
        const std::string& name,
        const int server_api_ver)
    {
        using namespace uhd::trigger_pv;

        if (value == SKIP_IF_NOT_RELEVANT) {
            return;
        }
        if (value == GET_INT && has_any(op.enable_strict_errors & strict_error_t::unsupported_server)) {
            value = -ENOSYS;
            return;
        }
        throw uhd::runtime_error(std::format(
            "{}: Setting the trigger output pulse duration is not supported if unsupported_server "
            "warnings are not suppressed. The server's API version ({}) is older than {}.",
            name, server_api_ver, TRIGGER_OVERHAUL_SERVER_R2_S));
    }

    void trigger_validate(trigger_request_t& request, const uhd::pv_device& dev)
    {
        using namespace uhd::trigger_pv;

        const trigger_op_t& op = request.op;
        trigger_check_reserved(op.reserved0, "op.reserved0");
        trigger_check_enum(op.setup_mode, setup_mode_t::standard, "op.setup_mode");
        trigger_check_enum(op.failure_handling, failure_handling_t::flexible, "op.failure_handling");
        // enable_strict_errors and disable_warnings ignore unknown bits.

        for (size_t i = 0; i < request.rx.size(); i++) {
            request.rx_ufl.push_back(trigger_validate_channel(request.rx[i], i, op));
        }
        for (size_t i = 0; i < request.tx.size(); i++) {
            request.tx_ufl.push_back(trigger_validate_channel(request.tx[i], i, op));
        }

        if (request.global.has_value()) {
            const trigger_global_t& global = *request.global;
            trigger_check_reserved(global.reserved0, "global.reserved0");
            trigger_check_enum(global.sma_direction, pin_direction_t::separate, "global.sma_direction");
            trigger_check_enum(global.sma_polarity, polarity_t::negative, "global.sma_polarity");
            trigger_check_integer(global.sma_override, 1, false, false, "", "global.sma_override");

            bool sma_output_relevant = false;
            for (const trigger_rx_t& rx : request.rx) {
                sma_output_relevant = sma_output_relevant || trigger_sma_output_relevant(rx);
            }
            for (const trigger_tx_t& tx : request.tx) {
                sma_output_relevant = sma_output_relevant || trigger_sma_output_relevant(tx);
            }
            trigger_check_integer(global.sma_output_pulse_length, INT64_MAX, true,
                sma_output_relevant,
                "a channel in this request has sma_output in trig_sel and sma_mode set to edge",
                "global.sma_output_pulse_length");
        }

        // Not an argument error, so checked after the arguments, but before the
        // SMA direction read so an unsupported server is not accessed.
        if (dev.server_api_ver < TRIGGER_OVERHAUL_SERVER_R1_S) {
            const std::string message = std::format(
                "The server's API version ({}) is not supported by trigger_configure. Update "
                "the server.",
                dev.server_api_ver);
            if (has_any(op.enable_strict_errors & strict_error_t::unsupported_server)) {
                throw uhd::runtime_error(message);
            }
            trigger_warn(op, warning_t::unsupported_server, message);
        }

        // The output pulse length properties were added in TRIGGER_OVERHAUL_SERVER_R2_S
        if (dev.server_api_ver < TRIGGER_OVERHAUL_SERVER_R2_S) {
            for (size_t i = 0; i < request.rx.size(); i++) {
                trigger_check_pulse_length_supported(request.rx[i].ufl_output_pulse_length, op,
                    trigger_element_name(request.rx[i], i) + ".ufl_output_pulse_length", dev.server_api_ver);
            }
            for (size_t i = 0; i < request.tx.size(); i++) {
                trigger_check_pulse_length_supported(request.tx[i].ufl_output_pulse_length, op,
                    trigger_element_name(request.tx[i], i) + ".ufl_output_pulse_length", dev.server_api_ver);
            }
            if (request.global.has_value()) {
                trigger_check_pulse_length_supported(request.global->sma_output_pulse_length, op,
                    "global.sma_output_pulse_length", dev.server_api_ver);
            }
        }

        // The TX stream property was added in TRIGGER_OVERHAUL_SERVER_R3_S
        request.tx_stream_supported = (dev.server_api_ver >= TRIGGER_OVERHAUL_SERVER_R3_S);

        // Last: may read the current SMA direction.
        trigger_plan_sma(request, dev.trigger_sma_pin_type());
    }

    /*******************************************************************
     * Apply: device access
     *
     * Each trigger_apply_*() helper writes one requested value (or reads it,
     * for get) and stores the value actually in effect back in the request.
     * They return true if the value in effect is the one requested (a get or a
     * skip always counts as exact), and false if the device coerced it.
     ******************************************************************/

    //! Read an integer setting and check it is in [0, max_value].
    template <typename property_t>
    int64_t trigger_read_integer(const fs_path& path, const int64_t max_value)
    {
        using namespace uhd::trigger_pv;

        int64_t value;
        if constexpr (std::is_same_v<property_t, int>) {
            value = detail::property_get_int(*_tree, path);
        } else {
            value = detail::parse_device_integer(
                detail::property_get_string(*_tree, path), std::string(path));
        }
        if (value < 0 || value > max_value) {
            throw uhd::runtime_error(std::format(
                "{}: device reported {}, outside the valid range 0 to {}", std::string(path), value,
                max_value));
        }
        return value;
    }

    //! Write an integer setting. property_t is the property's type: int or std::string.
    template <typename property_t>
    void trigger_write_integer(const fs_path& path, const int64_t value)
    {
        using namespace uhd::trigger_pv;

        if constexpr (std::is_same_v<property_t, int>) {
            detail::property_set_int(*_tree, path, static_cast<int>(value));
        } else {
            detail::property_set_string(*_tree, path, std::to_string(value));
        }
    }

    /*!
     * Apply an integer field that uses GET_INT and SKIP_IF_NOT_RELEVANT.
     * property_t is the property's type: int or std::string.
     * A SKIP_IF_NOT_RELEVANT value has already been validated as not relevant.
     */
    template <typename property_t>
    bool trigger_apply_integer(const fs_path& path,
        int64_t& value,
        const int64_t max_value,
        const uhd::trigger_pv::trigger_op_t& op)
    {
        using namespace uhd::trigger_pv;

        if (value == SKIP_IF_NOT_RELEVANT) {
            return true;
        }
        if (value == -ENOSYS) {
            // The server does not have this property. Set by validation, see
            // trigger_check_pulse_length_supported().
            return true;
        }
        if (value == GET_INT) {
            value = trigger_read_integer<property_t>(path, max_value);
            return true;
        }
        if (value < 0 || value > max_value) {
            // Unreachable: validation already rejected this.
            throw uhd::runtime_error(std::format("{}: value {} is outside the valid range 0 to {}",
                std::string(path), value, max_value));
        }
        trigger_write_integer<property_t>(path, value);
        const int64_t actual = trigger_read_integer<property_t>(path, max_value);
        if (actual == value) {
            return true;
        }
        trigger_handle_coerced_value(path, std::to_string(value), std::to_string(actual), op);
        value = actual;
        return false;
    }

    /*!
     * Apply an enum field that the device stores as a string.
     * \param from_device_string converts the device's string to E,
     *        e.g. &detail::polarity_from_device_string
     */
    template <uhd::trigger_pv::detail::enum_with_get E>
    bool trigger_apply_enum(const fs_path& path,
        E& value,
        const uhd::trigger_pv::trigger_op_t& op,
        E (*from_device_string)(const std::string&))
    {
        using namespace uhd::trigger_pv;

        if constexpr (requires { E::bypass; }) {
            if (value == E::bypass) {
                return true; // Never read or written.
            }
        }
        if (value == E::get) {
            value = from_device_string(detail::property_get_string(*_tree, path));
            return true;
        }
        const std::string requested = detail::to_device_string(value);
        detail::property_set_string(*_tree, path, requested);
        // Compare as enums, not strings, so formatting differences in the
        // device's reply (e.g. a trailing newline) aren't treated as coercion.
        const E actual = from_device_string(detail::property_get_string(*_tree, path));
        if (actual == value) {
            return true;
        }
        trigger_handle_coerced_value(path, requested, detail::to_device_string(actual), op);
        value = actual;
        return false;
    }

    //! Write a pin direction (in or out) without checking it.
    void trigger_write_direction(const fs_path& path, const uhd::trigger_pv::pin_direction_t dir)
    {
        using namespace uhd::trigger_pv;

        detail::property_set_string(*_tree, path, detail::to_device_string(dir));
    }

    /*!
     * Write a pin's final direction (in or out) and check it was applied.
     * \param reported in: the field's value; out: the value to report back
     */
    bool trigger_apply_final_direction(const fs_path& path,
        const uhd::trigger_pv::pin_direction_t dir,
        uhd::trigger_pv::pin_direction_t& reported,
        const uhd::trigger_pv::trigger_op_t& op)
    {
        using namespace uhd::trigger_pv;

        trigger_write_direction(path, dir);
        const uhd::trigger_pv::pin_direction_t actual = detail::direction_from_device_string(
            detail::property_get_string(*_tree, path));
        // A derived direction is reported back as derive, whatever was applied.
        const bool report_actual = (reported != uhd::trigger_pv::pin_direction_t::derive);
        if (actual == dir) {
            if (report_actual) {
                reported = actual;
            }
            return true;
        }
        trigger_handle_coerced_value(
            path, detail::to_device_string(dir), detail::to_device_string(actual), op);
        if (report_actual) {
            reported = actual;
        }
        return false;
    }

    /*!
     * The SMA direction in effect on the device, needed to translate trig_sel
     * on toggleable SMA pins. Read from the device if it is not known yet.
     */
    uhd::trigger_pv::pin_direction_t trigger_sma_in_effect(
        trigger_request_t& request, const uhd::pv_device& dev)
    {
        using namespace uhd::trigger_pv;

        if (dev.trigger_sma_pin_type() != trigger_pin_type_t::toggleable) {
            return pin_direction_t::separate;
        }
        if (!request.sma_in_effect.has_value()) {
            request.sma_in_effect = detail::direction_from_device_string(
                detail::property_get_string(*_tree, trigger_root() / "sma_dir"));
        }
        return *request.sma_in_effect;
    }

    //! Read trig_sel from the device and translate it to the API's meaning.
    uhd::trigger_pv::active_trigger_t trigger_read_trig_sel(
        const fs_path& path, trigger_request_t& request, const uhd::pv_device& dev)
    {
        using namespace uhd::trigger_pv;

        const int64_t device_bits = trigger_read_integer<std::string>(path, UINT32_MAX);
        const active_trigger_t trig_sel = dev.trig_sel_from_device(
            static_cast<uint32_t>(device_bits), trigger_sma_in_effect(request, dev));
        const active_trigger_t known =
            active_trigger_t::sma_input | active_trigger_t::channel_ufl | active_trigger_t::sma_output;
        if (has_any(trig_sel & ~known)) {
            throw uhd::runtime_error(std::format(
                "{}: device reported unknown trig_sel bits ({})", std::string(path), device_bits));
        }
        return trig_sel;
    }

    //! Apply trig_sel. bypass is never read or written.
    bool trigger_apply_trig_sel(const fs_path& path,
        uhd::trigger_pv::active_trigger_t& trig_sel,
        trigger_request_t& request,
        const uhd::pv_device& dev)
    {
        using namespace uhd::trigger_pv;

        if (trig_sel == active_trigger_t::bypass) {
            return true;
        }
        if (trig_sel == active_trigger_t::get) {
            trig_sel = trigger_read_trig_sel(path, request, dev);
            return true;
        }
        const uint32_t device_bits = dev.trig_sel_to_device(trig_sel);
        detail::property_set_string(*_tree, path, std::to_string(device_bits));
        const active_trigger_t actual = trigger_read_trig_sel(path, request, dev);
        if (actual == trig_sel) {
            return true;
        }
        trigger_handle_coerced_value(path, std::to_string(static_cast<uint32_t>(trig_sel)),
            std::to_string(static_cast<uint32_t>(actual)), request.op);
        trig_sel = actual;
        return false;
    }

    /*!
     * The property that enables a channel: stream for RX, and for TX stream if the
     * server has it (TRIGGER_OVERHAUL_SERVER_R3_S), otherwise pwr.
     */
    fs_path trigger_enable_path(const uhd::trigger_pv::trigger_rx_t& rx, const trigger_request_t&)
    {
        return rx_root(rx.channel) / "stream";
    }
    fs_path trigger_enable_path(const uhd::trigger_pv::trigger_tx_t& tx, const trigger_request_t& request)
    {
        if (request.tx_stream_supported) {
            return tx_root(tx.channel) / "stream";
        }
        // Older servers. Reads treat pwr == 1 as enabled. A channel with pwr == 1
        // could still be in reset if rstreq was never sent after pwr was set.
        return tx_root(tx.channel) / "pwr";
    }

    fs_path trigger_channel_root(const uhd::trigger_pv::trigger_rx_t& rx)
    {
        return rx_trigger_root(rx.channel);
    }
    fs_path trigger_channel_root(const uhd::trigger_pv::trigger_tx_t& tx)
    {
        return tx_trigger_root(tx.channel);
    }

    //! Apply the per-channel settings other than trig_sel, ufl_dir and enable.
    template <typename channel_t>
    bool trigger_apply_channel_settings(channel_t& chan, const uhd::trigger_pv::trigger_op_t& op)
    {
        using namespace uhd::trigger_pv;

        const fs_path root = trigger_channel_root(chan);
        // The call goes FIRST in each `&&`: written as `exact && trigger_...()`, the
        // call would be skipped as soon as exact became false, because && short-circuits.
        bool exact = true;
        exact = trigger_apply_integer<std::string>(root / "edge_backoff", chan.edge_backoff, UINT32_MAX, op) && exact;
        exact = trigger_apply_integer<std::string>(root / "edge_sample_num", chan.edge_sample_num, INT64_MAX, op) && exact;
        exact = trigger_apply_integer<std::string>(root / "ufl_output_edge_sample_num", chan.ufl_output_pulse_length, INT64_MAX, op) && exact;
        exact = trigger_apply_enum(root / "sma_mode", chan.sma_mode, op, &detail::trigger_mode_from_device_string) && exact;
        exact = trigger_apply_enum(root / "ufl_mode", chan.ufl_mode, op, &detail::trigger_mode_from_device_string) && exact;
        exact = trigger_apply_enum(root / "ufl_pol", chan.ufl_polarity, op, &detail::polarity_from_device_string) && exact;
        return exact;
    }

    /*!
     * Stage 2: apply a validated request in the order described by
     * setup_mode_t::standard, writing the values in effect back into it.
     * \return true if every value was applied exactly as requested
     */
    bool trigger_apply(trigger_request_t& request, const uhd::pv_device& dev)
    {
        using namespace uhd::trigger_pv;

        const trigger_op_t& op = request.op;
        // The call goes FIRST in each `&&`; see trigger_apply_channel_settings().
        bool exact = true;

        // -----------------------------------------------------------------
        // 1. DISABLE unspecified channels, then channels without hot_configure.
        // trig_sel is never changed to disable a channel: with no trigger
        // selected, a channel streams on normal SOB/EOB.
        // -----------------------------------------------------------------
        // Servers without the TX stream property can only disable TX by powering
        // down the RF chain, which is not restored when the channel is re-enabled.
        // TX channels are left enabled on those servers.
        bool tx_disable_skipped = false;

        for (const size_t channel : request.unspecified_rx) {
            detail::property_set_string(*_tree, rx_root(channel) / "stream", "0");
        }
        for (const size_t channel : request.unspecified_tx) {
            if (request.tx_stream_supported) {
                detail::property_set_string(*_tree, tx_root(channel) / "stream", "0");
            } else {
                tx_disable_skipped = true;
            }
        }
        // For a disabled channel whose enable is get, the value read here is
        // both the result of the get and the value restored in step 6.
        auto disable = [&](auto& chan) {
            if (chan.hot_configure) {
                return;
            }
            const fs_path path = trigger_enable_path(chan, request);
            if (chan.enable == GET_INT) {
                chan.enable = trigger_read_integer<std::string>(path, 1);
            }
            detail::property_set_string(*_tree, path, "0");
        };
        std::for_each(request.rx.begin(), request.rx.end(), disable);
        for (trigger_tx_t& tx : request.tx) {
            if (request.tx_stream_supported) {
                disable(tx);
            } else if (!tx.hot_configure) {
                tx_disable_skipped = true;
            }
        }
        if (tx_disable_skipped) {
            trigger_warn(op, warning_t::unsupported_server,
                "Disabling tx while configuring is not supported. Spurious activations may occur.");
        }

        // -----------------------------------------------------------------
        // 2. PINS TO INPUT: every pin being set starts as an input, so nothing
        // is driven while the rest is configured. Pins whose final direction
        // is in are finished here.
        // -----------------------------------------------------------------
        auto pin_to_input = [&](const fs_path& path, const trigger_pin_plan_t& plan,
                                pin_direction_t& reported) {
            if (plan.write == pin_direction_t::in) {
                exact = trigger_apply_final_direction(path, pin_direction_t::in, reported, op) && exact;
            } else if (plan.write == pin_direction_t::out) {
                trigger_write_direction(path, pin_direction_t::in);
            }
        };
        if (request.global.has_value()) {
            pin_to_input(trigger_root() / "sma_dir", request.sma, request.global->sma_direction);
        }
        for (size_t i = 0; i < request.rx.size(); i++) {
            pin_to_input(rx_trigger_root(request.rx[i].channel) / "ufl_dir", request.rx_ufl[i], request.rx[i].ufl_dir);
        }
        for (size_t i = 0; i < request.tx.size(); i++) {
            pin_to_input(tx_trigger_root(request.tx[i].channel) / "ufl_dir", request.tx_ufl[i], request.tx[i].ufl_dir);
        }
        if (request.sma.write != pin_direction_t::get) {
            // The SMA's final direction is known from here on.
            request.sma_in_effect = request.sma.write;
        }

        // -----------------------------------------------------------------
        // 3. GLOBAL SETTINGS
        // -----------------------------------------------------------------
        if (request.global.has_value()) {
            trigger_global_t& global = *request.global;
            exact = trigger_apply_enum(trigger_root() / "sma_pol", global.sma_polarity, op, &detail::polarity_from_device_string) && exact;
            exact = trigger_apply_integer<int>(trigger_root() / "sma_override", global.sma_override, 1, op) && exact;
            exact = trigger_apply_integer<std::string>(trigger_root() / "sma_output_edge_sample_num", global.sma_output_pulse_length, INT64_MAX, op) && exact;
        }

        // -----------------------------------------------------------------
        // 4. PER-CHANNEL SETTINGS, then trig_sel. TX gating is applied last.
        // -----------------------------------------------------------------
        for (trigger_rx_t& rx : request.rx) {
            exact = trigger_apply_channel_settings(rx, op) && exact;
            exact = trigger_apply_trig_sel(rx_trigger_root(rx.channel) / "trig_sel", rx.trig_sel, request, dev) && exact;
        }
        for (trigger_tx_t& tx : request.tx) {
            exact = trigger_apply_channel_settings(tx, op) && exact;
            exact = trigger_apply_trig_sel(tx_trigger_root(tx.channel) / "trig_sel", tx.trig_sel, request, dev) && exact;
        }
        for (trigger_tx_t& tx : request.tx) {
            exact = trigger_apply_enum(tx_trigger_root(tx.channel) / "gating", tx.gating, op, &detail::gating_from_device_string) && exact;
        }

        // -----------------------------------------------------------------
        // 5. PINS TO OUTPUT, then pins whose direction is get or reported
        // without device access.
        // -----------------------------------------------------------------
        auto pin_finish = [&](const fs_path& path, const trigger_pin_plan_t& plan,
                              pin_direction_t& reported) {
            if (plan.write == pin_direction_t::out) {
                exact = trigger_apply_final_direction(path, pin_direction_t::out, reported, op) && exact;
            } else if (plan.read) {
                reported = detail::direction_from_device_string(detail::property_get_string(*_tree, path));
            } else if (plan.report.has_value()) {
                reported = *plan.report;
            }
        };
        if (request.global.has_value()) {
            pin_finish(trigger_root() / "sma_dir", request.sma, request.global->sma_direction);
        }
        for (size_t i = 0; i < request.rx.size(); i++) {
            pin_finish(rx_trigger_root(request.rx[i].channel) / "ufl_dir", request.rx_ufl[i], request.rx[i].ufl_dir);
        }
        for (size_t i = 0; i < request.tx.size(); i++) {
            pin_finish(tx_trigger_root(request.tx[i].channel) / "ufl_dir", request.tx_ufl[i], request.tx[i].ufl_dir);
        }

        // -----------------------------------------------------------------
        // 6. RE-ENABLE: set enable on every specified channel. A disabled
        // channel whose enable was get is restored to the value read in step 1.
        // -----------------------------------------------------------------
        for (trigger_rx_t& rx : request.rx) {
            exact = trigger_apply_integer<std::string>(trigger_enable_path(rx, request), rx.enable, 1, op) && exact;
        }
        for (trigger_tx_t& tx : request.tx) {
            const fs_path path = trigger_enable_path(tx, request);
            if (request.tx_stream_supported) {
                exact = trigger_apply_integer<std::string>(path, tx.enable, 1, op) && exact;
            } else if (tx.enable == 0) {
                // Older servers: disabling would power down the RF chain, so the
                // channel is left as is and its actual state is reported.
                tx.enable = trigger_read_integer<std::string>(path, 1);
                if (tx.enable != 0) {
                    trigger_warn(op, warning_t::unsupported_server,
                        std::format("TX channel {}: disabling tx is not supported by this server. "
                                    "The channel was left enabled.",
                            tx.channel));
                    exact = false;
                }
            } else {
                // Every TX channel's enable is a real value here except hot
                // configured channels left as get, which are only read.
                const bool writes_pwr = (tx.enable != GET_INT);
                exact = trigger_apply_integer<std::string>(path, tx.enable, 1, op) && exact;
                if (writes_pwr && tx.enable == 1) {
                    // Setting pwr to 1 always leaves the channel in reset;
                    // rstreq takes it out of reset. rstreq is write-only.
                    detail::property_set_double(*_tree, tx_dsp_root(tx.channel) / "rstreq", 1.0);
                }
            }
        }

        return exact;
    }

    /*!
     * failure_handling_t::disable_all_after_validation: attempt to disable every
     * channel and set every toggleable pin to input. Each step is attempted even
     * if earlier ones fail; failures are logged, not thrown, so the caller can
     * rethrow the original error.
     */
    void trigger_disable_all_after_failure(const uhd::pv_device& dev) noexcept
    {
        using namespace uhd::trigger_pv;

        auto attempt = [this](const auto& make_path, const std::string& value) {
            std::string path_name = "(unknown path)";
            try {
                const fs_path path = make_path();
                path_name          = path;
                detail::property_set_string(*_tree, path, value);
            } catch (const std::exception& e) {
                UHD_LOG_ERROR("MULTI_USRP",
                    std::format("trigger_configure: failed to set {} after an error: {}",
                        path_name, e.what()));
            } catch (...) {
                UHD_LOG_ERROR("MULTI_USRP",
                    std::format("trigger_configure: failed to set {} after an error", path_name));
            }
        };

        size_t num_rx = 0;
        size_t num_tx = 0;
        try {
            num_rx = get_rx_num_channels();
            num_tx = get_tx_num_channels();
        } catch (const std::exception& e) {
            UHD_LOG_ERROR("MULTI_USRP",
                std::format("trigger_configure: failed to count channels after an error: {}",
                    e.what()));
        } catch (...) {
            UHD_LOG_ERROR("MULTI_USRP", "trigger_configure: failed to count channels after an error");
        }

        for (size_t channel = 0; channel < num_rx; channel++) {
            attempt([&]() { return rx_root(channel) / "stream"; }, "0");
        }
        // Servers without the TX stream property can only disable TX by powering
        // down the RF chain, so TX channels are left enabled on them.
        if (dev.server_api_ver >= TRIGGER_OVERHAUL_SERVER_R3_S) {
            for (size_t channel = 0; channel < num_tx; channel++) {
                attempt([&]() { return tx_root(channel) / "stream"; }, "0");
            }
        }

        bool sma_toggleable = false;
        try {
            sma_toggleable = (dev.trigger_sma_pin_type() == trigger_pin_type_t::toggleable);
        } catch (...) {
            // Not expected; leave the SMA alone.
        }
        if (sma_toggleable) {
            attempt([&]() { return trigger_root() / "sma_dir"; }, "in");
        }
        static_assert(trigger_ufl_pin_type == trigger_pin_type_t::toggleable);
        for (size_t channel = 0; channel < num_rx; channel++) {
            attempt([&]() { return rx_trigger_root(channel) / "ufl_dir"; }, "in");
        }
        for (size_t channel = 0; channel < num_tx; channel++) {
            attempt([&]() { return tx_trigger_root(channel) / "ufl_dir"; }, "in");
        }
    }

    /*!
     * Run failure handling for a uhd::value_error or uhd::index_error only if it
     * was thrown after validation (which should not happen). During validation
     * nothing on the device has been changed.
     */
    void handle_trigger_failure_if(
        const bool validated, const trigger_request_t& request, const uhd::pv_device& dev) noexcept
    {
        if (validated
            && request.op.failure_handling
                   == uhd::trigger_pv::failure_handling_t::disable_all_after_validation) {
            trigger_disable_all_after_failure(dev);
        }
    }

public:
    bool trigger_configure(const uhd::trigger_pv::trigger_op_t* op,
        uhd::trigger_pv::trigger_global_t* global,
        uhd::trigger_pv::trigger_rx_t* rx,
        size_t rx_len,
        uhd::trigger_pv::trigger_tx_t* tx,
        size_t tx_len) override
    {
        using namespace uhd::trigger_pv;

        // Throws uhd::not_implemented_error before anything else for non Per Vices devices.
        uhd::pv_device& dev = trigger_pv_device();

        // Deep copies of the caller's structs, in the library's layout. All work
        // below is done on these; the caller's structs are only written back on
        // success, so after an exception they still hold exactly what was requested.
        trigger_request_t request;
        // A null op means "use the defaults".
        request.op = (op != nullptr) ? detail::deep_copy_from_caller(*op) : trigger_op_t{};
        // A null global means "leave the global settings unchanged".
        if (global != nullptr) {
            request.global = detail::deep_copy_from_caller(*global);
        }
        // Throw if an array is null with a non-zero length, or holds malformed or
        // mixed-version elements.
        request.rx = detail::deep_copy_array_from_caller(rx, rx_len);
        request.tx = detail::deep_copy_array_from_caller(tx, tx_len);

        bool exact = false;
        // value_error and index_error skip failure handling only while nothing
        // on the device can have changed.
        bool validated = false;
        try {
            // Throw if a channel is unset, does not exist, or is specified twice.
            request.unspecified_rx = detail::find_unspecified_channels(request.rx, get_rx_num_channels());
            request.unspecified_tx = detail::find_unspecified_channels(request.tx, get_tx_num_channels());

            trigger_validate(request, dev);
            validated = true;
            exact     = trigger_apply(request, dev);
        } catch (const uhd::value_error&) {
            handle_trigger_failure_if(validated, request, dev);
            throw;
        } catch (const uhd::index_error&) {
            handle_trigger_failure_if(validated, request, dev);
            throw;
        } catch (...) {
            if (request.op.failure_handling == failure_handling_t::disable_all_after_validation) {
                trigger_disable_all_after_failure(dev);
            }
            throw; // Rethrow the original exception.
        }

        // Success. Validate every caller element before writing any of them, so
        // the caller's structs are either all updated or all left as requested.
        for (size_t channel_index = 0; channel_index < rx_len; channel_index++) {
            detail::caller_element(rx, rx_len, channel_index);
        }
        for (size_t channel_index = 0; channel_index < tx_len; channel_index++) {
            detail::caller_element(tx, tx_len, channel_index);
        }
        // Deep copy the actual values back to the caller.
        if (global != nullptr) {
            detail::deep_copy_to_caller(*request.global, *global);
        }
        detail::deep_copy_array_to_caller(request.rx, rx, rx_len);
        detail::deep_copy_array_to_caller(request.tx, tx, tx_len);

        return exact;
    }

};

multi_usrp::~multi_usrp(void)
{
    /* NOP */
}


/***********************************************************************
 * The Make Function
 **********************************************************************/
namespace uhd { namespace rfnoc { namespace detail {
// Forward declare
multi_usrp::sptr make_rfnoc_device(
    detail::rfnoc_device::sptr rfnoc_device, const uhd::device_addr_t& dev_addr);
}}} // namespace uhd::rfnoc::detail


multi_usrp::sptr multi_usrp::make(const device_addr_t& dev_addr)
{
    UHD_LOGGER_TRACE("MULTI_USRP")
        << "multi_usrp::make with args " << dev_addr.to_pp_string();

    device::sptr dev = device::make(dev_addr, device::USRP);

    auto rfnoc_dev = std::dynamic_pointer_cast<rfnoc::detail::rfnoc_device>(dev);
    if (rfnoc_dev) {
        return rfnoc::detail::make_rfnoc_device(rfnoc_dev, dev_addr);
    }
    return std::make_shared<multi_usrp_impl>(dev);
}
