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
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
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
        (boost::format(                                                          \
             "%s: gain \"%s\" not found for channel %d.\nAvailable gains: %s\n") \
            % UHD_FUNCTION % name % chan                                         \
            % string_vector_to_string(get_##dir##_gain_names(chan)))             \
            .str());

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
        std::string message = str(boost::format(
                   "Extreme difference between target and actual when setting %s center frequency:\n"
                   "Target center frequency: %f MHz\n"
                   "Actual center frequency: %f MHz\n")
                   % xx % (target_freq / 1e6) % (actual_freq / 1e6));
        UHD_LOG_ERROR("MULTI_USRP", message)
        throw uhd::runtime_error(message);

    } else if (diff > warn_threshold) {
        UHD_LOGGER_WARNING("MULTI_USRP")
            << boost::format(
                   "Error while attempting to set the requested %s center frequency:\n"
                   "Target center frequency: %f MHz\n"
                   "Actual center frequency: %f MHz\n")
                   % xx % (target_freq / 1e6) % (actual_freq / 1e6);
    }
}

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
        std::string buff = str(boost::format("%s USRP:\n"
                                             "  Device: %s\n")
                               % ((get_num_mboards() > 1) ? "Multi" : "Single")
                               % (_tree->access<std::string>("/name").get()));
        for (size_t m = 0; m < get_num_mboards(); m++) {
            buff += str(boost::format("  Mboard %d: %s\n") % m
                        % (_tree->access<std::string>(mb_root(m) / "name").get()));
        }

        //----------- rx side of life ----------------------------------
        for (size_t m = 0, chan = 0; m < get_num_mboards(); m++) {
            for (; chan < (m + 1) * get_rx_subdev_spec(m).size(); chan++) {
                buff += str(
                    boost::format("  RX Channel: %u\n"
                                  "    RX DSP: %s\n"
                                  "    RX Dboard: %s\n"
                                  "    RX Subdev: %s\n")
                    % chan % rx_dsp_root(chan).leaf()
                    % rx_rf_fe_root(chan).branch_path().branch_path().leaf()
                    % (_tree->access<std::string>(rx_rf_fe_root(chan) / "name").get()));
            }
        }

        //----------- tx side of life ----------------------------------
        for (size_t m = 0, chan = 0; m < get_num_mboards(); m++) {
            for (; chan < (m + 1) * get_tx_subdev_spec(m).size(); chan++) {
                buff += str(
                    boost::format("  TX Channel: %u\n"
                                  "    TX DSP: %s\n"
                                  "    TX Dboard: %s\n"
                                  "    TX Subdev: %s\n")
                    % chan % tx_dsp_root(chan).leaf()
                    % tx_rf_fe_root(chan).branch_path().branch_path().leaf()
                    % (_tree->access<std::string>(tx_rf_fe_root(chan) / "name").get()));
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
            this->get_device()->set_time_now(time_spec, mboard);
            this->get_device()->request_resync_time_diff();
            return;
        }
        for (size_t m = 0; m < get_num_mboards(); m++) {
            set_time_now(time_spec, m);
        }
    }

    void set_time_next_pps(const time_spec_t& time_spec, size_t mboard) override
    {
        if (mboard != ALL_MBOARDS) {
            _tree->access<time_spec_t>(mb_root(mboard) / "time/pps").set(time_spec);
            this->get_device()->request_resync_time_diff();
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
                    << boost::format(
                           "Detected time deviation between board %d and board 0.\n"
                           "Board 0 time is %f seconds.\n"
                           "Board %d time is %f seconds.\n")
                           % m % time_0.get_real_secs() % m % time_i.get_real_secs();
            }
        }
        this->get_device()->request_resync_time_diff();
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
                UHD_LOGGER_ERROR("MULTI_USRP")  <<boost::format( "Unable to set clock source. The program attempted to set it to %s but it returned: %s") % source % result ;
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
                    str(boost::format("multi_usrp::get_rx_subdev_spec(%u) failed to make "
                                      "default spec - %s")
                        % mboard % e.what()));
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
        std::vector<std::string> lo_names;
        if (_tree->exists(rx_rf_fe_root(chan) / "los")) {
            for (const std::string& name : _tree->list(rx_rf_fe_root(chan) / "los")) {
                lo_names.push_back(name);
            }
        }
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
        std::vector<std::string> lo_names;
        if (_tree->exists(tx_rf_fe_root(chan) / "los")) {
            for (const std::string& name : _tree->list(tx_rf_fe_root(chan) / "los")) {
                lo_names.push_back(name);
            }
        }
        return lo_names;
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

    // Both tx and rx trigger configurations set the trigger direction. These are used for a warning
    enum which_rxtx { none, rx, tx };
    which_rxtx last_trigger_dir_set_by = none;
    std::string previous_trigger_dir;

    uint64_t tx_trigger_setup(
        std::vector<size_t> channels,
        uint64_t num_samples_per_trigger,
        std::string trigger_dir
    ) {
        uint64_t actual_num_samples_per_trigger = 0;
        for(size_t n = 0; n < channels.size(); n++) {
            const std::string root { "/mboards/0/tx/" + std::to_string(channels[n]) + "/" };
            _tree->access<std::string>(root + "trigger/sma_mode").set("edge");
            _tree->access<std::string>(root + "trigger/trig_sel").set("1");
            _tree->access<std::string>(root + "trigger/edge_backoff").set("0");
            _tree->access<std::string>(root + "trigger/edge_sample_num").set(std::to_string(num_samples_per_trigger));
            actual_num_samples_per_trigger = std::stoul(_tree->access<std::string>(root + "trigger/edge_sample_num").get(),nullptr,10);
            _tree->access<std::string>(root + "trigger/gating").set("dsp");
        }
        // Prints warning message in case the user set the direction to something different when using rx
        if(last_trigger_dir_set_by == rx) {
            if(previous_trigger_dir != trigger_dir) {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << "Warning trigger direction was set to a a different value when configuring rx. RX and TX use the same trigger port. The value from rx is being overwritten Previously set to " + previous_trigger_dir + ", changing it to " + trigger_dir + " while setting up tx trigger\n";
            }
        } else {
            last_trigger_dir_set_by = tx;
            previous_trigger_dir = trigger_dir;
        }
        _tree->access<std::string>("/mboards/0/trigger/sma_dir").set(trigger_dir);

        _tree->access<std::string>("/mboards/0/trigger/sma_pol").set("positive");
        for(size_t n = 0; n < channels.size(); n++) {
            const std::string dsp_root { "/mboards/0/tx_dsps/" + std::to_string(channels[n]) + "/" };
            _tree->access<double>(dsp_root + "rstreq").set(1.0);
        }
        return actual_num_samples_per_trigger;
    }

    void tx_trigger_cleanup(
        std::vector<size_t> channels
    ) {
        for(size_t n = 0; n < channels.size(); n++) {
            const std::string root { "/mboards/0/tx/" + std::to_string(channels[n]) + "/" };
            _tree->access<std::string>(root + "trigger/edge_sample_num").set("0");
            _tree->access<std::string>(root + "trigger/trig_sel").set("1");
        }
        last_trigger_dir_set_by = none;
    }

    uint64_t rx_trigger_setup(
        std::vector<size_t> channels,
        uint64_t num_samples_per_trigger,
        std::string trigger_dir
    ) {
        uint64_t actual_num_samples_per_trigger = 0;
        for(size_t n = 0; n < channels.size(); n++) {
            const std::string root { "/mboards/0/rx/" + std::to_string(channels[n]) + "/" };
            _tree->access<std::string>(root + "stream").set("0");
            _tree->access<std::string>(root + "trigger/edge_sample_num").set(std::to_string(num_samples_per_trigger));
            actual_num_samples_per_trigger = std::stoul(_tree->access<std::string>(root + "trigger/edge_sample_num").get(),nullptr,10);
            _tree->access<std::string>(root + "trigger/sma_mode").set("edge");
            _tree->access<std::string>(root + "trigger/edge_backoff").set("0");
            _tree->access<std::string>(root + "trigger/trig_sel").set("1");
        }
        // Prints warning message in case the user set the direction to something different when using rx
        if(last_trigger_dir_set_by == tx) {
            if(previous_trigger_dir != trigger_dir) {
                UHD_LOGGER_WARNING("MULTI_USRP")
                    << boost::format(
                        "Warning trigger direction was set to a a different value when configuring tx. RX and TX use the same trigger port. The value from tx is being overwritten Previously set to %s, changing it to % while setting up tx trigger\n")
                        % previous_trigger_dir % trigger_dir;
            }
        } else {
            last_trigger_dir_set_by = tx;
            previous_trigger_dir = trigger_dir;
        }
        _tree->access<std::string>("/mboards/0/trigger/sma_dir").set(trigger_dir);

        _tree->access<std::string>("/mboards/0/trigger/sma_pol").set("positive");

        for(size_t n = 0; n < channels.size(); n++) {
            const std::string root { "/mboards/0/rx/" + std::to_string(channels[n]) + "/" };
            _tree->access<std::string>(root + "stream").set("1");
        }
        return actual_num_samples_per_trigger;
    }

    void rx_trigger_cleanup(
        std::vector<size_t> channels
    ) {
        //Stream needs to be set to 0 first, since certain channels share certain properties
        for(size_t n = 0; n < channels.size(); n++) {
            const std::string root { "/mboards/0/rx/" + std::to_string(channels[n]) + "/" };
            _tree->access<std::string>(root + "stream").set("0");
        }
        for(size_t n = 0; n < channels.size(); n++) {
            const std::string root { "/mboards/0/rx/" + std::to_string(channels[n]) + "/" };
            _tree->access<std::string>(root + "trigger/edge_sample_num").set("0");
            _tree->access<std::string>(root + "trigger/trig_sel").set("0");
        }
        last_trigger_dir_set_by = none;
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
                throw uhd::runtime_error(str(
                    boost::format("The hardware has no gpio attribute: `%s':\n") % attr));
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
            str(boost::format("The hardware has no GPIO bank `%s'") % bank));
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
                throw uhd::runtime_error(str(
                    boost::format("The hardware has no gpio attribute: `%s'") % attr));
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
            str(boost::format("The hardware has no gpio bank `%s'") % bank));
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
            throw uhd::index_error(str(
                boost::format(
                    "multi_usrp: RX channel %u out of range for configured RX frontends")
                % chan));
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
            throw uhd::index_error(str(
                boost::format(
                    "multi_usrp: TX channel %u out of range for configured TX frontends")
                % chan));
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
                throw uhd::index_error(str(
                    boost::format("multi_usrp::mb_root(%u) - path not found") % mboard));
            }
        } catch (const std::exception& e) {
            throw uhd::index_error(
                str(boost::format("multi_usrp::mb_root(%u) - %s") % mboard % e.what()));
        }
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
                    str(boost::format(
                            "multi_usrp::rx_dsp_root(%u) - mcp(%u) - path not found")
                        % chan % mcp.chan));
            }
        } catch (const std::exception& e) {
            throw uhd::index_error(
                str(boost::format("multi_usrp::rx_dsp_root(%u) - mcp(%u) - %s") % chan
                    % mcp.chan % e.what()));
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
                    str(boost::format(
                            "multi_usrp::tx_dsp_root(%u) - mcp(%u) - path not found")
                        % chan % mcp.chan));
            }
        } catch (const std::exception& e) {
            throw uhd::index_error(
                str(boost::format("multi_usrp::tx_dsp_root(%u) - mcp(%u) - %s") % chan
                    % mcp.chan % e.what()));
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
                str(boost::format("multi_usrp::rx_fe_root(%u) - mcp(%u) - %s") % chan
                    % mcp.chan % e.what()));
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
                str(boost::format("multi_usrp::tx_fe_root(%u) - mcp(%u) - %s") % chan
                    % mcp.chan % e.what()));
        }
    }

    fs_path tx_link_root(const size_t chan)
    {
        return "/mboards/0/tx_link/" + std::to_string(chan);
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
            throw uhd::key_error(str(
                boost::format("[multi_usrp]: radio slot name %s out of supported range.")
                % slot_name));
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
                str(boost::format("multi_usrp::rx_rf_fe_root(%u) - mcp(%u) - %s") % chan
                    % mcp.chan % e.what()));
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
                str(boost::format("multi_usrp::tx_rf_fe_root(%u) - mcp(%u) - %s") % chan
                    % mcp.chan % e.what()));
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
