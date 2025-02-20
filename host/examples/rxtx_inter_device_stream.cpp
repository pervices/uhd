//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2022-2023 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <iostream>
#include <csignal>
#include <string>
#include <chrono>
#include <thread>
#include <cassert>

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){
    stop_signal_called = true;
}

// device_num_channels: number of channels on the device, used for error checking
// channel_arg: String provided by the user containing the list of channels
std::vector<size_t> parse_channels(size_t device_num_channels, std::string channel_arg) {
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_arg, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        if(channel_strings[ch] == "") {
            continue;
        }
        size_t chan = std::stoi(channel_strings[ch]);
        if(chan > device_num_channels) {
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else {
            channel_nums.push_back(std::stoi(channel_strings[ch]));
        }
    }
    return channel_nums;
}

// Parses the string providing arguments for rf settings
// num_channels: Number of channels expected to be used
// rf_arg: String provided by the user containing the list of said rf setting
// error_msg: Error message to print if this is a mistmatch between the number of channels specified for the rf setting and the length of the list requested by the user
std::vector<double> parse_rf_settings(size_t num_channels, std::string rf_arg, std::string error_msg) {
    // Return empty vector if no argument provided
    if(rf_arg == "") {
        return std::vector<double>(0);
    }

    std::vector<std::string> rf_strings;
    boost::split(rf_strings, rf_arg, boost::is_any_of("\"',"));

    // Only 1 value specified, apply it to all
    if(rf_strings.size() == 1) {
        return std::vector(num_channels, std::stod(rf_strings[0]));
    }
    else if(rf_strings.size() != num_channels) {
        throw std::runtime_error(error_msg);
    }

    std::vector<double> rf_settings(num_channels);
    for(size_t n = 0; n < num_channels; n++){
        rf_settings[n] =  std::stod(rf_strings[n]);
    }
    return rf_settings;
}

// Verfies that there isn't a mismatch in rates between channels
inline void validate_rates(double desired, double actual) {
    if(desired != actual && desired != 0) {
        throw uhd::value_error("Mismatch in rate between channels");
    }
}

void set_rx_gain(double desired, size_t channel, std::string name, uhd::usrp::multi_usrp::sptr usrp) {
    usrp->set_rx_gain(desired, channel);
    double actual = usrp->get_rx_gain(channel);
    if(std::abs(actual - desired) >= 1) {
        UHD_LOGGER_WARNING("RXTX_INTER_DEVICE_STREAM") << "Unable to set " << name << " on ch " << channel<< ". Desired: " << desired << ", actual: " << actual;
    }
}

void set_rx_gain(std::vector<double> desired, std::vector<size_t> channel, std::string name, uhd::usrp::multi_usrp::sptr usrp) {
    assert(desired.size() == channel.size());

    for(size_t n = 0; n < desired.size(); n++) {
        set_rx_gain(desired[n], channel[n], name, usrp);
    }
}

void set_tx_gain(double desired, size_t channel, std::string name, uhd::usrp::multi_usrp::sptr usrp) {
    usrp->set_tx_gain(desired, channel);
    double actual = usrp->get_tx_gain(channel);
    if(std::abs(actual - desired) >= 1) {
        UHD_LOGGER_WARNING("RXTX_INTER_DEVICE_STREAM") << "Unable to set " << name << " on ch " << channel<< ". Desired: " << desired << ", actual: " << actual;
    }
}

void set_tx_gain(std::vector<double> desired, std::vector<size_t> channel, std::string name, uhd::usrp::multi_usrp::sptr usrp) {
    assert(desired.size() == channel.size());

    for(size_t n = 0; n < desired.size(); n++) {
        set_tx_gain(desired[n], channel[n], name, usrp);
    }
}

void set_rx_freq(double desired, size_t channel, uhd::usrp::multi_usrp::sptr usrp) {
    uhd::tune_request_t tune_request(desired);
    usrp->set_rx_freq(desired, channel);
}

void set_rx_freq(std::vector<double> desired, std::vector<size_t> channel, uhd::usrp::multi_usrp::sptr usrp) {
    assert(desired.size() == channel.size());

    for(size_t n = 0; n < desired.size(); n++) {
        set_rx_freq(desired[n], channel[n], usrp);
    }
}

void set_tx_freq(double desired, size_t channel, uhd::usrp::multi_usrp::sptr usrp) {
    uhd::tune_request_t tune_request(desired);
    usrp->set_tx_freq(desired, channel);
}

void set_tx_freq(std::vector<double> desired, std::vector<size_t> channel, uhd::usrp::multi_usrp::sptr usrp) {
    assert(desired.size() == channel.size());

    for(size_t n = 0; n < desired.size(); n++) {
        set_tx_freq(desired[n], channel[n], usrp);
    }
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){

    //variables to be set by po
    std::string rx_args, tx_args, ref, pps, rx_channel_arg, tx_channel_arg, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg, ab_rate_arg, ba_rate_arg, rate_arg;
    double duration;
    int ref_clock_freq;

    std::string a_args, a_rx_channel_arg, a_tx_channel_arg, a_rx_gain_arg, a_tx_gain_arg, a_tx_freq_arg, a_rx_freq_arg;

    std::string b_args, b_rx_channel_arg, b_tx_channel_arg, b_rx_gain_arg, b_tx_gain_arg, b_tx_freq_arg, b_rx_freq_arg;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        // Universal options
        ("help", "help message")
        ("ref_clock_freq", po::value<int>(&ref_clock_freq), "Frequency of external reference clock. Program will use an internal 10MHz clock if not specified")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external)")
        ("duration", po::value<double>(&duration), "How long to stream for, will stream until the program is exited if unspecified")

        ("rate", po::value<std::string>(&rate_arg), "Alias of ab_rate and ba_rate for backwards compatibility. Using this will override both ab_rate and ba_rate with the value provided here")

        ("ab_rate", po::value<std::string>(&ab_rate_arg)->default_value("40e6"), "Rate for each channel going from device A to device B in Hz. Enter one number to set all rx channels on A and all tx channels on B to said rate i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("ba_rate", po::value<std::string>(&ba_rate_arg)->default_value("40e6"), "Rate for each channel going from device B to device A in Hz. Enter one number to set all rx channels on B and all tx channels on A to said rate i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")


        // Paremters for device A
        ("a_args", po::value<std::string>(&a_args), "Identifying info device A. Example: rx_args=\"addr=192.168.10.2\"")
        ("a_rx_freq", po::value<std::string>(&a_rx_freq_arg)->default_value("0"), "RF center frequency in Hz for rx channels on device A. Enter one number to set all the rx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("a_tx_freq", po::value<std::string>(&a_tx_freq_arg)->default_value("0"), "RF center frequency in Hz for tx channels on device A. Enter one number to set all the tx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("a_rx_gain", po::value<std::string>(&a_rx_gain_arg)->default_value("0"), "gain for the Rx RF chain on device A. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("a_tx_gain", po::value<std::string>(&a_tx_gain_arg)->default_value("0"), "gain for the Tx RF chain on device A. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("a_rx_channels", po::value<std::string>(&a_rx_channel_arg)->default_value("0"), "which rx channels to use on device A (specify \"0\", \"1\", \"0,1\", etc)")
        ("a_tx_channels", po::value<std::string>(&a_tx_channel_arg)->default_value(""), "which tx channels to use on device A (specify \"0\", \"1\", \"0,1\", etc)")

        // Paremters for device B
        ("b_args", po::value<std::string>(&b_args), "Identifying info device B. Example: rx_args=\"addr=192.168.10.2\"")
        ("b_rx_freq", po::value<std::string>(&b_rx_freq_arg)->default_value("0"), "RF center frequency in Hz for rx channels on device B. Enter one number to set all the rx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("b_tx_freq", po::value<std::string>(&b_tx_freq_arg)->default_value("0"), "RF center frequency in Hz for tx channels on device B. Enter one number to set all the tx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("b_rx_gain", po::value<std::string>(&b_rx_gain_arg)->default_value("0"), "gain for the Rx RF chain on device B. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("b_tx_gain", po::value<std::string>(&b_tx_gain_arg)->default_value("0"), "gain for the Tx RF chain on device B. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("b_rx_channels", po::value<std::string>(&b_rx_channel_arg)->default_value(""), "which rx channels to use on device B (specify \"0\", \"1\", \"0,1\", etc)")
        ("b_tx_channels", po::value<std::string>(&b_tx_channel_arg)->default_value("0"), "which tx channels to use on device B (specify \"0\", \"1\", \"0,1\", etc)")

        // Alias options for backward compatibility
        // Makes treats a as rx and b as tx
        ("rx_args", po::value<std::string>(&rx_args), "Alias of a_args for backwards compatibility. This will override it")
        ("tx_args", po::value<std::string>(&tx_args), "Alias of b_args. This will override it")
        ("rx_freq", po::value<std::string>(&rx_freq_arg), "Alias of a_rx_freq. This will override it")
        ("tx_freq", po::value<std::string>(&tx_freq_arg), "Alias of b_tx_freq. This will override it")
        ("rx_gain", po::value<std::string>(&rx_gain_arg), "Alias of a_rx_gain. This will override it")
        ("tx_gain", po::value<std::string>(&tx_gain_arg), "Alias of b_tx_gain. This will override it")
        ("rx_channels", po::value<std::string>(&rx_channel_arg), "Alias of a_rx_channels. This will override it")
        ("tx_channels", po::value<std::string>(&tx_channel_arg), "Alias of b_tx_channels. This will override it")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
    }

    printf("T0\n");
    std::cout << "A0" << std::endl;

    // Applies overrides for backwards compatibility
    if(vm.count("rx_args")) {
        a_args = rx_args;
    }
    if(vm.count("tx_args")) {
        b_args = tx_args;
    }
    if(vm.count("rx_freq")) {
        a_rx_freq_arg = rx_freq_arg;
    }
    if(vm.count("tx_freq")) {
        b_tx_freq_arg = tx_freq_arg;
    }
    if(vm.count("rx_gain")) {
        a_rx_gain_arg = rx_gain_arg;
    }
    if(vm.count("tx_gain")) {
        b_tx_gain_arg = tx_gain_arg;
    }
    if(vm.count("rx_channels")) {
        a_rx_channel_arg = rx_channel_arg;
    }
    if(vm.count("tx_channels")) {
        b_tx_channel_arg = tx_channel_arg;
    }
    if(vm.count("rate")) {
        ab_rate_arg = rate_arg;
        ba_rate_arg = rate_arg;
    }

    // Stores whether or not to use rx or tx, useful for debugging
    //Must be set before adjusting args to bypass clock sync
    const bool use_a = a_args!="";
    const bool use_b = b_args!="";

    // flag to indicate that the unit is being run in loopback
    bool loopback_mode = a_args == b_args;
    if(loopback_mode) {
        std::cout << "Running loopback mode. Device A's rx settings and device B's tx settings will be used" << std::endl;
    }

    printf("T1\n");

    //create a usrp device
    std::cout << std::endl;
    uhd::usrp::multi_usrp::sptr a_usrp;
    uhd::usrp::multi_usrp::sptr b_usrp;
    if(!use_a && !use_b) {
        std::cerr << "ERROR: No device adress specified. Since this program is intended to run with multiple devices connected to the same host, you must specify which devices are used for each role" << std::endl;
        return ~0;
    }
    else {
        if(use_a) {
            std::cout << boost::format("Creating the usrp device with: %s...") % a_args << std::endl;
            a_usrp = uhd::usrp::multi_usrp::make(a_args);
        }
        else {
            UHD_LOGGER_ERROR("RXTX_INTER_DEVICE_STREAM") << "No device adress specified for A. A will not be used.";
        }

        if(loopback_mode) {
            b_usrp = a_usrp;
        } else if(use_b) {
            std::cout << boost::format("Creating the usrp device with: %s...") % b_args << std::endl;
            b_usrp = uhd::usrp::multi_usrp::make(b_args);
        }
        else {
            UHD_LOGGER_ERROR("RXTX_INTER_DEVICE_STREAM") << "No device adress specified for B. B will not be used.";
        }
    }

    std::vector<size_t> a_rx_channel_nums;
    std::vector<size_t> a_tx_channel_nums;
    if(use_a) {
        a_rx_channel_nums = parse_channels(a_usrp->get_rx_num_channels(), a_rx_channel_arg);
        a_tx_channel_nums = parse_channels(a_usrp->get_tx_num_channels(), a_tx_channel_arg);
    }

    std::vector<size_t> b_rx_channel_nums;
    std::vector<size_t> b_tx_channel_nums;
    if(use_b) {
        b_rx_channel_nums = parse_channels(b_usrp->get_rx_num_channels(), b_rx_channel_arg);
        b_tx_channel_nums = parse_channels(b_usrp->get_tx_num_channels(), b_tx_channel_arg);
    }

    if(loopback_mode && a_tx_channel_nums.size() != 0) {
        throw std::runtime_error("TX channels specified for A while address args indicate loopback mode");
    } else if(loopback_mode && b_rx_channel_nums.size() != 0) {
        throw std::runtime_error("RX channels specified for A while address args indicate loopback mode");
    }

    size_t ab_num_channels = a_rx_channel_nums.size();
    if(ab_num_channels != b_tx_channel_nums.size()) {
        throw std::runtime_error("Mistmatch between number of rx channels in A and tx channels in B requested");
    }

    size_t ba_num_channels = a_tx_channel_nums.size();
    if(ba_num_channels != b_rx_channel_nums.size()) {
        throw std::runtime_error("Mistmatch between number of rx channels in B and tx channels in A requested");
    }

    printf("T10\n");

    std::vector<double> ab_rates = parse_rf_settings(ab_num_channels, ab_rate_arg, "Mistmatch between the number of rx channels on A, the number of tx channels on B, and the number of rates specified");

    std::vector<double> ba_rates = parse_rf_settings(ba_num_channels, ba_rate_arg, "Mistmatch between the number of rx channels on B, the number of tx channels on A, and the number of rates specified");

    printf("T20\n");

    std::vector<double> actual_ab_rates(ab_rates.size(), 0);
    std::vector<double> actual_ba_rates(ba_rates.size(), 0);

    if(use_a) {
        if(a_rx_channel_nums.size() != 0) {
            for(size_t n = 0; n < a_rx_channel_nums.size(); n++) {
                std::cout << boost::format("Setting ch %lu A RX Rate: %f Msps...") % n % (ab_rates[n]/1e6) << std::endl;
                a_usrp->set_rx_rate(ab_rates[n], a_rx_channel_nums[n]);
                actual_ab_rates[n] = a_usrp->get_rx_rate(a_rx_channel_nums[n]);
                // Record actual rate so that the tx side of B is set to match
                ab_rates[n] = actual_ab_rates[n];
                std::cout << boost::format("Actual ch %lu A RX Rate: %f Msps...") % n % (actual_ab_rates[n]/1e6) << std::endl << std::endl;
            }
        }
        if(a_tx_channel_nums.size() != 0 && !loopback_mode) {
            for(size_t n = 0; n < a_tx_channel_nums.size(); n++) {
                std::cout << boost::format("Setting ch %lu A TX Rate: %f Msps...") % n % (ba_rates[n]/1e6) << std::endl;
                a_usrp->set_tx_rate(ba_rates[n], a_tx_channel_nums[n]);
                actual_ba_rates[n] = a_usrp->get_tx_rate(a_tx_channel_nums[n]);
                // Record actual rate so that the tx side of B is set to match
                ba_rates[n] = actual_ba_rates[n];
                std::cout << boost::format("Actual ch %lu A TX Rate: %f Msps...") % n % (actual_ba_rates[n]/1e6) << std::endl << std::endl;
            }
        }
    }

    if(use_b) {
        if(b_rx_channel_nums.size() != 0 && !loopback_mode) {
            for(size_t n = 0; n < b_rx_channel_nums.size(); n++) {
                std::cout << boost::format("Setting ch %lu B RX Rate: %f Msps...") % n % (ba_rates[n]/1e6) << std::endl;
                b_usrp->set_rx_rate(ba_rates[n], b_rx_channel_nums[n]);
                double actual_rate = b_usrp->get_rx_rate(b_rx_channel_nums[n]);
                // Verify B rx matches A tx
                validate_rates(actual_ba_rates[n], actual_rate);
                std::cout << boost::format("Actual ch %lu B RX Rate: %f Msps...") % n % (actual_rate/1e6) << std::endl << std::endl;
            }
        }
        if(b_tx_channel_nums.size() != 0) {
            for(size_t n = 0; n < b_tx_channel_nums.size(); n++) {
                std::cout << boost::format("Setting ch %lu B TX Rate: %f Msps...") % n % (ab_rates[n]/1e6) << std::endl;
                b_usrp->set_tx_rate(ab_rates[n], b_tx_channel_nums[n]);
                double actual_rate = b_usrp->get_tx_rate(b_tx_channel_nums[n]);
                // Verify B rx matches A tx
                validate_rates(actual_ab_rates[n], actual_rate);
                std::cout << boost::format("Actual ch %lu B TX Rate: %f Msps...") % n % (actual_rate/1e6) << std::endl << std::endl;
            }
        }
    }

    std::vector<double> a_rx_freqs = parse_rf_settings(ab_num_channels, a_rx_freq_arg, "Mistmatch between number of rx channels on A and the number of frequencies specified");
    std::vector<double> a_tx_freqs = parse_rf_settings(ba_num_channels, a_tx_freq_arg, "Mistmatch between number of tx channels on A and the number of frequencies specified");
    std::vector<double> a_rx_gains = parse_rf_settings(ab_num_channels, a_rx_gain_arg, "Mistmatch between number of rx channels on A and the number of gains specified");
    std::vector<double> a_tx_gains = parse_rf_settings(ba_num_channels, a_tx_gain_arg, "Mistmatch between number of tx channels on A and the number of gains specified");

    std::vector<double> b_rx_freqs = parse_rf_settings(ba_num_channels, b_rx_freq_arg, "Mistmatch between number of rx channels on B and the number of frequencies specified");
    std::vector<double> b_tx_freqs = parse_rf_settings(ab_num_channels, b_tx_freq_arg, "Mistmatch between number of tx channels on B and the number of frequencies specified");
    std::vector<double> b_rx_gains = parse_rf_settings(ba_num_channels, b_rx_gain_arg, "Mistmatch between number of rx channels on B and the number of gains specified");
    std::vector<double> b_tx_gains = parse_rf_settings(ab_num_channels, b_tx_gain_arg, "Mistmatch between number of tx channels on B and the number of gains specified");
    
    // Sets rf frequencies, must be done before setting gain
    set_rx_freq(a_rx_freqs, a_rx_channel_nums, a_usrp);
    set_tx_freq(a_tx_freqs, a_tx_channel_nums, a_usrp);
    set_rx_freq(b_rx_freqs, b_rx_channel_nums, b_usrp);
    set_tx_freq(b_tx_freqs, b_tx_channel_nums, b_usrp);

    // Sets gains
    set_rx_gain(a_rx_gains, a_rx_channel_nums, "A RX GAIN", a_usrp);
    set_tx_gain(a_tx_gains, a_tx_channel_nums, "A TX GAIN", a_usrp);
    set_rx_gain(b_rx_gains, b_rx_channel_nums, "B RX GAIN", b_usrp);
    set_tx_gain(b_tx_gains, b_tx_channel_nums, "B TX GAIN", b_usrp);


    if(vm.count("ref_clock_freq")) {
        if(use_a) {
            // Implicilty sets source to external
            a_usrp->set_clock_reference_freq(ref_clock_freq);
        }
        if(use_b) {
            // Implicilty sets source to external
            b_usrp->set_clock_reference_freq(ref_clock_freq);
        }
    }

    if(use_a) {
        a_usrp->set_clock_source(ref);
    }
    if(use_b) {
        b_usrp->set_clock_source(ref);
    }

    if(use_a) {
        std::cout << boost::format("Using A Device: %s") % a_usrp->get_pp_string() << std::endl;
    }
    if(use_b) {
        std::cout << boost::format("Using B Device: %s") % b_usrp->get_pp_string() << std::endl;
    }

    // Sets the destination IP and port in rx_usrp to match tx_usrp for the specified channels
    // TODO verify if the channel combination is not impossible due to sending from 1 SFP port to multiple
    if(ab_num_channels != 0) {
        a_usrp->rx_to_tx(b_usrp, a_rx_channel_nums, b_tx_channel_nums);
    }
    if(ba_num_channels != 0 && !loopback_mode) {
        b_usrp->rx_to_tx(a_usrp, b_rx_channel_nums, a_tx_channel_nums);
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    /// Enable buffer level based stream on tx
    // Skip if in loopback mode and no channels are specified to avoid
    if(use_a && !loopback_mode) {
        a_usrp->tx_start_force_stream(a_tx_channel_nums);
    }
    if(use_b) {
        b_usrp->tx_start_force_stream(b_tx_channel_nums);
    }

    // Enable always stream on rx (must be done after tx)
    if(use_a) {
        a_usrp->rx_start_force_stream(a_rx_channel_nums);
    }
    if(use_b && !loopback_mode) {
        b_usrp->rx_start_force_stream(b_rx_channel_nums);
    }

    auto current_time = std::chrono::steady_clock::now();
    const auto start_time = current_time;
    bool duration_specified = vm.count("duration");
    std::chrono::time_point<std::chrono::steady_clock> stop_time;
    if(duration_specified) {
        stop_time = start_time + std::chrono::milliseconds(int64_t(1000 * duration));
    }

    while(!stop_signal_called && (stop_time < current_time || !duration_specified)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(duration_specified) {
            current_time = std::chrono::steady_clock::now();
        }
    }

    // Stops streaming
    if(use_a) {
        a_usrp->rx_stop_force_stream(a_rx_channel_nums);
    }
    if(use_b && !loopback_mode) {
        b_usrp->rx_stop_force_stream(b_rx_channel_nums);
    }

    if(use_a && !loopback_mode) {
        a_usrp->tx_stop_force_stream(a_tx_channel_nums);
    }
    if(use_b) {
        b_usrp->tx_stop_force_stream(b_tx_channel_nums);
    }

    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
