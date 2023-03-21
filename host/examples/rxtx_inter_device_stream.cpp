//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
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

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){
    stop_signal_called = true;
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){

    //variables to be set by po
    std::string rx_args, tx_args, ref, pps, rx_channel_list, tx_channel_list, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg;
    double rate, duration;
    int ref_clock_freq;
    std::vector<double> tx_gains, rx_gains, tx_freqs, rx_freqs;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("rx_args", po::value<std::string>(&rx_args)->default_value(""), "Identifying info unit receiving rf data. Example: rx_args=\"addr=192.168.10.2\"")
        ("tx_args", po::value<std::string>(&tx_args)->default_value(""), "Identifying info unit receiving rf data. Example: tx_args=\"addr=192.168.11.2\"")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("rx_freq", po::value<std::string>(&rx_freq_arg), "RF center frequency in Hz. Enter one number to set all the rx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("tx_freq", po::value<std::string>(&tx_freq_arg), "RF center frequency in Hz. Enter one number to set all the tx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("rx_gain", po::value<std::string>(&rx_gain_arg)->default_value("0"), "gain for the Rx RF chain. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("tx_gain", po::value<std::string>(&tx_gain_arg)->default_value("0"), "gain for the Tx RF chain. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\"")
        ("ref_clock_freq", po::value<int>(&ref_clock_freq), "Frequency of external reference clock. Program will use an internal 10MHz clock if not specified")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external)")
        ("rx_channels", po::value<std::string>(&rx_channel_list)->default_value("0"), "which rx channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx_channels", po::value<std::string>(&tx_channel_list)->default_value("0"), "which tx channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("duration", po::value<double>(&duration), "How long to stream for, will stream until the program is exited if unspecified")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
    }

    // Stores whether or not to use rx or tx, useful for debugging
    //Must be set before adjusting args to bypass clock sync
    const bool use_rx = rx_args!="";
    const bool use_tx = tx_args!="";

    // Lets UHD knwo no to use clock sync when creating the device
    rx_args+=",bypass_clock_sync=true";
    tx_args+=",bypass_clock_sync=true";

    //create a usrp device
    std::cout << std::endl;
    uhd::usrp::multi_usrp::sptr rx_usrp;
    uhd::usrp::multi_usrp::sptr tx_usrp;
    if(!use_rx && !use_tx) {
        std::cerr << "ERROR: No device adress specified. Since this program is intended to run with multiple devices connected to the same host, you must specify which devices are used for each role" << std::endl;
        return ~0;
    }
    else {
        if(use_rx) {
            rx_args+=",bypass_clock_sync=true";
            std::cout << boost::format("Creating the usrp device with: %s...") % rx_args << std::endl;
            rx_usrp = uhd::usrp::multi_usrp::make(rx_args);
        }
        else {
            std::cerr << "ERROR: No device adress specified for rx, rx will not be used. Note: runing this without rx is only useful for checking if rx is configured correctly" << std::endl;
        }
        if(use_tx) {
            tx_args+=",bypass_clock_sync=true";
            std::cout << boost::format("Creating the usrp device with: %s...") % tx_args << std::endl;
            tx_usrp = uhd::usrp::multi_usrp::make(tx_args);
        }
        else {
            std::cerr << "ERROR: No device adress specified for tx, tx will not be used. Note: runing this without tx is only useful for checking if tx is configured correctly" << std::endl;
        }
    }

    //detect which rx channels to use
    std::vector<std::string> rx_channel_strings;
    std::vector<size_t> rx_channel_nums;
    boost::split(rx_channel_strings, rx_channel_list, boost::is_any_of("\"',"));
    if(use_rx) {
        for(size_t ch = 0; ch < rx_channel_strings.size(); ch++){
            size_t chan = std::stoi(rx_channel_strings[ch]);
            if(chan >= rx_usrp->get_rx_num_channels()) {
                throw std::runtime_error("Invalid channel(s) specified.");
            }
            else
                rx_channel_nums.push_back(std::stoi(rx_channel_strings[ch]));
        }
    }

    //detect which tx channels to use
    std::vector<std::string> tx_channel_strings;
    std::vector<size_t> tx_channel_nums;
    boost::split(tx_channel_strings, tx_channel_list, boost::is_any_of("\"',"));
    if(use_tx) {
        for(size_t ch = 0; ch < tx_channel_strings.size() && use_tx; ch++){
            size_t chan = std::stoi(tx_channel_strings[ch]);
            if(chan >= tx_usrp->get_tx_num_channels()) {
                throw std::runtime_error("Invalid channel(s) specified.");
            }
            else
                tx_channel_nums.push_back(std::stoi(tx_channel_strings[ch]));
        }
    }

    size_t num_channels = tx_channel_strings.size();
    if(num_channels != rx_channel_strings.size()) {
        throw std::runtime_error("Mistmatch between number of rx and tx channels requested");
    }

std::vector<std::string> tx_gain_strings, rx_gain_strings, tx_freq_strings, rx_freq_strings;
    boost::split(tx_gain_strings, tx_gain_arg, boost::is_any_of("\"',"));
    boost::split(rx_gain_strings, rx_gain_arg, boost::is_any_of("\"',"));
    boost::split(tx_freq_strings, tx_freq_arg, boost::is_any_of("\"',"));
    boost::split(rx_freq_strings, rx_freq_arg, boost::is_any_of("\"',"));

    // Error checking to make sure number of channels specific settings are provided for matches the number of channels
    bool all_tx_same_gain = tx_gain_strings.size() == 1;
    if((!all_tx_same_gain) && num_channels != tx_gain_strings.size()) {
        throw std::runtime_error("Mistmatch between number of tx channels specified and number of channels gain is provided for");
    }
    bool all_rx_same_gain = rx_gain_strings.size() == 1;
    if((!all_rx_same_gain) && num_channels != rx_gain_strings.size()) {
        throw std::runtime_error("Mistmatch between number of rx channels specified and number of channels gain is provided for");
    }
    bool all_tx_same_freq = tx_freq_strings.size() == 1;
    if((!all_tx_same_gain) && num_channels != tx_gain_strings.size()) {
        throw std::runtime_error("Mistmatch between number of tx channels specified and number of channels gain is provided for");
    }
    bool all_rx_same_freq = rx_freq_strings.size() == 1;
    if((!all_rx_same_freq) && num_channels != rx_freq_strings.size()) {
        throw std::runtime_error("Mistmatch between number of tx channels specified and number of channels gain is provided for");
    }

    if(all_tx_same_gain) {
        tx_gains = std::vector<double>(num_channels, std::stod(tx_gain_strings[0]));
    } else {
        tx_gains = std::vector<double>(num_channels);
        for(size_t n = 0; n < num_channels; n++) {
            tx_gains[n] = std::stod(tx_gain_strings[n]);
        }
    }

    if(all_rx_same_gain) {
        rx_gains = std::vector<double>(num_channels, std::stod(rx_gain_strings[0]));
    } else {
        rx_gains = std::vector<double>(num_channels);
        for(size_t n = 0; n < num_channels; n++) {
            rx_gains[n] = std::stod(rx_gain_strings[n]);
        }
    }

    if(all_tx_same_freq) {
        tx_freqs = std::vector<double>(num_channels, std::stod(tx_freq_strings[0]));
    } else {
        tx_freqs = std::vector<double>(num_channels);
        for(size_t n = 0; n < num_channels; n++) {
            tx_freqs[n] = std::stod(tx_freq_strings[n]);
        }
    }

    if(all_rx_same_freq) {
        rx_freqs = std::vector<double>(num_channels, std::stod(rx_freq_strings[0]));
    } else {
        rx_freqs = std::vector<double>(num_channels);
        for(size_t n = 0; n < num_channels; n++) {
            rx_freqs[n] = std::stod(rx_freq_strings[n]);
        }
    }

    if(vm.count("ref_clock_freq")) {
        if(use_rx) {
            // Implicilty sets source to external
            rx_usrp->set_clock_reference_freq(ref_clock_freq);
        }
        if(use_tx) {
            // Implicilty sets source to external
            tx_usrp->set_clock_reference_freq(ref_clock_freq);
        }
    } else {
        if(use_rx) {
            rx_usrp->set_clock_source("internal");
        }
        if(use_tx) {
            tx_usrp->set_clock_source("internal");
        }
    }

    if(use_rx) {
        std::cout << boost::format("Using rx Device: %s") % rx_usrp->get_pp_string() << std::endl;
    }
    if(use_tx) {
        std::cout << boost::format("Using tx Device: %s") % tx_usrp->get_pp_string() << std::endl;
    }

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }

    if(use_rx) {
        std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
        rx_usrp->set_rx_rate(rate);
        rate = rx_usrp->get_rx_rate();
        std::cout << boost::format("Actual RX Rate: %f Msps...") % (rate/1e6) << std::endl << std::endl;
    }

    if(use_tx) {
        std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
        tx_usrp->set_tx_rate(rate);
        rate = tx_usrp->get_tx_rate();
        std::cout << boost::format("Actual TX Rate: %f Msps....") % (rate/1e6) << std::endl << std::endl;
    }

    if(use_rx) {
        for(size_t ch = 0; ch < rx_channel_nums.size(); ch++) {
            std::cout << boost::format("Setting ch %u RX Freq: %f MHz...") % rx_channel_nums[ch] % (rx_freqs[ch]/1e6) << std::endl;
            uhd::tune_request_t rx_tune_request(rx_freqs[ch]);
            rx_usrp->set_rx_freq(rx_tune_request, rx_channel_nums[ch]);
            std::cout << boost::format("Actual RX ch %u Freq: %f MHz...") % rx_channel_nums[ch] % (rx_usrp->get_rx_freq(rx_channel_nums[ch])/1e6) << std::endl << std::endl;

            std::cout << boost::format("Setting ch%i RX Gain: %f dB...") % rx_channel_nums[ch] % rx_gains[ch] << std::endl;
            rx_usrp->set_rx_gain(rx_gains[ch], rx_channel_nums[ch]);
            std::cout << boost::format("Actual ch%i RX Gain: %f dB...") % rx_channel_nums[ch] % rx_usrp->get_rx_gain(rx_channel_nums[ch]) << std::endl << std::endl;
        }
    }

    if(use_tx) {
        for(size_t ch = 0; ch < tx_channel_nums.size(); ch++) {
            std::cout << boost::format("Setting ch %u TX Freq: %f MHz...") % tx_channel_nums[ch] % (tx_freqs[ch]/1e6) << std::endl;
            uhd::tune_request_t tx_tune_request(tx_freqs[ch]);
            tx_usrp->set_tx_freq(tx_tune_request, tx_channel_nums[ch]);
            std::cout << boost::format("Actual TX ch %u Freq: %f MHz...") % tx_channel_nums[ch] % (tx_usrp->get_tx_freq(tx_channel_nums[ch])/1e6) << std::endl << std::endl;

            std::cout << boost::format("Setting ch%i TX Gain: %f dB...") % tx_channel_nums[ch] % tx_gains[ch] << std::endl;
            tx_usrp->set_tx_gain(tx_gains[ch], tx_channel_nums[ch]);
            std::cout << boost::format("Actual ch%i TX Gain: %f dB...") % tx_channel_nums[ch] % tx_usrp->get_tx_gain(tx_channel_nums[ch]) << std::endl << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1)); //allow for some setup time

    if(use_rx && use_tx) {
        rx_usrp->rx_to_tx(tx_usrp, rx_channel_nums, tx_channel_nums);
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    if(use_rx) {
        rx_usrp->set_time_now(0.0);
    }
    if(use_tx) {
        tx_usrp->set_time_now(0.0);
    }

    if(use_tx) {
        tx_usrp->tx_start_force_stream(tx_channel_nums);
    }

    if(use_rx) {
        rx_usrp->rx_start_force_stream(rx_channel_nums);
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

    if(use_tx) {
        tx_usrp->tx_stop_force_stream(tx_channel_nums);
    }

    if(use_rx) {
        rx_usrp->rx_stop_force_stream(rx_channel_nums);
    }

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
