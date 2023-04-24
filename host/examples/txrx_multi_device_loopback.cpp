//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include "wavetable.hpp"
#include <stdint.h>
#include <complex>
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <thread>
#include <vector>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

struct device_parameters {
    std::string args;
    std::vector<size_t> tx_channels, rx_channels;
    std::vector<double> tx_gains, rx_gains, tx_freqs, rx_freqs;
    std::string time_reference;
};

/**
 * Receives and saves rx data
 * @param rx_stream The streamer to receive data from
 * @param start_time The time in seconds to start streaming at
 * @param total_num_samps The number of samples to receive
 * @param data_file_fd File descriptor of the file to save data to
 */
void rx_run(uhd::rx_streamer::sptr rx_stream, double start_time, size_t total_num_samps, std::vector<int> recv_file_fd) {

}

/**
 * Send tx data
 * @param tx_stream The streamer to receive data from
 * @param start_time The time in seconds to start streaming at
 * @param total_num_samps The number of samples to receive
 */
void tx_run( uhd::tx_streamer::sptr tx_stream, double start_time, size_t total_num_samps) {

}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, ref, rx_channels, tx_channels, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg;
    double start_time;
    size_t total_num_samps;
    double rate;
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "Transmits a sinewave on multiple channels on multiple devices and receive data from multiple channels. Arguments with device specific parameter are sperated by spaces between devices and seperated by commas within a device. - indicates don't use this parameter for this device")
        ("args", po::value<std::string>(&args)->default_value(""), "Arguments for selecting devices. Provide device specific parameters")
        ("start_time", po::value<double>(&start_time)->default_value(2), "number of seconds in the future to begin receiving. (From when iniialization is complete, not from when the program is called")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming (Rx) and outgoing (Tx) samples")
        ("rx_channels", po::value<std::string>(&rx_channels)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc). Provide device specific parameters")
        ("tx_channels", po::value<std::string>(&tx_channels)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc). Provide device specific parameters")
        ("tx_gain", po::value<std::string>(&tx_gain_arg)->default_value("0"), "gain for the Tx RF chain. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("rx_gain", po::value<std::string>(&rx_gain_arg)->default_value("0"), "gain for the Rx RF chain. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("tx_freq", po::value<std::string>(&tx_freq_arg), "RF center frequency in Hz. Enter one number to set all the tx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("rx_freq", po::value<std::string>(&rx_freq_arg), "RF center frequency in Hz. Enter one number to set all the rx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "Numer of samples to send/receive")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "Whether to use an internal or external clock reference (internal, external)")
    ;
    
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("%s") % desc << std::endl;
        return ~0;
    }
    
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
