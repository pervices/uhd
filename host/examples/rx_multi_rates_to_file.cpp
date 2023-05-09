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
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <chrono>

#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

#include <errno.h>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

/**
 * Parses arguments
 * @tparam T dataype of the argument
 * @param argument The argument, with channels seperated by ,
 * @param number_of_channels The expected number of channels. Set to 0 to automatically set to however many channels' worth of arguments are provided. If there are fewer channels with arguments than expected the last channel specified will be used for all
 * @return A vector with the seperated arguments, converted to the requested type
 */
template<typename T>
std::vector<T> parse_argument(std::string argument, size_t number_of_channels) {
    //detect which rx channels to use
    std::vector<std::string> seperated_argument_strings;
    boost::split(seperated_argument_strings, argument, boost::is_any_of("\"',"));

    if(number_of_channels == 0) {
        number_of_channels = seperated_argument_strings.size();
    }
    std::vector<T> values(number_of_channels);

    size_t last_channel_specified = 0;
    for(size_t n = 0; n < number_of_channels; n++) {
        if(n < seperated_argument_strings.size()) {
            std::stringstream(seperated_argument_strings[n]) >> values[n];
        } else {
            // Set any extra channels to the value of the last channel specified
            std::stringstream(seperated_argument_strings[last_channel_specified]) >> values[n];
        }
        last_channel_specified++;
    }

    return values;
}

/**
 * Sets various device parameters
 * @tparam T datatype of the parameter to set
 * @param set_func The function to be used to set the paramter on the device
 * @param get_func The function used to check if the parameter was set correctly
 * @param channels The channels to use
 * @param values A vector of the value of the paramter to be set
 * @param parameter_name The name of the parameter. Used in the print messages and nothing else
 * @return A vector with the seperated arguments, converted to the requested type
 */
#define set_parameter(T, set_func, get_func, channels, values, parameter_name) \
    for(size_t n = 0; n < channels.size(); n++) {\
        std::string value_s = std::to_string(values[n]);\
        printf("Setting %s on ch %lu to: %s\n", parameter_name, channels[n], value_s.c_str());\
        set_func(values[n], channels[n]);\
        T actual_value = get_func(channels[n]);\
\
        if(actual_value!=values[n]) {\
            std::string actual_value_s = std::to_string(actual_value);\
            printf("Actual value set: %s\n", actual_value_s.c_str());\
        }\
    }

int UHD_SAFE_MAIN(int argc, char* argv[])
{

    // variables to be set by po
    std::string args, folder, channel_arg, rate_arg;
    size_t total_num_samps, spb;
    double freq, gain, bw, total_time, setup_time, lo_offset, start_delay;

    // setup the program options
    po::options_description desc("Receives data from device and saves to to file. Supports using different sample rates per channel");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "uhd device address args")
        ("folder", po::value<std::string>(&folder)->default_value("results"), "name of the file to write binary samples to")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive")

        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("rate", po::value<std::string>(&rate_arg)->default_value("1000000"), "rate of incoming samples")
        ("freq", po::value<double>(&freq)->default_value(0), "RF center frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("channel", po::value<std::string>(&channel_arg)->default_value("0"), "which channel to use")
        ("start_delay", po::value<double>(&start_delay)->default_value(0.0), "The number of seconds to wait between issuing the stream command and starting streaming")
        ("null", "run without writing to file")
        ("strict", "Abort on a bad packet")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
        std::cout << std::endl
                  << "Receives data from device and saves to to file. Supports using different sample rates per channel\n"
                  << std::endl;
        return ~0;
    }

    bool no_save = vm.count("null");
    bool strict = vm.count("strict");

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args
              << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    std::vector<size_t> channels = parse_argument<size_t>(channel_arg, 0);
    std::vector<double> rates = parse_argument<double>(rate_arg, channels.size());

   set_parameter(double, usrp->set_rx_rate, usrp->get_rx_rate, channels, rates, "rate");

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
