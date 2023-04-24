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
#include <boost/algorithm/string.hpp>
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

/**
 * Seperates the arguments used for each device
 * @param parameter The parameter to be seperated
 * @param return A vector of strings for each device. Note this will likely need to be broken down further
 */
inline std::vector<std::string> seperate_device_argument(std::string parameter) {
    std::vector<std::string> device_strings;
    boost::split(device_strings, parameter, boost::is_any_of("\"' "));
    return device_strings;
}

/**
 * Seperates the arguments used for each device per channel
 * @param parameter The parameter to be seperated
 * @param return A vector of strings for each channel
 */
inline std::vector<std::string> seperate_channel_argument(std::string parameter) {
    std::vector<std::string> channel_strings;
    boost::split(channel_strings, parameter, boost::is_any_of("\"',"));
    return channel_strings;
}


inline bool validate_number_of_device_arguments(size_t num_devices, std::vector<std::string> device_arguments) {
    return device_arguments.size() == num_devices;
}

inline bool validate_number_of_channel_arguments(size_t num_channels, std::vector<std::string> channel_arguments) {
    return channel_arguments.size() == 1 || channel_arguments.size() == num_channels;
}

// Contains the parameters for each individual connected device
struct device_parameters {
    std::string args;
    size_t num_tx_channels;
    size_t num_rx_channels;
    std::vector<size_t> tx_channels, rx_channels;
    std::vector<double> tx_gains, rx_gains, tx_freqs, rx_freqs;
    std::string time_reference;
};

std::vector<device_parameters> parse_device_parameters(std::string args, std::string ref, std::string tx_channels_s, std::string rx_channels_s, std::string tx_gains_s, std::string rx_gains_s, std::string tx_freqs_s, std::string rx_freqs_s) {
    std::vector<std::string> device_args = seperate_device_argument(args);
    size_t num_devices = device_args.size();

    std::vector<std::string> device_ref = seperate_device_argument(ref);
    if(!validate_number_of_device_arguments(num_devices, device_ref)) {
        throw std::runtime_error("Incorrect number of devices a time reference was specified for");
    }

    std::vector<std::string> device_tx_channels_arg = seperate_device_argument(tx_channels_s);
    if(!validate_number_of_device_arguments(num_devices, device_tx_channels_arg)) {
        throw std::runtime_error("Incorrect number of devices tx channels were specified for");
    }
    std::vector<std::string> device_rx_channels_arg = seperate_device_argument(rx_channels_s);
    if(!validate_number_of_device_arguments(num_devices, device_rx_channels_arg)) {
        throw std::runtime_error("Incorrect number of devices rx channels were specified for");
    }

    std::vector<std::string> device_tx_gain_arg = seperate_device_argument(tx_gains_s);
    if(!validate_number_of_device_arguments(num_devices, device_tx_gain_arg)) {
        throw std::runtime_error("Incorrect number of devices tx gains were specified for");
    }
    std::vector<std::string> device_rx_gain_arg = seperate_device_argument(rx_gains_s);
    if(!validate_number_of_device_arguments(num_devices, device_rx_gain_arg)) {
        throw std::runtime_error("Incorrect number of devices rx gains were specified for");
    }

    std::vector<std::string>device_tx_freq_arg = seperate_device_argument(tx_freqs_s);
    if(!validate_number_of_device_arguments(num_devices, device_tx_freq_arg)) {
        throw std::runtime_error("Incorrect number of devices tx frequencies were specified for");
    }
    std::vector<std::string> device_rx_freq_arg = seperate_device_argument(rx_freqs_s);
    if(!validate_number_of_device_arguments(num_devices, device_rx_freq_arg)) {
        throw std::runtime_error("Incorrect number of devices rx frequencies were specified for");
    }

    std::vector<device_parameters> parameters(num_devices);
    for(size_t n = 0; n < num_devices; n++) {
        parameters[n].args = device_args[n];
        parameters[n].time_reference = device_ref[n];

        // Parse the channels
        if(device_tx_channels_arg[n] == "n") {
            parameters[n].num_tx_channels = 0;
        } else {
            std::vector<std::string> tx_channel_strings = seperate_channel_argument(device_tx_channels_arg[n]);
            parameters[n].num_tx_channels = tx_channel_strings.size();
            for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                parameters[n].tx_channels.push_back(std::stoi(tx_channel_strings[i]));
            }
        }

        if(device_rx_channels_arg[n] == "n") {
            parameters[n].num_rx_channels = 0;
        } else {
            std::vector<std::string> rx_channel_strings = seperate_channel_argument(device_rx_channels_arg[n]);
            parameters[n].num_rx_channels = rx_channel_strings.size();
            for(size_t i = 0; i < parameters[n].num_rx_channels; i++) {
                parameters[n].rx_channels.push_back(std::stoi(rx_channel_strings[i]));
            }
        }

        // Parse the gains
        if(device_tx_gain_arg[n] == "n") {
            if(parameters[n].num_tx_channels != 0) {
                throw std::runtime_error("Tx gains specified for a devie without tx channels");
            }
        } else {
            std::vector<std::string> tx_gain_arg_s = seperate_channel_argument(device_tx_gain_arg[n]);
            if(!validate_number_of_channel_arguments(parameters[n].num_tx_channels, tx_gain_arg_s)) {
                throw std::runtime_error("Mistmatch between number of tx channels specified and number of tx gains for a device");
            } else {
                if(tx_gain_arg_s.size()==1) {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].tx_gains.push_back(std::stod(tx_gain_arg_s[0]));
                    }
                }
                else {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].tx_gains.push_back(std::stod(tx_gain_arg_s[i]));
                    }
                }
            }
        }

        if(device_rx_gain_arg[n] == "n") {
            if(parameters[n].num_rx_channels != 0) {
                throw std::runtime_error("Rx gains specified for a device without rx channels");
            }
        } else {
            std::vector<std::string> rx_gain_arg_s = seperate_channel_argument(device_rx_gain_arg[n]);
            if(!validate_number_of_channel_arguments(parameters[n].num_rx_channels, rx_gain_arg_s)) {
                throw std::runtime_error("Mistmatch between number of rx channels specified and number of rx gains for a device");
            } else {
                if(rx_gain_arg_s.size()==1) {
                    for(size_t i = 0; i < parameters[n].num_rx_channels; i++) {
                        parameters[n].rx_gains.push_back(std::stod(rx_gain_arg_s[0]));
                    }
                }
                else {
                    for(size_t i = 0; i < parameters[n].num_rx_channels; i++) {
                        parameters[n].rx_gains.push_back(std::stod(rx_gain_arg_s[i]));
                    }
                }
            }
        }

        // Parse the frequencies
        if(device_tx_freq_arg[n] == "n") {
            if(parameters[n].num_tx_channels != 0) {
                throw std::runtime_error("Tx frequencies specified for a device without tx channels");
            }
        } else {
            std::vector<std::string> tx_freq_arg_s = seperate_channel_argument(device_tx_freq_arg[n]);
            if(!validate_number_of_channel_arguments(parameters[n].num_tx_channels, tx_freq_arg_s)) {
                throw std::runtime_error("Mistmatch between number of tx channels specified and number of tx frequencies for a device");
            } else {
                if(tx_freq_arg_s.size()==1) {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].tx_freqs.push_back(std::stod(tx_freq_arg_s[0]));
                    }
                }
                else {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].tx_freqs.push_back(std::stod(tx_freq_arg_s[i]));
                    }
                }
            }
        }

        if(device_rx_freq_arg[n] == "n") {
            if(parameters[n].num_rx_channels != 0) {
                throw std::runtime_error("Rx frequencies specified for a device without rx channels");
            }
        } else {
            std::vector<std::string> rx_freq_arg_s = seperate_channel_argument(device_rx_freq_arg[n]);
            if(!validate_number_of_channel_arguments(parameters[n].num_rx_channels, rx_freq_arg_s)) {
                throw std::runtime_error("Mistmatch between number of rx channels specified and number of rx frequencies for a device");
            } else {
                if(rx_freq_arg_s.size()==1) {
                    for(size_t i = 0; i < parameters[n].num_rx_channels; i++) {
                        parameters[n].rx_freqs.push_back(std::stod(rx_freq_arg_s[0]));
                    }
                }
                else {
                    for(size_t i = 0; i < parameters[n].num_rx_channels; i++) {
                        parameters[n].rx_freqs.push_back(std::stod(rx_freq_arg_s[i]));
                    }
                }
            }
        }
    }
    return parameters;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, ref, rx_channel_arg, tx_channel_arg, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg;
    double start_time;
    size_t total_num_samps;
    double rate;
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "Transmits a sinewave on multiple channels on multiple devices and receive data from multiple channels. Arguments with device specific parameter are sperated by spaces between devices and seperated by commas within a device. \"n\" indicates don't use this parameter for this device. Only specifying for one channel on a device indicates use the argument is for all channels on that device")
        ("args", po::value<std::string>(&args)->default_value(""), "Arguments for selecting devices. Provide device specific parameters")
        ("start_time", po::value<double>(&start_time)->default_value(2), "number of seconds in the future to begin receiving. (From when iniialization is complete, not from when the program is called")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming (Rx) and outgoing (Tx) samples")
        ("rx_channels", po::value<std::string>(&rx_channel_arg)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc). Provide device specific parameters")
        ("tx_channels", po::value<std::string>(&tx_channel_arg)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc). Provide device specific parameters")
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

    std::vector<device_parameters> parameters = parse_device_parameters(args, ref, tx_channel_arg, rx_channel_arg, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg);

    std::vector<uhd::usrp::multi_usrp::sptr> devices(parameters.size());
    // Connects to each device
    // TODO: parallelize connecting to devices
    for(size_t n = 0; n < devices.size(); n++) {
        std::cout << boost::format("Creating the usrp device with: %s...") % parameters[n].args << std::endl;
        devices[n] = uhd::usrp::multi_usrp::make(parameters[n].args);
    }
    
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
