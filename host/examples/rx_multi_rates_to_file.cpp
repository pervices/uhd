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
#include <boost/filesystem.hpp>
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

class channel_group {
public:
    const double common_start_time_delay;
    const double common_rate;
    std::vector<size_t> channels;

    channel_group(size_t channel, double start_time_delay, double rate)
    : common_start_time_delay(start_time_delay),
    common_rate(rate),
    channels(std::vector<size_t>(1, channel))
    {
    }

    /** adds a channel to the group if it delay and rate match
    * @return True is the channel can be added to the group
    */
    bool add_channel(size_t channel, double start_time_delay, double rate) {
        if(common_start_time_delay == start_time_delay && common_rate == rate) {
            channels.push_back(channel);
            return true;
        } else {
            return false;
        }
    }
};

void receive_function(uhd::usrp::multi_usrp *usrp, channel_group *group_info, size_t requested_num_samps, size_t spb, std::string folder, bool skip_save, bool strict) {
    uhd::stream_args_t rx_stream_args("sc16");
    rx_stream_args.channels = group_info->channels;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);

    std::vector<std::ofstream> outfiles(group_info->channels.size());
    if (!skip_save) {
        for(size_t n = 0; n < group_info->channels.size(); n++) {
            std::string path = folder + "/rx_ch_" + std::to_string(group_info->channels[n]) + ".dat";
            outfiles[n].open(path.c_str(), std::ofstream::binary);
        }
    }

    // Metadata return struct
    uhd::rx_metadata_t md;

    // Configure and send stream command
    uhd::stream_cmd_t stream_cmd((requested_num_samps == 0) ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(requested_num_samps);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(group_info->common_start_time_delay);
    rx_stream->issue_stream_cmd(stream_cmd);

    // Buffers to store rx data in
    std::vector<std::vector<std::complex<short>>> buffers(group_info->channels.size(), std::vector<std::complex<short>>(spb, 0));

    // Pointers to where to store data for each channel
    std::vector<void*> buffer_ptrs(group_info->channels.size());
    for(size_t n = 0; n < buffers.size(); n++) {
        buffer_ptrs[n] = buffers[n].data();
    }

    size_t num_samples_received = 0;
    bool overflow_occured = false;
    // Receive loop
    while(!stop_signal_called && (num_samples_received < requested_num_samps || requested_num_samps == 0)) {
        size_t num_samples = rx_stream->recv(buffer_ptrs, spb, md, group_info->common_start_time_delay + 3);

        if(md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                fprintf(stderr, "Timeout received after %lu samples\n", num_samples_received);
                // Break if overflow occured and a set number of samples was requested, since that probably means all samples were sent, and missed ones weren't counted
                if(strict || (overflow_occured && requested_num_samps !=0)) {
                    break;
                }
            } else if(md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                if(!overflow_occured) {
                    fprintf(stderr, "Overflow occured after %lu samples\n", num_samples_received);
                    overflow_occured = true;
                    if(strict) {
                        break;
                    }
                }
            }
        }
        if(!skip_save) {
            for(size_t n = 0; n < group_info->channels.size(); n++) {
                outfiles[n].write((const char*)buffers[n].data(), num_samples * sizeof(std::complex<short>));
            }
        }
        num_samples_received += num_samples;
    }

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    for(auto& outfile : outfiles) {
        if(outfile.is_open()) {
            outfile.close();
        }
    }

    printf("Received %lu samples on ch", num_samples_received);
    for(auto& channel : group_info->channels) {
        printf(", %lu", channel);
    }
    printf("\n");
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{

    // variables to be set by po
    std::string args, folder, channel_arg, rate_arg, freq_arg, gain_arg, start_delay_arg;
    size_t total_num_samps, spb;

    // setup the program options
    po::options_description desc("Receives data from device and saves to to file. Supports using different sample rates per channel");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "uhd device address args")
        ("folder", po::value<std::string>(&folder)->default_value("results"), "name of the file to write binary samples to")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")

        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("rate", po::value<std::string>(&rate_arg)->default_value("1000000"), "rate of incoming samples. Can be channel specific")
        ("freq", po::value<std::string>(&freq_arg)->default_value("0"), "RF center frequency in Hz. Can be channel specific")
        ("gain", po::value<std::string>(&gain_arg), "gain for the RF chain. Can be channel specific")
        ("channels", po::value<std::string>(&channel_arg)->default_value("0"), "which channel(s) to use")
        ("start_delay", po::value<std::string>(&start_delay_arg)->default_value("0.0"), "The number of seconds to wait between issuing the stream command and starting streaming. Can be channel specific")
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

    bool skip_save = vm.count("null");
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

    std::vector<double> center_frequencies = parse_argument<double>(freq_arg, channels.size());
    set_parameter(double, usrp->set_rx_freq, usrp->get_rx_freq, channels, center_frequencies, "frequency");

    std::vector<double> gains = parse_argument<double>(gain_arg, channels.size());
    set_parameter(double, usrp->set_rx_gain, usrp->get_rx_gain, channels, gains, "gain");

    std::vector<double> start_delays = parse_argument<double>(start_delay_arg, channels.size());

    std::vector<channel_group> groups;

    for(size_t ch_i = 0; ch_i < channels.size(); ch_i++) {
        bool ch_added = false;
        for(size_t group_i = 0; group_i < groups.size(); group_i++) {
            bool add_ch_success = groups[group_i].add_channel(channels[ch_i], start_delays[ch_i], rates[ch_i]);
            if(add_ch_success) {
                ch_added = true;
                break;
            }
        }
        if(!ch_added) {
            groups.emplace_back(channel_group(channels[ch_i], start_delays[ch_i], rates[ch_i]));
        }
    }

    std::cout << ("Setting device timestamp to 0") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));

    if(!skip_save) {
        // This function is in the standard library of c++17
        boost::filesystem::create_directories(folder);
    }

    std::signal(SIGINT, &sig_int_handler);

    std::vector<std::thread> receive_threads;
    for(size_t n = 0; n < groups.size(); n++) {
        receive_threads.emplace_back(std::thread(receive_function, usrp.get(), &groups[n], total_num_samps, spb, folder, skip_save, strict));
    }

    for(auto& receive_thread : receive_threads) {
        receive_thread.join();
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
