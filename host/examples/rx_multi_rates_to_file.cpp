//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2023-2024 Per Vices Corporation
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
#include <filesystem>
#include <uhd/utils/thread.hpp>

// C Linux IO
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <aio.h>

#include <algorithm>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

void free_recv_buffers(union sigval aiocb_to_free) {
    int ret = aio_error((aiocb*)(aiocb_to_free.sival_ptr));
    if(ret != 0) {
        UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "AIO write failed with error code: " + std::string(strerror(errno)));
    }

    free((void*)((aiocb*)(aiocb_to_free.sival_ptr))->aio_buf);

    free((aiocb*)(aiocb_to_free.sival_ptr));
}

// Waits for aio writes to finish, the closes the file descriptors
void close_aio(int output_fd[], size_t num_fds) {
    for(size_t ch = 0; ch < num_fds; ch++) {
        struct aiocb aiocbp_fsync;
        memset(&aiocbp_fsync, 0, sizeof(aiocb));
        aiocbp_fsync.aio_fildes = output_fd[ch];

        if(fsync(output_fd[ch])) {
            UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "fsync failed with " + std::string(strerror(errno)));
        }

        // Wait for aio writes to be written to disk
        if(aio_fsync(O_SYNC, &aiocbp_fsync)) {
            UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "aio_fsync failed with " + std::string(strerror(errno)));
        }
    }

    //Closes data files
    for(uint64_t n = 0; n < num_fds; n++) {
        close(output_fd[n]);
    }
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
            last_channel_specified = n;
        } else {
            // Set any extra channels to the value of the last channel specified
            std::stringstream(seperated_argument_strings[last_channel_specified]) >> values[n];
        }
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
    }\
    printf("\n");

class channel_group {
public:
    const double common_start_time_delay;
    const double common_rate;
    const size_t common_nsamps_requested;
    std::vector<size_t> channels;

    channel_group(size_t channel, double start_time_delay, double rate, size_t requested_num_samps)
    : common_start_time_delay(start_time_delay),
    common_rate(rate),
    common_nsamps_requested(requested_num_samps),
    channels(std::vector<size_t>(1, channel))
    {
    }

    /** adds a channel to the group if it delay and rate match
    * @return True is the channel can be added to the group
    */
    bool add_channel(size_t channel, double start_time_delay, double rate, size_t nsamps_requested) {
        if(common_start_time_delay == start_time_delay && common_rate == rate && common_nsamps_requested == nsamps_requested) {
            channels.push_back(channel);
            return true;
        } else {
            return false;
        }
    }
};

bool disk_rate_message_printed = false;

// output_fd: vector to store file descriptors in. Must be a pointer with length = group_info->channels.size()
void receive_function(uhd::usrp::multi_usrp *usrp, channel_group *group_info, size_t spb, std::string folder, bool skip_save, bool strict, int output_fd[]) {
    uhd::stream_args_t rx_stream_args("sc16");
    rx_stream_args.channels = group_info->channels;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);

    // Open files to write data to
    if(!skip_save) {
        for(size_t n = 0; n < group_info->channels.size(); n++) {
            std::string path = folder + "/rx_ch_" + std::to_string(group_info->channels.at(n)) + ".dat";
            output_fd[n] = open(path.c_str(), O_CREAT | O_WRONLY | O_LARGEFILE | O_TRUNC, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
            if(output_fd[n] == -1) {
                UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "Unable to open file: " + path + ". Failed with error code: " + strerror(errno));
                std::exit(errno);
            }

            // Allocates/advises kernel for write optimizations if the amount to of data required is known
            if(group_info->common_nsamps_requested != 0) {
                int64_t bytes_to_allocate = group_info->common_nsamps_requested * sizeof(std::complex<short>);

                int r = posix_fallocate64(output_fd[n], 0, bytes_to_allocate);
                if(r!=0) {
                    UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "Unable to allocate file: " + path + ". Failed with error code: " + strerror(errno));
                    std::exit(errno);
                }
            }
        }
    }

    // Metadata return struct
    uhd::rx_metadata_t md;

    // Configure and send stream command
    uhd::stream_cmd_t stream_cmd((group_info->common_nsamps_requested == 0) ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(group_info->common_nsamps_requested);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(group_info->common_start_time_delay);
    rx_stream->issue_stream_cmd(stream_cmd);

    size_t num_samples_received = 0;
    size_t num_writes_issued = 0;
    bool overflow_occured = false;
    // Receive loop
    while(!stop_signal_called && (num_samples_received < group_info->common_nsamps_requested || group_info->common_nsamps_requested == 0)) {
        // Initializes the to store rx data in
        std::vector<void*>buffer_ptrs(group_info->channels.size());
        struct aiocb* aiocb_info[group_info->channels.size()];

        size_t samples_this_recv;
        if(group_info->common_nsamps_requested != 0) {
            samples_this_recv = std::min(spb, group_info->common_nsamps_requested - num_samples_received);
        } else {
            samples_this_recv = spb;
        }

        // Pointers to where to store data for each channel
        for(size_t ch = 0; ch < group_info->channels.size(); ch++) {

            // Creates buffers to store rx data in
            buffer_ptrs[ch] = malloc(samples_this_recv * sizeof(std::complex<short>));
            if(buffer_ptrs[ch] == NULL) {
                printf("malloc failed\n");
                std::exit(~0);
            }

            memset(buffer_ptrs[ch], 0, samples_this_recv * sizeof(std::complex<short>));

            // Creates struct to contian data for io write
            aiocb_info[ch] = (aiocb*) malloc(sizeof(aiocb));
            memset(aiocb_info[ch], 0, sizeof(aiocb));

            aiocb_info[ch]->aio_fildes = output_fd[ch];
            aiocb_info[ch]->aio_offset = num_samples_received * sizeof(std::complex<short>);
            // By giving ascending priority the aio schedueler is able to more efficiently order writes
            aiocb_info[ch]->aio_offset = num_writes_issued;
            aiocb_info[ch]->aio_buf = buffer_ptrs[ch];

            aiocb_info[ch]->aio_sigevent.sigev_notify = SIGEV_THREAD;

            // Pointer passed to callback when async IO finishes. Used to know what memory to clean up
            aiocb_info[ch]->aio_sigevent.sigev_value.sival_ptr = aiocb_info[ch];

            aiocb_info[ch]->aio_sigevent.sigev_notify_function = &free_recv_buffers;
        }

        size_t num_samples = rx_stream->recv(buffer_ptrs, samples_this_recv, md, group_info->common_start_time_delay + 3);

        if(md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                fprintf(stderr, "Timeout received after %lu samples\n", num_samples_received);
                // Break if overflow occured and a set number of samples was requested, since that probably means all samples were sent, and missed ones weren't counted
                if(strict || (overflow_occured && group_info->common_nsamps_requested !=0)) {
                    // Set to true so other threads know to stop
                    stop_signal_called = true;
                    break;
                } else {
                    continue;
                }
            } else if(md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                if(!overflow_occured) {
                    fprintf(stderr, "Overflow occured after %lu samples\n", num_samples_received);
                    overflow_occured = true;
                    if(strict) {
                        // Set to true so other threads know to stop
                        stop_signal_called = true;
                        break;
                    } else {
                        continue;
                    }
                }
            }
        }

        num_samples_received += num_samples;

        if(!skip_save) {
            bool write_failed = false;
            for(size_t ch = 0; ch < group_info->channels.size(); ch++) {
                aiocb_info[ch]->aio_nbytes = num_samples * sizeof(std::complex<short>);

                // If this isn't fast enough look into lio_listio
                if(aio_write(aiocb_info[ch])) {
                    UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "Start AIO write failed for ch " + std::to_string(group_info->channels[ch]) + " with error code " + strerror(errno));
                    // Free data here since it is not getting freed by the aio_write callback
                    free(buffer_ptrs[ch]);
                    free(aiocb_info[ch]);
                    write_failed = true;
                    num_writes_issued++;
                }
            }
            // Exit recv loop if any of the writes failed and strict mode
            if(write_failed && strict) {
                // Set to true so other threads know to stop
                stop_signal_called = true;
                break;
            }
        } else {
            for(size_t ch = 0; ch < group_info->channels.size(); ch++) {
                // Free data buffer and write struct here since aio_write is unused
                // free is surprisingly slow, often writing will be faster
                free(buffer_ptrs[ch]);
                free(aiocb_info[ch]);
            }
        }

    }

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    printf("Received %lu samples on ch", num_samples_received);
    for(auto& channel : group_info->channels) {
        printf(", %lu", channel);
    }
    printf("\n");

    // Close the file descriptors for writing to disk later. Doing so here would impact performance in threads that are still receiving
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    uhd::set_thread_priority();

    // variables to be set by po
    std::string args, folder, channel_arg, rate_arg, freq_arg, gain_arg, start_delay_arg, nsamp_arg;
    size_t spb;

    // setup the program options
    po::options_description desc("Receives data from device and saves to to file. Supports using different sample rates per channel");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "uhd device address args")
        ("folder", po::value<std::string>(&folder)->default_value("results"), "name of the file to write binary samples to")
        ("nsamps", po::value<std::string>(&nsamp_arg)->default_value("0"), "total number of samples to receive. Setting this will improve performance. Can be channel specific")

        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("rate", po::value<std::string>(&rate_arg)->default_value("1000000"), "rate of incoming samples. Can be channel specific")
        ("freq", po::value<std::string>(&freq_arg)->default_value("0"), "RF center frequency in Hz. Can be channel specific")
        ("gain", po::value<std::string>(&gain_arg)->default_value("0"), "gain for the RF chain. Can be channel specific")
        ("channels", po::value<std::string>(&channel_arg)->default_value("0"), "which channel(s) to use")
        ("start_delay", po::value<std::string>(&start_delay_arg)->default_value("5"), "The number of seconds to wait between issuing the stream command and starting streaming. Can be channel specific")
        ("null", "run without writing to file")
        ("strict", "exit if any errors detected")
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

    std::vector<size_t> nsamps = parse_argument<size_t>(nsamp_arg, channels.size());

    std::vector<channel_group> groups;

    for(size_t ch_i = 0; ch_i < channels.size(); ch_i++) {
        bool ch_added = false;
        for(size_t group_i = 0; group_i < groups.size(); group_i++) {
            bool add_ch_success = groups[group_i].add_channel(channels[ch_i], start_delays[ch_i], rates[ch_i], nsamps[ch_i]);
            if(add_ch_success) {
                ch_added = true;
                break;
            }
        }
        if(!ch_added) {
            groups.emplace_back(channel_group(channels[ch_i], start_delays[ch_i], rates[ch_i], nsamps[ch_i]));
        }
    }

    std::cout << ("Setting device timestamp to 0") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));

    if(!skip_save) {
        std::filesystem::create_directories(folder);
    }

    std::signal(SIGINT, &sig_int_handler);

    std::vector<std::thread> receive_threads;
    // Create array to store file descriptors
    int* output_fd_groups[groups.size()];
    for(size_t n = 0; n < groups.size(); n++) {
        output_fd_groups[n] = (int*) malloc(groups[n].channels.size() * sizeof(int));
    }

    for(size_t n = 0; n < groups.size(); n++) {

        receive_threads.emplace_back(std::thread(receive_function, usrp.get(), &groups[n], spb, folder, skip_save, strict, output_fd_groups[n]));
    }

    for(auto& receive_thread : receive_threads) {
        receive_thread.join();
    }

    if(!skip_save) {
        for(size_t n = 0; n < groups.size(); n++) {
            close_aio(output_fd_groups[n], groups[n].channels.size());
            free(output_fd_groups[n]);
        }
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
