//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2023 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "wave_generator.hpp"
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/trigger_options.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <format>
#include <stdint.h>
#include <iostream>
#include <csignal>
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <vector>
#include <memory>
#include <new>
#include <algorithm>
#include <complex>
#include <functional>

#include <uhd/types/tune_request.hpp>

namespace po = boost::program_options;

// Whether a channel listens to (input) or drives (output) a trigger
enum class trigger_sel { input, output, none };

/**
 * Splits a comma separated list, i.e. "0,1,2"
 * @param list The list to split
 * @return Each element of the list
 */
std::vector<std::string> split_list(const std::string& list) {
    std::vector<std::string> elements;
    boost::split(elements, list, boost::is_any_of("\"',"));
    return elements;
}

/**
 * Parses an argument that has a value per channel.
 * Specifying one value applies it to every channel. Otherwise there must be one value per channel.
 * @param argument The argument as entered by the user, i.e. "edge" or "edge,level"
 * @param argument_name The name of the argument, used in error messages
 * @param num_channels The number of channels in use
 * @param parse_value Converts one element of the list to T
 * @return One value per channel
 */
template <typename T, typename F>
std::vector<T> parse_per_channel(const std::string& argument, const std::string& argument_name, size_t num_channels, F parse_value) {
    std::vector<std::string> value_strings = split_list(argument);

    if(value_strings.size() != 1 && value_strings.size() != num_channels) {
        throw std::runtime_error(std::format("--{} has {} values but {} channels are in use. Specify one value for all channels or one value per channel", argument_name, value_strings.size(), num_channels));
    }

    std::vector<T> values;
    for(size_t ch = 0; ch < num_channels; ch++) {
        // A single value applies to every channel
        const std::string& value_string = (value_strings.size() == 1) ? value_strings[0] : value_strings[ch];
        values.push_back(parse_value(value_string, argument_name));
    }
    return values;
}

/**
 * Converts "edge" or "level" to a trigger mode
 */
uhd::trigger_pv::trigger_mode_t parse_trigger_mode(const std::string& value, const std::string& argument_name) {
    if(value == "edge") {
        return uhd::trigger_pv::trigger_mode_t::edge;
    } else if(value == "level") {
        return uhd::trigger_pv::trigger_mode_t::level;
    } else {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Valid values: edge, level", value, argument_name));
    }
}

/**
 * Converts "input", "output" or "none" to a trigger_sel
 */
trigger_sel parse_trigger_sel(const std::string& value, const std::string& argument_name) {
    if(value == "input") {
        return trigger_sel::input;
    } else if(value == "output") {
        return trigger_sel::output;
    } else if(value == "none") {
        return trigger_sel::none;
    } else {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Valid values: input, output, none", value, argument_name));
    }
}

/**
 * Converts "positive" or "negative" to a trigger polarity
 */
uhd::trigger_pv::polarity_t parse_polarity(const std::string& value, const std::string& argument_name) {
    if(value == "positive") {
        return uhd::trigger_pv::polarity_t::positive;
    } else if(value == "negative") {
        return uhd::trigger_pv::polarity_t::negative;
    } else {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Valid values: positive, negative", value, argument_name));
    }
}

/**
 * Converts "dsp" or "output" to a tx gating mode
 */
uhd::trigger_pv::gating_t parse_gating(const std::string& value, const std::string& argument_name) {
    if(value == "dsp") {
        return uhd::trigger_pv::gating_t::dsp;
    } else if(value == "output") {
        return uhd::trigger_pv::gating_t::output;
    } else {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Valid values: dsp, output", value, argument_name));
    }
}

/**
 * Converts a number to a non-negative integer. Used for numbers of samples and ticks
 */
int64_t parse_non_negative(const std::string& value, const std::string& argument_name) {
    size_t characters_used = 0;
    int64_t number = 0;
    try {
        number = std::stoll(value, &characters_used);
    } catch(const std::exception&) {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Must be a number", value, argument_name));
    }
    // Reject values like "12abc"
    if(characters_used != value.size()) {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Must be a number", value, argument_name));
    }
    if(number < 0) {
        throw std::runtime_error(std::format("Invalid value \"{}\" for --{}. Must not be negative", value, argument_name));
    }
    return number;
}

/**
 * Combines a channel's sma and ufl selections into its trig_sel
 */
uhd::trigger_pv::active_trigger_t make_trig_sel(trigger_sel sma_sel, trigger_sel ufl_sel) {
    using namespace uhd::trigger_pv;

    active_trigger_t trig_sel = active_trigger_t::none;
    if(sma_sel == trigger_sel::input) {
        trig_sel |= active_trigger_t::sma_input;
    } else if(sma_sel == trigger_sel::output) {
        trig_sel |= active_trigger_t::sma_output;
    }
    // Whether the ufl is an input or output is set by ufl_dir (see ufl_direction)
    if(ufl_sel != trigger_sel::none) {
        trig_sel |= active_trigger_t::channel_ufl;
    }
    return trig_sel;
}

/**
 * The ufl direction for a channel's ufl selection
 */
uhd::trigger_pv::pin_direction_t ufl_direction(trigger_sel ufl_sel) {
    using namespace uhd::trigger_pv;

    if(ufl_sel == trigger_sel::input) {
        return pin_direction_t::in;
    } else if(ufl_sel == trigger_sel::output) {
        return pin_direction_t::out;
    } else {
        // The ufl isn't used, so leave its direction as is
        return pin_direction_t::get;
    }
}

// Cache line size
// Assume it is 64, which is the case for virtually all AMD64 systems
constexpr size_t CACHE_LINE_SIZE = 64;

/**
 * Allocator for std::vector that starts the memory on a cache line boundary
 */
template <typename T>
struct cache_aligned_allocator {
    using value_type = T;

    cache_aligned_allocator() = default;
    template <typename U>
    cache_aligned_allocator(const cache_aligned_allocator<U>&) {}

    T* allocate(size_t n) {
        return static_cast<T*>(::operator new(n * sizeof(T), std::align_val_t(CACHE_LINE_SIZE)));
    }
    void deallocate(T* p, size_t) {
        ::operator delete(p, std::align_val_t(CACHE_LINE_SIZE));
    }

    template <typename U>
    bool operator==(const cache_aligned_allocator<U>&) const { return true; }
};

// Samples to send, shared between every tx thread
using tx_buffer_t = std::vector<std::complex<short>, cache_aligned_allocator<std::complex<short>>>;
// Samples received by one rx thread
using rx_buffer_t = std::vector<std::complex<short>, cache_aligned_allocator<std::complex<short>>>;

static std::atomic<bool> stop_signal_called(false);
void sig_int_handler(int){
    stop_signal_called = true;
}

/**
 * Waits for every thread in threads to finish
 */
void join_threads(std::vector<std::thread>& threads) {
    for(std::thread& thread : threads) {
        if(thread.joinable()) {
            thread.join();
        }
    }
}


/**
 * The path of the file to save a batch of rx samples to. Same naming as txrx_trigger_loopback, with an underscore added between the batch and channel numbers
 * @param results_directory The directory to save results into
 * @param channel The channel number
 * @param batch_number Which batch the file is for. Ignored when overwrite is set
 * @param overwrite Use the same file for every batch instead of a new file per batch
 */
std::string rx_result_path(const std::string& results_directory, size_t channel, uint64_t batch_number, bool overwrite) {
    if(overwrite) {
        return std::format("{}/ch_{}_result.dat", results_directory, channel);
    } else {
        return std::format("{}/burst_{}_ch_{}_result.dat", results_directory, batch_number, channel);
    }
}

// The maximum number of samples in the rx buffer
constexpr size_t MAX_RX_BUFFER = 100e6;

/**
 * Receives samples on one rx channel in continuous mode (rx_burst_lengths 0). A different function handles burst mode.
 * Samples that arrive close together are batched and each batch is saved to its own file.
 * For common edge trigger settings each batch will be one trigger.
 * @param streamer The streamer for this channel only
 * @param channel The channel number, used in messages and file names
 * @param start_time The device time to start receiving at
 * @param rate The channel's actual rx sample rate. Used to choose the recv timeout
 * @param burst_length Must be 0 (continuous)
 * @param total_samples Samples to receive in total. 0 receives until Ctrl + C is pressed
 * @param results_directory The directory to save results into. Must already exist
 * @param overwrite Use the same file for every batch instead of a new file per batch
 */
void rx_continous_function(uhd::rx_streamer::sptr streamer, size_t channel, uhd::time_spec_t start_time, double rate, uint64_t burst_length, uint64_t total_samples, std::string results_directory, bool overwrite) {

    if(burst_length != 0) {
        throw std::runtime_error(std::format("RX ch{}: rx_continous_function only supports continuous mode (burst length 0), got a burst length of {}. Burst mode is handled by rx_burst_function", channel, burst_length));
    }

    // Only allocate as much as is needed if the total is small
    size_t buffer_size = MAX_RX_BUFFER;
    if(total_samples != 0) {
        buffer_size = std::min<uint64_t>(MAX_RX_BUFFER, total_samples);
    }
    rx_buffer_t buff(buffer_size);

    // How long it would take to receive 50e3 samples
    // Once a batch has started, going this long without samples ends the batch
    const double timeout = 50e3 / rate;

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time;
    streamer->issue_stream_cmd(stream_cmd);

    uint64_t total_samples_recv = 0;
    uint64_t batch_number = 0;
    // Batch loop
    while((total_samples == 0 || total_samples_recv < total_samples) && !stop_signal_called) {
        // Opened once the batch's first samples arrive. Opening it earlier would leave an empty file if Ctrl + C is pressed
        // while waiting, and with overwrite would erase the previous batch
        std::ofstream outfile;

        uint64_t samples_this_batch = 0;
        bool batch_in_progress = true;
        while(batch_in_progress) {
            // Fill the buffer until it is full, the batch ends, or the total is reached
            size_t samples_in_buffer = 0;
            bool timed_out = false;
            do {
                // Receive up to the space left in the buffer, without going past the total
                uint64_t samples_to_recv = buff.size() - samples_in_buffer;
                if(total_samples != 0) {
                    samples_to_recv = std::min(samples_to_recv, total_samples - total_samples_recv);
                }

                uhd::rx_metadata_t md;
                size_t samples_this_recv = streamer->recv(&buff[samples_in_buffer], samples_to_recv, md, timeout);
                samples_in_buffer += samples_this_recv;
                samples_this_batch += samples_this_recv;
                total_samples_recv += samples_this_recv;

                timed_out = samples_this_recv == 0 && md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
                if(md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE && md.error_code != uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                    std::cerr << std::format("RX ch{}: {}\n", channel, md.strerror());
                }
            // Timeouts before the batch's first sample are waiting for the batch to start, so keep receiving
            } while((total_samples == 0 || total_samples_recv < total_samples) && samples_in_buffer < buff.size() && !stop_signal_called && (!timed_out || samples_this_batch == 0));

            // A well built program would save the samples in a different thread.
            // But since this is a simple example we will save them here.
            if(samples_in_buffer > 0) {
                if(!outfile.is_open()) {
                    outfile.open(rx_result_path(results_directory, channel, batch_number, overwrite), std::ofstream::binary);
                }
                outfile.write(reinterpret_cast<const char*>(buff.data()), samples_in_buffer * sizeof(buff[0]));
            }

            // The batch ends when samples stop arriving, the total is reached, or Ctrl + C is pressed
            bool total_reached = total_samples != 0 && total_samples_recv >= total_samples;
            batch_in_progress = !(stop_signal_called || total_reached || (timed_out && samples_this_batch > 0));
        }

        // A batch with no samples only happens when Ctrl + C is pressed while waiting for it to start
        if(samples_this_batch > 0) {
            outfile.close();
            std::cout << std::format("RX ch{}: received {} samples in batch {}\n", channel, samples_this_batch, batch_number);
            batch_number++;
        }
    }

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    stream_cmd.stream_now = true;
    streamer->issue_stream_cmd(stream_cmd);

    std::cout << std::format("RX ch{}: received {} samples\n", channel, total_samples_recv);
}

/**
 * Issues the stream command for the first burst that starts after the current device time.
 * Bursts start at first_burst_time + n * burst_increment. Bursts that have already started are skipped.
 * i.e. with the first burst at 5, burst_increment 2 and the current time 9.5, the burst at 11 is requested
 * @param streamer The streamer to issue the command to
 * @param device_now The current device time
 * @param first_burst_time The device time of the first burst
 * @param burst_increment Seconds between the start of each burst
 * @param burst_length Samples per burst
 * @param rate The channel's actual rx sample rate
 * @return The device time the requested burst finishes at
 */
uhd::time_spec_t issue_next_burst_cmd(uhd::rx_streamer::sptr streamer, uhd::time_spec_t device_now, uhd::time_spec_t first_burst_time, double burst_increment, uint64_t burst_length, double rate) {
    // Which burst to request
    uint64_t burst_number = 0;
    if(device_now >= first_burst_time) {
        burst_number = static_cast<uint64_t>((device_now - first_burst_time).get_real_secs() / burst_increment) + 1;
    }
    uhd::time_spec_t burst_start_time = first_burst_time + uhd::time_spec_t(burst_number * burst_increment);

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = burst_length;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = burst_start_time;
    streamer->issue_stream_cmd(stream_cmd);

    // The burst lasts as long as burst_length samples take, even if input triggers mean fewer samples are received
    return burst_start_time + uhd::time_spec_t(burst_length / rate);
}

/**
 * Receives samples on one rx channel in burst mode (rx_burst_lengths not 0). A different function handles continuous mode.
 * Batches and saving work the same as rx_continous_function: samples that arrive close together are batched and
 * each batch is saved to its own file. A burst may contain several batches if input triggers are used.
 * The next burst is requested once the previous one has finished.
 * @param usrp The device. Used to get the current device time
 * @param streamer The streamer for this channel only
 * @param channel The channel number, used in messages and file names
 * @param first_burst_time The device time to start the first burst at
 * @param rate The channel's actual rx sample rate. Used to choose the recv timeout and burst durations
 * @param burst_increment Seconds between the start of each burst
 * @param burst_length Samples per burst. Must not be 0 (continuous)
 * @param total_samples Samples to receive in total. 0 receives until Ctrl + C is pressed
 * @param results_directory The directory to save results into. Must already exist
 * @param overwrite Use the same file for every batch instead of a new file per batch
 */
void rx_burst_function(uhd::usrp::multi_usrp::sptr usrp, uhd::rx_streamer::sptr streamer, size_t channel, uhd::time_spec_t first_burst_time, double rate, double burst_increment, uint64_t burst_length, uint64_t total_samples, std::string results_directory, bool overwrite) {

    if(burst_length == 0) {
        throw std::runtime_error(std::format("RX ch{}: rx_burst_function only supports burst mode (burst length not 0). Continuous mode is handled by rx_continous_function", channel));
    }

    // Only allocate as much as is needed if the total is small
    size_t buffer_size = MAX_RX_BUFFER;
    if(total_samples != 0) {
        buffer_size = std::min<uint64_t>(MAX_RX_BUFFER, total_samples);
    }
    rx_buffer_t buff(buffer_size);

    // How long it would take to receive 50e3 samples
    // Once a batch has started, going this long without samples ends the batch
    const double timeout = 50e3 / rate;

    // Request the first burst
    uhd::time_spec_t device_now = usrp->get_time_now();
    uhd::time_spec_t burst_end_time = issue_next_burst_cmd(streamer, device_now, first_burst_time, burst_increment, burst_length, rate);

    // Getting the device time requires communicating with the device, so avoid doing it every timeout.
    // Instead estimate when the burst will finish using the host's clock and only check the device time after that.
    std::chrono::steady_clock::time_point next_time_check = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>((burst_end_time - device_now).get_real_secs()));

    uint64_t total_samples_recv = 0;
    uint64_t batch_number = 0;
    // Batch loop
    while((total_samples == 0 || total_samples_recv < total_samples) && !stop_signal_called) {
        // Opened once the batch's first samples arrive. Opening it earlier would leave an empty file if Ctrl + C is pressed
        // while waiting, and with overwrite would erase the previous batch
        std::ofstream outfile;

        uint64_t samples_this_batch = 0;
        bool batch_in_progress = true;
        while(batch_in_progress) {
            // Fill the buffer until it is full, the batch ends, or the total is reached
            size_t samples_in_buffer = 0;
            bool timed_out = false;
            do {
                // Receive up to the space left in the buffer, without going past the total
                uint64_t samples_to_recv = buff.size() - samples_in_buffer;
                if(total_samples != 0) {
                    samples_to_recv = std::min(samples_to_recv, total_samples - total_samples_recv);
                }

                uhd::rx_metadata_t md;
                size_t samples_this_recv = streamer->recv(&buff[samples_in_buffer], samples_to_recv, md, timeout);
                samples_in_buffer += samples_this_recv;
                samples_this_batch += samples_this_recv;
                total_samples_recv += samples_this_recv;

                timed_out = samples_this_recv == 0 && md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
                if(md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE && md.error_code != uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                    std::cerr << std::format("RX ch{}: {}\n", channel, md.strerror());
                }

                // A timeout means we are between bursts (or waiting for a trigger). Request the next burst once the previous one has finished
                if(timed_out && std::chrono::steady_clock::now() >= next_time_check) {
                    device_now = usrp->get_time_now();
                    if(device_now >= burst_end_time) {
                        burst_end_time = issue_next_burst_cmd(streamer, device_now, first_burst_time, burst_increment, burst_length, rate);
                    }
                    next_time_check = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>((burst_end_time - device_now).get_real_secs()));
                }
            // Timeouts before the batch's first sample are waiting for the batch to start, so keep receiving
            } while((total_samples == 0 || total_samples_recv < total_samples) && samples_in_buffer < buff.size() && !stop_signal_called && (!timed_out || samples_this_batch == 0));

            // A well built program would save the samples in a different thread.
            // But since this is a simple example we will save them here.
            if(samples_in_buffer > 0) {
                if(!outfile.is_open()) {
                    outfile.open(rx_result_path(results_directory, channel, batch_number, overwrite), std::ofstream::binary);
                }
                outfile.write(reinterpret_cast<const char*>(buff.data()), samples_in_buffer * sizeof(buff[0]));
            }

            // The batch ends when samples stop arriving, the total is reached, or Ctrl + C is pressed
            bool total_reached = total_samples != 0 && total_samples_recv >= total_samples;
            batch_in_progress = !(stop_signal_called || total_reached || (timed_out && samples_this_batch > 0));
        }

        // A batch with no samples only happens when Ctrl + C is pressed while waiting for it to start
        if(samples_this_batch > 0) {
            outfile.close();
            std::cout << std::format("RX ch{}: received {} samples in batch {}\n", channel, samples_this_batch, batch_number);
            batch_number++;
        }
    }

    // Cancel the burst in progress or waiting to start, if any
    uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    stop_cmd.stream_now = true;
    streamer->issue_stream_cmd(stop_cmd);

    std::cout << std::format("RX ch{}: received {} samples\n", channel, total_samples_recv);
}

/**
 * Sends samples on one tx channel
 * @param streamer The streamer for this channel only
 * @param channel The channel number, used in messages
 * @param buff The samples to send. Must hold at least the streamer's spb + fundamental_period samples. Only read, so every channel's thread can share it
 * @param fundamental_period How many samples the wave takes to repeat exactly (see wave_generator::get_fundamental_period)
 * @param first_burst_time The device time to start the first burst at
 * @param burst_increment Seconds between the start of each burst
 * @param burst_length Samples per burst. 0 sends one continuous burst
 * @param total_samples Samples to send in total. 0 sends until Ctrl + C is pressed
 */
void tx_function(uhd::tx_streamer::sptr streamer, size_t channel, const tx_buffer_t& buff, size_t fundamental_period, uhd::time_spec_t first_burst_time, double burst_increment, uint64_t burst_length, uint64_t total_samples) {
    // Send 10 packets worth of samples at a time
    const size_t spb = streamer->get_max_num_samps() * 10;

    uint64_t total_samples_sent = 0;
    uint64_t num_bursts = 0;
    while((total_samples == 0 || total_samples_sent < total_samples) && !stop_signal_called) {
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst = false;
        md.has_time_spec = true;
        md.time_spec = first_burst_time + uhd::time_spec_t(num_bursts * burst_increment);

        uint64_t samples_sent_this_burst = 0;
        while((burst_length == 0 || samples_sent_this_burst < burst_length) && (total_samples == 0 || total_samples_sent < total_samples) && !stop_signal_called) {
            // Send up to spb samples, without going past the end of the burst or the total
            uint64_t samples_to_send = spb;
            if(burst_length != 0) {
                samples_to_send = std::min(samples_to_send, burst_length - samples_sent_this_burst);
            }
            if(total_samples != 0) {
                samples_to_send = std::min(samples_to_send, total_samples - total_samples_sent);
            }

            // Start where the previous send left off in the wave. Every burst starts at the start of the wave
            const std::complex<short>* buff_ptr = &buff[samples_sent_this_burst % fundamental_period];

            size_t samples_this_send = streamer->send(buff_ptr, samples_to_send, md);
            samples_sent_this_burst += samples_this_send;
            total_samples_sent += samples_this_send;

            // Only the first packet of a burst is the start of burst and has the start time
            if(samples_this_send > 0) {
                md.start_of_burst = false;
                md.has_time_spec = false;
            }
        }

        // Send a mini EOB (end of burst) packet to indicate that the burst is over
        md.start_of_burst = false;
        md.has_time_spec = false;
        md.end_of_burst = true;
        streamer->send("", 0, md);

        num_bursts++;
    }

    std::cout << std::format("TX ch{}: sent {} samples\n", channel, total_samples_sent);
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, wave_type, channel_list, results_directory;
    double rx_sob, tx_sob, rate, freq, tx_gain, rx_gain, wave_freq;
    float ampl;
    uint64_t total_samples;
    uint64_t setpoint;
    double burst_increment;

    // Trigger arguments as entered by the user. Parsed into the variables below
    std::string sma_trigpol_s;
    std::string rx_sma_mode_s, tx_sma_mode_s, rx_ufl_mode_s, tx_ufl_mode_s;
    std::string rx_sma_sel_s, tx_sma_sel_s, rx_ufl_sel_s, tx_ufl_sel_s;
    std::string rx_edge_backoff_s, tx_edge_backoff_s;
    std::string rx_edge_sample_num_s, tx_edge_sample_num_s;
    std::string sma_edge_output_s, rx_ufl_edge_output_s, tx_ufl_edge_output_s;
    std::string tx_gating_s;
    std::string rx_ufl_trigpol_s, tx_ufl_trigpol_s;
    std::string rx_burst_length_s, tx_burst_length_s;

    //setup the program options
    // Shown at the top of --help so the per channel list behaviour doesn't need repeating for every argument
    po::options_description desc(
        "Allowed options\n"
        "Arguments that are a list per channel take comma separated values, one per channel in --channels (i.e. \"edge,level\").\n"
        "Specifying a single value applies it to every channel.");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("rx_sob" , po::value<double >(&rx_sob)->default_value(5.0), "(Seconds) Rx channels will stream after this time. If any trigger is set to input RX will stream after this time AND according to trigger settings.")
        ("tx_sob" , po::value<double >(&tx_sob)->default_value(5.0), "(Seconds) Tx channels will stream after this time. If any trigger is set to input TX will stream after this time AND according to trigger settings.")
        ("rate", po::value<double>(&rate), "rate of incomming/outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz. The device's DSP/RF shifts the samples by this frequency. Not the frequency of the waveform in the samples (see --wave-freq)")
        ("tx_gain", po::value<double>(&tx_gain)->default_value(0), "gain for the tx RF chain")
        ("rx_gain", po::value<double>(&rx_gain)->default_value(0), "gain for the rx RF chain")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("wave_type", po::value<std::string>(&wave_type)->default_value("CONST"), "Baseband waveform type (CONST, SQUARE, RAMP, SINE, SINE_NO_Q, COMB)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "Frequency in Hz of the waveform in the samples sent to the device (before the shift by --freq). Comb spacing for COMB waves. Ignored for CONST waves")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("sma_trigpol", po::value<std::string>(&sma_trigpol_s)->default_value("positive"), "Whether the global trigger is active high or low. Valid values: positive, negative")
        ("rx_ufl_trigpol", po::value<std::string>(&rx_ufl_trigpol_s)->default_value("positive"), "Whether the channel's ufl is active high or low. Valid values: positive, negative")
        ("tx_ufl_trigpol", po::value<std::string>(&tx_ufl_trigpol_s)->default_value("positive"), "Whether the channel's ufl is active high or low. Valid values: positive, negative")
        ("rx_sma_mode", po::value<std::string>(&rx_sma_mode_s)->default_value("edge"), "A list per channel for whether the sma trigger activates/is active on edge or level.")
        ("tx_sma_mode", po::value<std::string>(&tx_sma_mode_s)->default_value("edge"), "A list per channel for whether the sma trigger activates/is active on edge or level.")
        ("rx_ufl_mode", po::value<std::string>(&rx_ufl_mode_s)->default_value("edge"), "A list per channel for whether the ufl trigger activates on edge or level.")
        ("tx_ufl_mode", po::value<std::string>(&tx_ufl_mode_s)->default_value("edge"), "A list per channel for whether the ufl trigger activates on edge or level.")
        ("rx_sma_sel", po::value<std::string>(&rx_sma_sel_s)->default_value("input"), "A list per channel for whether the the channel is inputting or outputing to the trigger. A single channel can only do one of input or output on the sma at a time regardless of whether the device support have separate input and output pins. On units with bidirectional sma pins all channels' sma setting must either match or be none. Valid values: input, output, none")
        ("tx_sma_sel", po::value<std::string>(&tx_sma_sel_s)->default_value("input"), "A list per channel for whether the the channel is inputting or outputing to the trigger. A single channel can only do one of input or output on the sma at a time regardless of whether the device support have separate input and output pins. On units with bidirectional sma pins all channels' sma setting must either match or be none. Valid values: input, output, none")
        ("rx_ufl_sel", po::value<std::string>(&rx_ufl_sel_s)->default_value("none"), "A list per channel for whether the the channel is inputting or outputing to the trigger. Valid values: input, output, none")
        ("tx_ufl_sel", po::value<std::string>(&tx_ufl_sel_s)->default_value("none"), "A list per channel for whether the the channel is inputting or outputing to the trigger. Valid values: input, output, none")
        ("rx_edge_backoff", po::value<std::string>(&rx_edge_backoff_s)->default_value("0"), "A list per channel for how long in ticks edges should be ignored after receiving an edge. Does nothing for triggers in level mode.")
        ("tx_edge_backoff", po::value<std::string>(&tx_edge_backoff_s)->default_value("0"), "A list per channel for how long in ticks edges should be ignored after receiving an edge. Does nothing for triggers in level mode.")
        ("rx_edge_sample_num", po::value<std::string>(&rx_edge_sample_num_s)->default_value("0"), "A list per channel for how long in samples after edges an input trigger is considered active. Streaming only occurs when all input triggers and time commands say to stream.")
        ("tx_edge_sample_num", po::value<std::string>(&tx_edge_sample_num_s)->default_value("0"), "A list per channel for how long in samples after edges an input trigger is considered active. Streaming only occurs when all input triggers and time commands say to stream.")
        ("sma_edge_output", po::value<std::string>(&sma_edge_output_s)->default_value("0"), "How long in samples after a channel starts streaming the output trigger is active in edge mode. Each channel tracks their output pulse separately and the final result is ORed together. 0 stays active forever")
        ("rx_ufl_edge_output", po::value<std::string>(&rx_ufl_edge_output_s)->default_value("0"), "A list per channel for how long in samples after a channel starts streaming the output trigger is active in edge mode. 0 stays active forever.")
        ("tx_ufl_edge_output", po::value<std::string>(&tx_ufl_edge_output_s)->default_value("0"), "A list per channel for how long in samples after a channel starts streaming the output trigger is active in edge mode. 0 stays active forever.")
        ("tx_gating", po::value<std::string>(&tx_gating_s)->default_value("dsp"), "A list per channel for whether to gate at the dsp or output. Gating at the dsp consumes samples when a trigger is received. Gating at the output consumes samples normally but the samples sent to the DAC are set to 0 unless the trigger is active. Applies to input trigger streaming only.")


        ("total_samples", po::value<uint64_t>(&total_samples)->default_value(0), "How many samples in total to send/receive before exiting. 0 sends/receives until Ctrl+C is pressed.")
        // TODO: explain that streaming only occurs when time conditions (rx/tx_sob, burst_length, burst_increment) AND ufl trigger (if input) AND sma trigger (if input) without duplicating the message for each argument
        ("rx_burst_lengths", po::value<std::string>(&rx_burst_length_s)->default_value("0"), "A list per channel of how many samples to receive per rx burst. Set to 0 for continuous. When not using continuous mode samples will only received when all input triggers are active AND the burst command says samples should be received. The actual number of samples received per burst will be much lower.")
        ("tx_burst_lengths", po::value<std::string>(&tx_burst_length_s)->default_value("0"), "A list per channel of how many samples to send per tx burst. Set to 0 for continuous. Recomendation: If using exactly one edge input trigger match tx_edge_sample_num for keep phase consistent between triggers Otherwise use continuous mode to keep interactions between commands and triggers simple.")
        ("burst_increment", po::value<double>(&burst_increment)->default_value(2), "How many seconds between the start of bursts. Does nothing for channels whose burst length is 0 (continuous).")
        ("tx_setpoint", po::value<uint64_t>(&setpoint)->default_value(30000), "Target buffer level when using edge input triggers and dsp gating. Does nothing otherwise. Setting the setpoint to low will result in underflows.")
        ("results_dir", po::value<std::string>(&results_directory)->default_value("results"), "Directory to save results into")
        ("tx_only", "Do not use rx")
        ("rx_only", "Do not use tx")
        ("overwrite", "Overwrite the results files every trigger, instead of creating a new one each time")
    ;
    po::variables_map vm;
    // parse_command_line silently ignores arguments that aren't options, i.e. "–tx_only" where a copy paste turned "--" into an en dash.
    // Accepting no positional arguments makes them an error instead.
    po::positional_options_description no_positional_arguments;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(no_positional_arguments).run(), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << "UHD TX/RX Advanced Trigger" << desc << std::endl;
        std::cout << "Usage notes\n";
        std::cout << "\tKeep rx batches less than " << MAX_RX_BUFFER << " samples to avoid dropping samples while saving\n";
        return ~0;
    }

    bool overwrite = vm.count("overwrite");

    bool use_rx = !vm.count("tx_only");
    bool use_tx = !vm.count("rx_only");
    if(!use_rx && !use_tx) {
        std::cerr << "--tx_only and --rx_only cannot be used together" << std::endl;
        return ~0;
    }

    // Check if user specified the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }

    // Check if the user specified the frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << std::format("Creating the usrp device with: {}...\n", args);
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << std::format("Using Device: {}\n", usrp->get_pp_string());

    //detect which channels to use
    std::vector<size_t> channel_nums;
    for(const std::string& channel_string : split_list(channel_list)) {
        size_t chan = std::stoi(channel_string);
        if((chan >= usrp->get_tx_num_channels() && use_tx) || (chan >= usrp->get_rx_num_channels() && use_rx)) {
            throw std::runtime_error(std::format("Invalid channel {} specified.", chan));
        }
        channel_nums.push_back(chan);
    }
    const size_t num_channels = channel_nums.size();

    // Settings that apply to every channel
    uhd::trigger_pv::polarity_t sma_trigpol = parse_polarity(sma_trigpol_s, "sma_trigpol");
    int64_t sma_edge_output = parse_non_negative(sma_edge_output_s, "sma_edge_output");

    // Per channel trigger settings. Each vector has one element per channel in channel_nums
    std::vector<uhd::trigger_pv::trigger_mode_t> rx_sma_mode = parse_per_channel<uhd::trigger_pv::trigger_mode_t>(rx_sma_mode_s, "rx_sma_mode", num_channels, parse_trigger_mode);
    std::vector<uhd::trigger_pv::trigger_mode_t> tx_sma_mode = parse_per_channel<uhd::trigger_pv::trigger_mode_t>(tx_sma_mode_s, "tx_sma_mode", num_channels, parse_trigger_mode);
    std::vector<uhd::trigger_pv::trigger_mode_t> rx_ufl_mode = parse_per_channel<uhd::trigger_pv::trigger_mode_t>(rx_ufl_mode_s, "rx_ufl_mode", num_channels, parse_trigger_mode);
    std::vector<uhd::trigger_pv::trigger_mode_t> tx_ufl_mode = parse_per_channel<uhd::trigger_pv::trigger_mode_t>(tx_ufl_mode_s, "tx_ufl_mode", num_channels, parse_trigger_mode);

    std::vector<trigger_sel> rx_sma_sel = parse_per_channel<trigger_sel>(rx_sma_sel_s, "rx_sma_sel", num_channels, parse_trigger_sel);
    std::vector<trigger_sel> tx_sma_sel = parse_per_channel<trigger_sel>(tx_sma_sel_s, "tx_sma_sel", num_channels, parse_trigger_sel);
    std::vector<trigger_sel> rx_ufl_sel = parse_per_channel<trigger_sel>(rx_ufl_sel_s, "rx_ufl_sel", num_channels, parse_trigger_sel);
    std::vector<trigger_sel> tx_ufl_sel = parse_per_channel<trigger_sel>(tx_ufl_sel_s, "tx_ufl_sel", num_channels, parse_trigger_sel);

    std::vector<int64_t> rx_edge_backoff = parse_per_channel<int64_t>(rx_edge_backoff_s, "rx_edge_backoff", num_channels, parse_non_negative);
    std::vector<int64_t> tx_edge_backoff = parse_per_channel<int64_t>(tx_edge_backoff_s, "tx_edge_backoff", num_channels, parse_non_negative);
    std::vector<int64_t> rx_edge_sample_num = parse_per_channel<int64_t>(rx_edge_sample_num_s, "rx_edge_sample_num", num_channels, parse_non_negative);
    std::vector<int64_t> tx_edge_sample_num = parse_per_channel<int64_t>(tx_edge_sample_num_s, "tx_edge_sample_num", num_channels, parse_non_negative);
    std::vector<int64_t> rx_ufl_edge_output = parse_per_channel<int64_t>(rx_ufl_edge_output_s, "rx_ufl_edge_output", num_channels, parse_non_negative);
    std::vector<int64_t> tx_ufl_edge_output = parse_per_channel<int64_t>(tx_ufl_edge_output_s, "tx_ufl_edge_output", num_channels, parse_non_negative);

    std::vector<uhd::trigger_pv::polarity_t> rx_ufl_trigpol = parse_per_channel<uhd::trigger_pv::polarity_t>(rx_ufl_trigpol_s, "rx_ufl_trigpol", num_channels, parse_polarity);
    std::vector<uhd::trigger_pv::polarity_t> tx_ufl_trigpol = parse_per_channel<uhd::trigger_pv::polarity_t>(tx_ufl_trigpol_s, "tx_ufl_trigpol", num_channels, parse_polarity);

    std::vector<uhd::trigger_pv::gating_t> tx_gating = parse_per_channel<uhd::trigger_pv::gating_t>(tx_gating_s, "tx_gating", num_channels, parse_gating);

    // Per channel streaming settings
    std::vector<uint64_t> rx_burst_length = parse_per_channel<uint64_t>(rx_burst_length_s, "rx_burst_lengths", num_channels, parse_non_negative);
    std::vector<uint64_t> tx_burst_length = parse_per_channel<uint64_t>(tx_burst_length_s, "tx_burst_lengths", num_channels, parse_non_negative);

    // Rates are applied before frequencies because the frequency tuning depends on the sample rate
    // The actual rates are needed later to generate the tx waveform
    double tx_rate = 0;
    double rx_rate = 0;
    for(size_t ch = 0; ch < num_channels; ch++) {
        if(use_tx) {
            std::cout << std::format("Setting ch{} TX Rate: {} Msps...\n", channel_nums[ch], (rate/1e6));
            usrp->set_tx_rate(rate, channel_nums[ch]);
            tx_rate = usrp->get_tx_rate(channel_nums[ch]);
            std::cout << std::format("Actual ch{} TX Rate: {} Msps...\n\n", channel_nums[ch], (tx_rate/1e6));
        }
        if(use_rx) {
            std::cout << std::format("Setting ch{} RX Rate: {} Msps...\n", channel_nums[ch], (rate/1e6));
            usrp->set_rx_rate(rate, channel_nums[ch]);
            rx_rate = usrp->get_rx_rate(channel_nums[ch]);
            std::cout << std::format("Actual ch{} RX Rate: {} Msps...\n\n", channel_nums[ch], (rx_rate/1e6));
        }
    }

    // Every tx channel shares one sample buffer, so they must all have the same rate
    if(use_tx) {
        double first_tx_rate = usrp->get_tx_rate(channel_nums[0]);
        for(size_t ch = 0; ch < num_channels; ch++) {
            double this_tx_rate = usrp->get_tx_rate(channel_nums[ch]);
            if(this_tx_rate != first_tx_rate) {
                throw std::runtime_error(std::format("TX ch{} has a rate of {} Msps but TX ch{} has a rate of {} Msps. Every tx channel must have the same rate", channel_nums[ch], this_tx_rate/1e6, channel_nums[0], first_tx_rate/1e6));
            }
        }
    }

    // Apply frequencies
    for(size_t ch = 0; ch < num_channels; ch++) {
        if(use_tx) {
            std::cout << std::format("Setting ch{} TX Freq: {} MHz...\n", channel_nums[ch], (freq/1e6));
            uhd::tune_request_t tune_request(freq);
            usrp->set_tx_freq(tune_request, channel_nums[ch]);
            std::cout << std::format("Actual ch{} TX Freq: {} MHz...\n\n", channel_nums[ch], (usrp->get_tx_freq(channel_nums[ch])/1e6));
        }
        if(use_rx) {
            std::cout << std::format("Setting ch{} RX Freq: {} MHz...\n", channel_nums[ch], (freq/1e6));
            uhd::tune_request_t tune_request(freq);
            usrp->set_rx_freq(tune_request, channel_nums[ch]);
            std::cout << std::format("Actual ch{} RX Freq: {} MHz...\n\n", channel_nums[ch], (usrp->get_rx_freq(channel_nums[ch])/1e6));
        }
    }

    // Apply gains
    for(size_t ch = 0; ch < num_channels; ch++) {
        if(use_tx) {
            std::cout << std::format("Setting ch{} TX Gain: {} dB...\n", channel_nums[ch], tx_gain);
            usrp->set_tx_gain(tx_gain, channel_nums[ch]);
            std::cout << std::format("Actual ch{} TX Gain: {} dB...\n\n", channel_nums[ch], usrp->get_tx_gain(channel_nums[ch]));
        }
        if(use_rx) {
            std::cout << std::format("Setting ch{} RX Gain: {} dB...\n", channel_nums[ch], rx_gain);
            usrp->set_rx_gain(rx_gain, channel_nums[ch]);
            std::cout << std::format("Actual ch{} RX Gain: {} dB...\n\n", channel_nums[ch], usrp->get_rx_gain(channel_nums[ch]));
        }
    }

    // Create the tx waveform
    // Note: wave_freq is the frequency of the waveform in the samples sent to the device.
    // freq (set above) is how far the device's DSP/RF shifts those samples. They are unrelated.
    // Every tx channel uses the same rate, wave type, wave frequency and amplitude, so they share one generator.
    std::unique_ptr<wave_generator<short>> tx_wave;
    // How many samples the wave takes to repeat exactly. Each tx thread's buffer is this much longer than it sends at once
    size_t fundamental_period = 0;
    if(use_tx) {
        //error when the waveform is not possible to generate
        if (std::abs(wave_freq) > tx_rate/2){
            throw std::runtime_error("wave freq out of Nyquist zone");
        }
        tx_wave = std::make_unique<wave_generator<short>>(wave_type, ampl, tx_rate, wave_freq);

        // Limit the size of the sample buffer to avoid excessive resource use (same limits as tx_waveforms)
        // Most waves are limited to 2e9 samples (8Gb of RAM)
        // Comb waves are limited to 100e3 samples due to how long lookup table generation takes
        fundamental_period = tx_wave->get_fundamental_period();
        const size_t max_lut_size = (wave_type == "COMB") ? 100000 : 2000000000;
        if(fundamental_period > max_lut_size) {
            std::cout << std::format("The fundamental period of the wave is very large. The lookup table will be limited to {} samples. This will cause a discontinuity every {} seconds.\n", max_lut_size, max_lut_size / tx_rate);
            fundamental_period = max_lut_size;
        }
    }

    // Configure trigger
    // Channels not passed to trigger_configure will be disabled. Every channel used by this program is passed.
    {
        using namespace uhd::trigger_pv;

        trigger_op_t op;
        // Let the device adjust values it can't apply exactly (i.e. edge_sample_num rounded up to a multiple of 2).
        // The adjusted values are written back into the structs and printed below.
        op.enable_strict_errors &= ~strict_error_t::coerced_value;

        trigger_global_t global;
        // derive sets the sma direction from the channels, so the same arguments work on devices with a single
        // bidirectional sma pin and on devices with separate sma input and output pins
        global.sma_direction = pin_direction_t::derive;
        global.sma_polarity = sma_trigpol;
        global.sma_override = 0;
        global.sma_output_pulse_length = sma_edge_output;

        std::vector<trigger_rx_t> rx_trigger;
        if(use_rx) {
            for(size_t ch = 0; ch < num_channels; ch++) {
                trigger_rx_t chan;
                chan.channel = channel_nums[ch];
                chan.trig_sel = make_trig_sel(rx_sma_sel[ch], rx_ufl_sel[ch]);
                chan.sma_mode = rx_sma_mode[ch];
                chan.ufl_mode = rx_ufl_mode[ch];
                chan.ufl_dir = ufl_direction(rx_ufl_sel[ch]);
                chan.ufl_polarity = rx_ufl_trigpol[ch];
                chan.edge_backoff = rx_edge_backoff[ch];
                chan.edge_sample_num = rx_edge_sample_num[ch];
                chan.ufl_output_pulse_length = rx_ufl_edge_output[ch];
                // Everything is configured before streaming starts, so the channel doesn't need to stay enabled
                chan.hot_configure = false;
                // Leave the channel disabled. Creating a streamer enables it
                chan.enable = 0;
                rx_trigger.push_back(chan);
            }
        }

        std::vector<trigger_tx_t> tx_trigger;
        if(use_tx) {
            for(size_t ch = 0; ch < num_channels; ch++) {
                trigger_tx_t chan;
                chan.channel = channel_nums[ch];
                chan.trig_sel = make_trig_sel(tx_sma_sel[ch], tx_ufl_sel[ch]);
                chan.sma_mode = tx_sma_mode[ch];
                chan.ufl_mode = tx_ufl_mode[ch];
                chan.ufl_dir = ufl_direction(tx_ufl_sel[ch]);
                chan.ufl_polarity = tx_ufl_trigpol[ch];
                chan.edge_backoff = tx_edge_backoff[ch];
                chan.edge_sample_num = tx_edge_sample_num[ch];
                chan.ufl_output_pulse_length = tx_ufl_edge_output[ch];
                chan.gating = tx_gating[ch];
                // Everything is configured before streaming starts, so the channel doesn't need to stay enabled
                chan.hot_configure = false;
                // Leave the channel disabled. Creating a streamer enables it
                chan.enable = 0;
                tx_trigger.push_back(chan);
            }
        }

        std::cout << "Configuring trigger...\n";
        bool exact = usrp->trigger_configure(&op, &global, rx_trigger.data(), rx_trigger.size(), tx_trigger.data(), tx_trigger.size());

        // Use the values the device actually applied from here on
        for(size_t ch = 0; ch < rx_trigger.size(); ch++) {
            rx_edge_sample_num[ch] = rx_trigger[ch].edge_sample_num;
        }
        for(size_t ch = 0; ch < tx_trigger.size(); ch++) {
            tx_edge_sample_num[ch] = tx_trigger[ch].edge_sample_num;
        }

        if(!exact) {
            std::cout << "Some trigger settings were adjusted by the device. Actual edge_sample_num values:\n";
            for(size_t ch = 0; ch < rx_trigger.size(); ch++) {
                std::cout << std::format("    RX ch{}: {}\n", channel_nums[ch], rx_edge_sample_num[ch]);
            }
            for(size_t ch = 0; ch < tx_trigger.size(); ch++) {
                std::cout << std::format("    TX ch{}: {}\n", channel_nums[ch], tx_edge_sample_num[ch]);
            }
        }
    }

    // The time on the device as of the end of configuring
    // Streaming start of this + rx_sob or tx_sob
    uhd::time_spec_t init_time = usrp->get_time_now();

    // Create the tx streamers. Each channel gets its own streamer so channels don't have to send the same number of samples
    // Streamers are created here, one at a time, rather than inside the threads, to avoid creating several at once
    // Creating a streamer enables its channel (trigger_configure left them disabled)
    std::vector<uhd::tx_streamer::sptr> tx_streamers;
    std::vector<bool> tx_uses_blocking_fc;
    if(use_tx) {
        for(size_t ch = 0; ch < num_channels; ch++) {
            uhd::stream_args_t tx_stream_args("sc16");
            tx_stream_args.channels = {channel_nums[ch]};
            uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);

            // With dsp gating an input trigger decides when samples are consumed, so blocking flow control is needed
            // to avoid sending samples faster than they are used
            bool any_input_trigger = tx_sma_sel[ch] == trigger_sel::input || tx_ufl_sel[ch] == trigger_sel::input;
            bool use_blocking_fc = any_input_trigger && tx_gating[ch] == uhd::trigger_pv::gating_t::dsp;
            if(use_blocking_fc) {
                // Keep the device's buffer filled to the setpoint
                tx_stream->enable_blocking_fc(setpoint);
            }

            tx_streamers.push_back(tx_stream);
            tx_uses_blocking_fc.push_back(use_blocking_fc);
        }
    }

    // Create the sample buffer shared by every tx thread
    // Each thread sends up to 10 packets worth (spb) at a time, starting anywhere in the first fundamental period.
    // Holding one fundamental period more than the largest spb lets every send continue the wave without a discontinuity (see tx_waveforms)
    tx_buffer_t tx_buff;
    if(use_tx) {
        size_t largest_spb = 0;
        for(uhd::tx_streamer::sptr& tx_stream : tx_streamers) {
            largest_spb = std::max(largest_spb, tx_stream->get_max_num_samps() * 10);
        }

        // Pad the end to a whole number of cache lines
        const size_t samples_per_cache_line = CACHE_LINE_SIZE / sizeof(std::complex<short>);
        size_t buffer_length = largest_spb + fundamental_period;
        buffer_length = ((buffer_length + samples_per_cache_line - 1) / samples_per_cache_line) * samples_per_cache_line;

        tx_buff.resize(buffer_length);
        for (size_t n = 0; n < tx_buff.size(); n++) {
            tx_buff[n] = (*tx_wave)(n);
        }
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    std::vector<std::thread> tx_threads;
    std::vector<std::thread> rx_threads;
    try {
        // Create the tx threads, one per channel
        for(size_t ch = 0; ch < tx_streamers.size(); ch++) {
            tx_threads.emplace_back(tx_function, tx_streamers[ch], channel_nums[ch], std::cref(tx_buff), fundamental_period, init_time + uhd::time_spec_t(tx_sob), burst_increment, tx_burst_length[ch], total_samples);
        }

        // Create the rx streamers. Each channel gets its own streamer to avoid conflicts in the number of samples
        // Streamers are created here, one at a time, rather than inside the threads, to avoid creating several at once
        // Creating a streamer enables its channel (trigger_configure left them disabled)
        std::vector<uhd::rx_streamer::sptr> rx_streamers;
        if(use_rx) {
            std::filesystem::create_directories(results_directory);

            for(size_t ch = 0; ch < num_channels; ch++) {
                uhd::stream_args_t rx_stream_args("sc16");
                rx_stream_args.channels = {channel_nums[ch]};
                rx_streamers.push_back(usrp->get_rx_stream(rx_stream_args));
            }
        }

        // Create the rx threads, one per channel
        // Channels with a burst length of 0 receive continuously, the others receive in bursts
        for(size_t ch = 0; ch < rx_streamers.size(); ch++) {
            uhd::time_spec_t rx_start_time = init_time + uhd::time_spec_t(rx_sob);
            double this_rx_rate = usrp->get_rx_rate(channel_nums[ch]);
            if(rx_burst_length[ch] == 0) {
                rx_threads.emplace_back(rx_continous_function, rx_streamers[ch], channel_nums[ch], rx_start_time, this_rx_rate, rx_burst_length[ch], total_samples, results_directory, overwrite);
            } else {
                rx_threads.emplace_back(rx_burst_function, usrp, rx_streamers[ch], channel_nums[ch], rx_start_time, this_rx_rate, burst_increment, rx_burst_length[ch], total_samples, results_directory, overwrite);
            }
        }
    } catch(...) {
        // Destroying a std::thread that is still running terminates the program ("terminate called without an active exception"),
        // which would hide this error. Stop the threads that were already started and wait for them before passing the error on.
        stop_signal_called = true;
        join_threads(tx_threads);
        join_threads(rx_threads);
        throw;
    }

    // Wait for tx and rx to finish
    join_threads(tx_threads);
    join_threads(rx_threads);

    for(size_t ch = 0; ch < tx_streamers.size(); ch++) {
        if(tx_uses_blocking_fc[ch]) {
            tx_streamers[ch]->disable_blocking_fc();
        }
    }

    // TODO: cleanup trigger


    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
