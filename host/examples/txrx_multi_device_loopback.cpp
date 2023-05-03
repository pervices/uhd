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
#include <boost/filesystem.hpp>
#include "wavetable.hpp"
#include <stdint.h>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <thread>
#include <vector>

namespace po = boost::program_options;

// Maximum size of the receive buffer, mainly to avoid using to much RAM
const size_t max_receive_buffer_size = 100000000;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

// Contains the parameters for each individual connected device
struct device_parameters {
    std::string args;
    size_t num_tx_channels;
    size_t num_rx_channels;
    std::vector<size_t> tx_channels, rx_channels;
    std::vector<double> tx_gains, rx_gains, tx_freqs, rx_freqs, amplitude, wave_freq;
    std::string time_reference;
    std::string clock_reference;

};

/**
 * Receives and saves rx data
 * @param rx_stream The streamer to receive data from
 * @param output_folder The folder to save data to
 * @param device_number The index of the device in the argument list. Used to figure out the path to save data to
 * @param start_time The time in seconds to start streaming at
 * @param requested_num_samps The number of samples to receive
 * @param rate The sample rate
 * @param save_rx set to false to disable saving rx data
 */
void rx_run(uhd::rx_streamer* rx_stream, std::string output_folder, size_t device_number, std::vector<size_t> rx_channels, double start_time, size_t requested_num_samps, double rate, bool save_rx) {
    size_t receive_buffer_size = std::min(requested_num_samps, max_receive_buffer_size);
    std::vector<std::vector<std::complex<short>>> buffers(rx_channels.size(), std::vector<std::complex<short>>(receive_buffer_size));

    std::vector<std::complex<short>*> buffer_ptrs(rx_channels.size());

    for(size_t n = 0; n < rx_channels.size(); n++) {
            buffer_ptrs[n] = &buffers[n].front();
    }

    std::vector<std::ofstream> outfiles(rx_channels.size());
    if (save_rx) {
        for(size_t n = 0; n < rx_channels.size(); n++) {
            std::string path = output_folder + "/rx_" + std::to_string(device_number) + "_ch_" + std::to_string(rx_channels[n]) + ".dat";
            outfiles[n].open(path.c_str(), std::ofstream::binary);
        }
    }

    // Metadata return struct
    uhd::rx_metadata_t md;

    // Configure stream command
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(requested_num_samps);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(start_time);

    rx_stream->issue_stream_cmd(stream_cmd);

    size_t total_rx_samps = 0;
    while(total_rx_samps < requested_num_samps && !stop_signal_called) {
        // Receives rx data
        size_t nsamps_to_recv = std::min(requested_num_samps - total_rx_samps, receive_buffer_size);
        size_t num_rx_samps = rx_stream->recv(buffer_ptrs, nsamps_to_recv, md, start_time + (requested_num_samps/rate) + 3, false);

        //TODO add error checking

        // Saves rx data
        if(save_rx) {
            for(size_t n = 0; n < rx_channels.size(); n++) {
                outfiles[n].write((const char*)&buffers[n].front(), num_rx_samps * sizeof(std::complex<short>));
            }
        }
        total_rx_samps += num_rx_samps;
    }

    printf("Received %lu samples of %lu requested from device %lu\n", total_rx_samps, requested_num_samps, device_number);
}

/**
 * Send tx data
 * @param tx_stream The streamer to receive data from
 * @param parameters Struct containing various paramters. The relevant ones for this wave wave_freq and ampl
 * @param start_time The time in seconds to start streaming at
 * @param requested_num_samps The number of samples to receive
 */
void tx_run( uhd::tx_streamer* tx_stream, device_parameters* parameters, size_t device_number, double start_time, size_t requested_num_samps, double rate) {
    size_t spb = tx_stream->get_max_num_samps()*10;

    // Buffer contains samples for each channel
    std::vector<std::vector<std::complex<short>>> buffer(parameters->num_tx_channels, std::vector<std::complex<short>>(spb));
    // Vector containing pointers to the start of the sample buffer for each channel
    std::vector<std::complex<short> *> buffer_ptrs(parameters->num_tx_channels);

    for(size_t n = 0; n < parameters->num_tx_channels; n++) {
        buffer_ptrs[n] = &buffer[n].front();
    }

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(start_time);
    double timeout = start_time + (requested_num_samps/rate) + 3;
    size_t total_samples_sent = 0;

    size_t wavetable_index = 0;
    //pre-compute the waveform values
    std::vector<wave_table_class> wave_tables;
    std::vector<size_t> steps;
    for(size_t ch = 0; ch <parameters->num_tx_channels; ch++) {
        wave_tables.push_back(wave_table_class("SINE", parameters->amplitude[ch]));
        steps.push_back((size_t) ::round(parameters->wave_freq[ch]/rate * wave_table_len));
    }

    while(total_samples_sent < requested_num_samps && !stop_signal_called) {

        size_t samples_to_send = std::min(requested_num_samps - total_samples_sent, spb);

        for(size_t ch = 0; ch < parameters->num_tx_channels; ch++) {
            for(size_t n = 0; n < samples_to_send; n++) {
                buffer[ch][n] = wave_tables[ch](wavetable_index);
            }
            wavetable_index+=steps[ch];
        }

        // Currently will send starting at the begining of the buffer each time
        total_samples_sent+=tx_stream->send(buffer_ptrs, samples_to_send, md, 20);

        // Indicate packets are no longer at the start
        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    md.end_of_burst = true;
    tx_stream->send("", 0, md);

    printf("Sent %lu samples of %lu attempted to device %lu\n", total_samples_sent, requested_num_samps, device_number);
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

std::vector<device_parameters> parse_device_parameters(std::string args, std::string time_ref_s, std::string clock_ref_s,  std::string tx_channels_s, std::string rx_channels_s, std::string tx_gains_s, std::string rx_gains_s, std::string tx_freqs_s, std::string rx_freqs_s, std::string ampl_s, std::string wave_freq_s) {
    std::vector<std::string> device_args = seperate_device_argument(args);
    size_t num_devices = device_args.size();

    std::vector<std::string> device_time_ref = seperate_device_argument(time_ref_s);
    if(!validate_number_of_device_arguments(num_devices, device_time_ref)) {
        throw std::runtime_error("Incorrect number of devices a time reference was specified for");
    }

    std::vector<std::string> device_clock_ref = seperate_device_argument(clock_ref_s);
    if(!validate_number_of_device_arguments(num_devices, device_clock_ref)) {
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

    std::vector<std::string> device_ampl_arg = seperate_device_argument(ampl_s);
    if(!validate_number_of_device_arguments(num_devices, device_ampl_arg)) {
        throw std::runtime_error("Incorrect number of devices amplitude were specified for");
    }

    std::vector<std::string> device_wave_freq_arg = seperate_device_argument(wave_freq_s);
    if(!validate_number_of_device_arguments(num_devices, device_wave_freq_arg)) {
        throw std::runtime_error("Incorrect number of devices wave frequencies were specified for");
    }

    std::vector<device_parameters> parameters(num_devices);
    for(size_t n = 0; n < num_devices; n++) {
        parameters[n].args = device_args[n];
        parameters[n].time_reference = device_time_ref[n];
        parameters[n].clock_reference = device_clock_ref[n];

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
                throw std::runtime_error("Tx gains unspecified for a device with tx channels");
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
                throw std::runtime_error("Rx gains unspecified for a device with rx channels");
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
                throw std::runtime_error("Tx frequencies unspecified for a device with tx channels");
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
                throw std::runtime_error("Rx frequencies unspecified for a device with rx channels");
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

        if(device_ampl_arg[n] == "n") {
            if(parameters[n].num_tx_channels != 0) {
                throw std::runtime_error("Tx amplitude unspecified, even though tx is being used");
            }
        } else {
            std::vector<std::string> ampl_arg_s = seperate_channel_argument(device_ampl_arg[n]);
            if(!validate_number_of_channel_arguments(parameters[n].num_tx_channels, ampl_arg_s)) {
                throw std::runtime_error("Mistmatch between number of tx channels specified and number of tx amplitudes for a device");
            } else {
                if(ampl_arg_s.size()==1) {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].amplitude.push_back(std::stod(ampl_arg_s[0]));
                    }
                }
                else {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].amplitude.push_back(std::stod(ampl_arg_s[i]));
                    }
                }
            }
        }

        if(device_wave_freq_arg[n] == "n") {
            if(parameters[n].num_tx_channels != 0) {
                throw std::runtime_error("Tx wave frequencies unspecified, even though tx is being used");
            }
        } else {
            std::vector<std::string> wave_freq_arg_s = seperate_channel_argument(device_wave_freq_arg[n]);
            if(!validate_number_of_channel_arguments(parameters[n].num_tx_channels, wave_freq_arg_s)) {
                throw std::runtime_error("Mistmatch between number of tx channels specified and number of tx amplitudes for a device");
            } else {
                if(wave_freq_arg_s.size()==1) {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].wave_freq.push_back(std::stod(wave_freq_arg_s[0]));
                    }
                }
                else {
                    for(size_t i = 0; i < parameters[n].num_tx_channels; i++) {
                        parameters[n].wave_freq.push_back(std::stod(wave_freq_arg_s[i]));
                    }
                }
            }
        }
    }
    return parameters;
}

/**
 * Configures the device
 * @param device The device to be configured
 * @param rate Pointer to samples rate, will be modified to what the actual rate is
 * @param parameters A struct containing various settings for the device
 */
void configure_device(uhd::usrp::multi_usrp* device, double& rate, device_parameters& parameters) {
    printf("Configuring device with args: %s\n", parameters.args.c_str());

    device->set_time_source(parameters.time_reference);

    device->set_clock_source(parameters.clock_reference);

    device->set_tx_rate(rate);
    double actual_tx_rate = device->get_tx_rate();
    if(std::abs(actual_tx_rate - rate) > 1) {
        printf("Desired tx rate: %lfMsps, actual tx rate: %lfMsps\n", rate/1e6, actual_tx_rate/1e6);
    }
    device->set_rx_rate(actual_tx_rate);
    double actual_rx_rate = device->get_rx_rate();
    if(std::abs(actual_rx_rate - rate) > 1) {
        printf("Desired rx rate: %lfMsps, actual rx rate: %lfMsps\n", actual_tx_rate/1e6, actual_rx_rate/1e6);
    }
    if(actual_tx_rate != actual_rx_rate) {
        fprintf(stderr, "Mistmatch between tx rate: %lfMsps, rx rate: %lfMsps\n The rest of the program will use tx rate\n", actual_tx_rate/1e6, actual_rx_rate/1e6);
    }
    rate = actual_tx_rate;

    for(size_t n = 0; n < parameters.num_tx_channels; n++) {
        // Setting tx gain
        device->set_tx_gain(parameters.tx_gains[n], parameters.tx_channels[n]);
        double actual_tx_gain = device->get_tx_gain(parameters.tx_channels[n]);
        if(std::abs(actual_tx_gain - parameters.tx_gains[n]) > 0.1) {
            fprintf(stderr, "Unable to set tx gain on ch %lu. Actual gain: %lfdB, desired gain: %lfdB\n", parameters.tx_channels[n], actual_tx_gain, parameters.tx_gains[n]);
        }

        // Setting tx freq
        device->set_tx_freq(parameters.tx_channels[n], parameters.tx_freqs[n]);
        double actual_tx_freq = device->get_tx_freq(parameters.tx_channels[n]);
        if(std::abs(actual_tx_freq - parameters.tx_freqs[n]) > 1) {
            fprintf(stderr, "Unable to set tx frequency on ch %lu. Actual frequency: %lfMHz, desired frequency: %lfMHz\n", parameters.tx_channels[n], actual_tx_freq, parameters.tx_freqs[n]);
        }
    }

    for(size_t n = 0; n < parameters.num_rx_channels; n++) {
        // Setting rx gain
        device->set_rx_gain(parameters.rx_gains[n], parameters.rx_channels[n]);
        double actual_rx_gain = device->get_rx_gain(parameters.rx_channels[n]);
        if(std::abs(actual_rx_gain - parameters.rx_gains[n]) > 0.1) {
            fprintf(stderr, "Unable to set rx gain on ch %lu. Actual gain: %lfdB, desired gain: %lfdB\n", parameters.rx_channels[n], actual_rx_gain, parameters.rx_gains[n]);
        }

        // Setting rx freq
        device->set_rx_freq(parameters.rx_freqs[n], parameters.rx_channels[n]);
        double actual_rx_freq = device->get_rx_freq(parameters.rx_channels[n]);
        if(std::abs(actual_rx_freq - parameters.rx_freqs[n]) > 1) {
            fprintf(stderr, "Unable to set rx frequency on ch %lu. Actual frequency: %lfMHz, desired frequency: %lfMHz\n", parameters.rx_channels[n], actual_rx_freq, parameters.rx_freqs[n]);
        }
    }
}

/**
 * Sets all device times to 0, synchronized to pps
 * @param devices The devices to be synced
 */
void sync_devices(std::vector<uhd::usrp::multi_usrp::sptr> devices) {
    printf("Starting device synchronization\n");
    // Sets the time on device 0 to be 0. Function shoudl return immediatly after a pps
    devices[0]->set_time_unknown_pps(uhd::time_spec_t(0.0));

    // Sets the time on all other devices to be 0
    // Must be done with 1s of set_time_unknown_pps finishing
    uhd::time_spec_t pps_time = devices[0]->get_time_last_pps();
    for(size_t n = 1; n < devices.size(); n++) {
        devices[n]->set_time_next_pps(pps_time);
    }

    // Waits for next pps
    auto timeout_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(1100);
    uhd::time_spec_t latest_pps_time;
    do {
        if (std::chrono::steady_clock::now() > timeout_time) {
            throw uhd::runtime_error("Device 0 no PPS detected\n");
        }
        latest_pps_time = devices[0]->get_time_last_pps();
    } while (pps_time == latest_pps_time); {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Verifies that all clocks are synced to the same pps. Must be completed within 1s of the wait for the previous pps finishing
    for(size_t n = 1; n < devices.size(); n++) {
        uhd::time_spec_t other_device_pps_time = devices[n]->get_time_last_pps();
        if(latest_pps_time != other_device_pps_time) {
            throw uhd::runtime_error("Desync between devices 0 and " + std::to_string(n) + "\n");
        }
    }
    printf("All devices synchronized\n");
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, time_ref, clock_ref, rx_channel_arg, tx_channel_arg, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg, rx_folder, ampl_arg, wave_freq_arg;
    double start_time;
    size_t total_num_samps;
    double rate;
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "Transmits a sinewave on multiple channels on multiple devices and receive data from multiple channels. Arguments with device specific parameter are sperated by spaces between devices and seperated by commas within a device. \"n\" indicates don't use this parameter for this device. Only specifying for one channel on a device indicates use the argument is for all channels on that device")
        ("args", po::value<std::string>(&args)->default_value("192.168.10.2"), "Arguments for selecting devices. Provide device specific parameters")
        ("start_time", po::value<double>(&start_time)->default_value(5), "number of seconds in the future to begin receiving. (From when iniialization is complete, not from when the program is called")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming (Rx) and outgoing (Tx) samples")
        ("rx_channels", po::value<std::string>(&rx_channel_arg)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc). Provide device specific parameters")
        ("tx_channels", po::value<std::string>(&tx_channel_arg)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc). Provide device specific parameters")
        ("tx_gain", po::value<std::string>(&tx_gain_arg)->default_value("0"), "gain for the Tx RF chain. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("rx_gain", po::value<std::string>(&rx_gain_arg)->default_value("0"), "gain for the Rx RF chain. Enter one number to set all the channels to said gain i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("tx_freq", po::value<std::string>(&tx_freq_arg)->default_value("0"), "RF center frequency in Hz. Enter one number to set all the tx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("rx_freq", po::value<std::string>(&rx_freq_arg)->default_value("0"), "RF center frequency in Hz. Enter one number to set all the rx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("rx_folder", po::value<std::string>(&rx_folder)->default_value("output"), "Folder to store rx data. Files will be saved as rx_<device number>_ch_<channel_number>.dat")
        ("no_save", "Do not save rx data")
        ("rx_only", "Do not use rx")
        ("tx_only", "Do not use tx")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(100000), "Numer of samples to send/receive")
        ("time_ref", po::value<std::string>(&time_ref)->default_value("internal"), "Whether to use an internal or external time (PPS) reference (internal, external)")
        ("clock_ref", po::value<std::string>(&clock_ref)->default_value("internal"), "Whether to use an internal or external clock reference (internal, external)")

        ("ampl", po::value<std::string>(&ampl_arg)->default_value("0.7"), "Amplitude of the wave in tx samples. B Enter one number to set all the tx channels to said amplitude i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
        ("wave_freq", po::value<std::string>(&wave_freq_arg)->default_value("0"), "Amplitude of the wave in tx samples. Enter one number to set all the rx channels to said freq i.e. \"0\", enter comma seperated number to set each channel individually i.e. \"0,1\". Provide device specific parameters")
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

    std::vector<device_parameters> parameters = parse_device_parameters(args, time_ref, clock_ref, tx_channel_arg, rx_channel_arg, tx_gain_arg, rx_gain_arg, tx_freq_arg, rx_freq_arg, ampl_arg, wave_freq_arg);

    std::vector<uhd::usrp::multi_usrp::sptr> devices(parameters.size());
    // Connects to each device
    // TODO: parallelize connecting to devices
    for(size_t n = 0; n < devices.size(); n++) {
        std::cout << boost::format("Creating the usrp device with: %s...") % parameters[n].args << std::endl;
        devices[n] = uhd::usrp::multi_usrp::make(parameters[n].args);
    }

    for(size_t n = 0; n < devices.size(); n++) {
        configure_device(devices[n].get(), rate, parameters[n]);
    }

    // Create rx streamers
    std::vector<uhd::rx_streamer::sptr> rx_streamers;
    for(size_t n = 0; n < devices.size(); n++) {
        // create receive streamers
        uhd::stream_args_t rx_stream_args("sc16"); //short complex
        rx_stream_args.channels = parameters[n].rx_channels;
        rx_streamers.push_back(devices[n]->get_rx_stream(rx_stream_args));
    }

    // Create tx streamers
    std::vector<uhd::tx_streamer::sptr> tx_streamers;
    for(size_t n = 0; n < devices.size(); n++) {
        // create receive streamers
        uhd::stream_args_t tx_stream_args("sc16"); //short complex
        tx_stream_args.channels = parameters[n].rx_channels;
        tx_streamers.push_back(devices[n]->get_tx_stream(tx_stream_args));
    }

    sync_devices(devices);


    std::vector<std::thread> rx_threads;
    bool save_rx = !vm.count("no_save");

    if(save_rx) {
        // This function is in the standard library of c++17
        boost::filesystem::create_directories(rx_folder);
    }

    for(size_t n = 0; n < devices.size(); n++) {
        rx_threads.push_back(std::thread(rx_run, rx_streamers[n].get(), rx_folder, n, parameters[n].rx_channels, start_time, total_num_samps, rate ,save_rx));
    }

    std::vector<std::thread> tx_threads;
    for(size_t n = 0; n < devices.size(); n++) {
        tx_threads.push_back(std::thread(tx_run, tx_streamers[n].get(), &parameters[n], n, start_time, total_num_samps, rate));
    }

    printf("\nPress Ctrl + C to stop streaming...\n");

    //Waits for rx to finish
    for(size_t n = 0; n < devices.size(); n++) {
        rx_threads[n].join();
    }

    //Waits for tx to finish
    for(size_t n = 0; n < devices.size(); n++) {
        tx_threads[n].join();
    }
    
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
