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

// Performas send for this channel
tx_function(size_t channel) {

    tx_streamer streamer =...

    if(any trigger is input AND gating is set to dsp) {
       streamer->enable_blocking_fc(/* TODO: copy setpoint from txrx_trigger_loopback */)
    }

    Create buff for samples with a length of streamer->get_max_num_samps() * 10 + fundamental_period (see tx_waveforms)


    while(total_samples_sent < total_samples) {
        Fill out tx metadata to start of burst enabled and time set to init_time + tx_sob + (num_bursts * burst_increment)
        while((samples_sent_this_trigger < burst_length || burst_length == 0) && total_samples_sent < total_samples) {
            samples_this_send = streamer->send

            if(samples_this_send > 0) {
                set sob and has timestamp flags to false
            }
        }
        Send end of burst

        num_bursts++;
    }

    Print total_samples_sent
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
    uint64_t num_trigger;

    // Trigger arguments as entered by the user. Parsed into the variables below
    std::string sma_trigpol_s;
    std::string rx_sma_mode_s, tx_sma_mode_s, rx_ufl_mode_s, tx_ufl_mode_s;
    std::string rx_sma_sel_s, tx_sma_sel_s, rx_ufl_sel_s, tx_ufl_sel_s;
    std::string rx_edge_backoff_s, tx_edge_backoff_s;
    std::string rx_edge_sample_num_s, tx_edge_sample_num_s;
    std::string sma_edge_output_s, rx_ufl_edge_output_s, tx_ufl_edge_output_s;
    std::string tx_gating_s;
    std::string rx_ufl_trigpol_s, tx_ufl_trigpol_s;

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


        ("total_samples", po::value<uint64_t>(&total_samples)->default_value(0), "How many samples in total to send/receive before exiting.")
        // TODO: explain that streaming only occurs when time conditions (rx/tx_sob, burst_length, burst_increment) AND ufl trigger (if input) AND sma trigger (if input) without duplicating the message for each argument
        ("burst_length", po::value<uint64_t>(&burst_length)->default_value(0), "How many samples to send per tx burst, or receive per rx burst. Set to 0 for continuous. Recomended to use 0 for continuous if using input triggers.")
        ("burst_increment", po::value<double>(&burst_increment)->default_value(2), "How many seconds between the start of bursts. Does nothing if burst_length is 0 (continuous).")
        ("results_dir", po::value<std::string>(&results_directory)->default_value("results"), "Directory to save results into")
        ("tx_only", "Do not use rx")
        ("rx_only", "Do not use tx")
        ("overwrite", "Overwrite the results files every trigger, instead of creating a new one each time")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << "UHD TX/RX Advanced Trigger" << desc << std::endl;
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
    if(use_tx) {
        //error when the waveform is not possible to generate
        if (std::abs(wave_freq) > tx_rate/2){
            throw std::runtime_error("wave freq out of Nyquist zone");
        }
        tx_wave = std::make_unique<wave_generator<short>>(wave_type, ampl, tx_rate, wave_freq);
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

    // TODO: Create rx and tx threads. Give each channel it's own streamer to avoid conflicts in the number of samples

    // TODO: cleanup trigger


    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
