//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "wavetable.hpp"
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

static std::atomic<bool> stop_signal_called(false);
void sig_int_handler(int){
#ifdef DEBUG_TX_WAVE
    std::cout << "stop_signal_called" << std::endl;
#endif
    stop_signal_called = true;
}

void tx_run(uhd::tx_streamer::sptr tx_stream, std::vector<std::complex<float> *> buffs, double start_time, uint64_t num_pulses, size_t samples_per_pulse) {
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(start_time);

    for(uint64_t pulses_sent = 0; ((pulses_sent < num_pulses || num_pulses == 0) && !stop_signal_called); pulses_sent++)
    {
        //this statement will block until the data is sent
        //send the entire contents of the buffer
        tx_stream->send(buffs, samples_per_pulse, md);

        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    //send a mini EOB (end of burst) packet to indicate that streaming is over
    md.end_of_burst = true;
    tx_stream->send("", 0, md);
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, wave_type, channel_list;
    size_t samples_per_pulse;
    uint64_t num_pulses, setpoint;
    double rate, freq, tx_gain, rx_gain, wave_freq, start_time;
    float ampl;

    bool use_rx, use_tx;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("start_time" , po::value<double >(&start_time)->default_value(5.0), "(Seconds) Transmitter will enable after this many seconds")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("tx_gain", po::value<double>(&tx_gain), "gain for the tx RF chain")
        ("rx_gain", po::value<double>(&rx_gain), "gain for the rx RF chain")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("CONST"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        //SIN_NO_Q can also be used to generate a sinwave without the q component, whichi s useful when debugging the FPGA
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("num_pulses", po::value<uint64_t>(&num_pulses)->default_value(0), "Number of pulses to send. Set to 0 for continuous")
        ("samples_per_pulse", po::value<size_t>(&samples_per_pulse), "Number of samples to send per pulse")
        ("setpoint", po::value<uint64_t>(&setpoint), "Target buffer level")
        ("tx_only", "Do not use rx")
        ("rx_only", "Do not use tx")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
    }

    use_rx = !vm.count("tx_only");
    use_tx = !vm.count("rx_only");

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

    // Check if the user specified the pulse length
    if (not vm.count("samples_per_pulse")){
        std::cerr << "Please specify the number of samples per pulse with --samples_per_pulse" << std::endl;
        return ~0;
    }

    if(not vm.count("setpoint")) {
        setpoint = 10 * samples_per_pulse;
    }

    if (samples_per_pulse > setpoint) {
        std::cerr << "Setpoint must be greater than samples_per_pulse" << std::endl;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = std::stoi(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels())
            throw std::runtime_error("Invalid channel(s) specified.");
        else
            channel_nums.push_back(std::stoi(channel_strings[ch]));
    }

    //Lock mboard clocks
    usrp->set_clock_source("internal");

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    if(use_tx) {
        std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
        usrp->set_tx_rate(rate);
        //Adjust the requested rate to match the desired rate
        rate = usrp->get_tx_rate();
        std::cout << boost::format("Actual TX Rate: %f Msps...") % (rate/1e6) << std::endl << std::endl;
    }
    if(use_rx) {
        std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
        usrp->set_rx_rate(rate);
        std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;
    }

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        if(use_tx) {
            std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
            uhd::tune_request_t tune_request(freq);
            usrp->set_tx_freq(tune_request, channel_nums[ch]);
            std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;
        }
        if(use_rx) {
            std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
            uhd::tune_request_t tune_request(freq);
            usrp->set_rx_freq(tune_request, channel_nums[ch]);
            std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;
        }


        //set the rf gain
        if(use_tx) {
            if (vm.count("tx_gain")){
                std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
                usrp->set_tx_gain(tx_gain, channel_nums[ch]);
                std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;
            }
        }
        //set the rf gain
        if(use_rx) {
            if (vm.count("rx_gain")){
                std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
                usrp->set_rx_gain(rx_gain, channel_nums[ch]);
                std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain(channel_nums[ch]) << std::endl << std::endl;
            }
        }
    }

    //for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST"){
        wave_freq = usrp->get_tx_rate()/2;
    }

    //error when the waveform is not possible to generate
    if (std::abs(wave_freq) > usrp->get_tx_rate()/2){
        throw std::runtime_error("wave freq out of Nyquist zone");
    }
    if (usrp->get_tx_rate()/std::abs(wave_freq) > wave_table_len/2){
        throw std::runtime_error("wave freq too small for table");
    }

    //pre-compute the waveform values
    const wave_table_class wave_table(wave_type, ampl);
    const size_t step = boost::math::iround(wave_freq/rate * wave_table_len);
    size_t index = 0;

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32");
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    std::vector<std::complex<float> > buff(samples_per_pulse);
    std::vector<std::complex<float> *> buffs(channel_nums.size(), &buff.front());

    //Fill up the buffer. The same data should be sent for every pulse
    for (size_t n = 0; n < buff.size(); n++) {
        buff[n] = wave_table(index += step);
    }

    std::signal(SIGINT, &sig_int_handler);
    usrp->tx_trigger_setup(channel_nums, setpoint, samples_per_pulse);

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(0.0);

    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    std::thread tx_thread (tx_run, tx_stream, buffs, start_time, num_pulses, samples_per_pulse);

    // Wait for tx to finish
    tx_thread.join();
    usrp->tx_trigger_cleanup(channel_nums);

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
