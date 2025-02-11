//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "wave_generator.hpp"
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
#include <algorithm>
#include <filesystem>
#include <fstream>

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){
    stop_signal_called = true;
}

void tx_function(uhd::usrp::multi_usrp::sptr usrp, const std::vector<size_t> channel_nums, const uint64_t burst_size, const double wave_freq, const double rate, const std::string wave_type, const double ampl, const double first, const double burst_period, const uint64_t burst_count) {

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("sc16");
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // Calculate size of buffer to send
    uint64_t spb = tx_stream->get_max_num_samps()*10;
    spb = std::min(spb, burst_size);

    double period;
    if(wave_freq != 0) {
        period = rate/wave_freq;
    } else {
        period = 0;
    }
    double full_period;
    double frac_period = std::modf(period, &full_period);
    // Length of the period of the sampled signal, to take into account mismatch between period and sample rate
    size_t super_period;

    if(frac_period < 0.00001 || frac_period > 0.99999) {
        super_period = period;
    } else {
        double extra_cycles;
        if(frac_period < 0.5) {
            extra_cycles = 1.0/frac_period;
        } else {
            extra_cycles = 1.0/(1.0-frac_period);
        }

        super_period = (size_t) ::round(period * extra_cycles);
    }

    std::vector<std::complex<short> > buff(spb + super_period);
    std::vector<std::complex<short> *> buffs(channel_nums.size(), &buff.front());

    //fill the buffer with the waveform
    wave_generator<short> wave_generator(wave_type, ampl, rate, wave_freq);
    for (size_t n = 0; n < buff.size(); n++){
        buff[n] = wave_generator(n);
    }

    for(uint64_t bursts_sent = 0; (bursts_sent < burst_count || burst_count == 0) && !stop_signal_called; bursts_sent++) {
        // Set up metadata. We start streaming a bit in the future
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst   = false;
        md.has_time_spec  = true;
        md.time_spec = uhd::time_spec_t(first + (bursts_sent * burst_period));

        // Number of samples sent this burst
        uint64_t num_sent_samples = 0;
        while(!stop_signal_called && num_sent_samples < burst_size){

            // Locates where in the buffer to use samples from
            for(auto& buff_ptr : buffs) {
                if(super_period != 0) {
                    buff_ptr = &buff[num_sent_samples % super_period];
                } else {
                    buff_ptr = &buff.front();
                }
            }

            size_t nsamps_this_send = std::min(spb, burst_size - num_sent_samples);

            //this statement will block until the data is sent
            //send the entire contents of the buffer
            num_sent_samples += tx_stream->send(buffs, nsamps_this_send, md);
            md.start_of_burst = false;
            md.has_time_spec = false;
        }
        //send a mini EOB packet
        md.end_of_burst = true;
        tx_stream->send("", 0, md);
    }
}

void rx_function(uhd::usrp::multi_usrp::sptr usrp, std::string result_folder, const std::vector<size_t> channel_nums, const uint64_t burst_size, const double first, const double burst_period, const uint64_t burst_count) {

    std::filesystem::create_directories(result_folder);

    // Create a receive streamer
    uhd::stream_args_t stream_args("sc16");
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // Buffer to store received data in
    std::vector<std::vector<std::complex<short>>> buffers(channel_nums.size(), std::vector<std::complex<short>>(burst_size, 0));

    // How long until recv times out
    double timeout = first + (burst_period/2);

    for(uint64_t bursts_sent = 0; (bursts_sent < burst_count || burst_count == 0) && !stop_signal_called; bursts_sent++) {
        // Tells device to start streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps  = burst_size;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec  = uhd::time_spec_t(first + (bursts_sent * burst_period));
        rx_stream->issue_stream_cmd(stream_cmd);

        std::vector<std::ofstream> result_files(channel_nums.size());
        for (size_t ch_i = 0; ch_i < channel_nums.size(); ch_i++) {
                std::string filename = result_folder + "/burst_" + std::to_string(bursts_sent) + "_channel_" + std::to_string(channel_nums[ch_i]) + ".dat";
                result_files[ch_i].open(filename.c_str(), std::ofstream::binary);
        }

        // Number of samples received this burst
        uint64_t num_received_samples = 0;
        while(!stop_signal_called && num_received_samples < burst_size){

            // Struct to store metadata related to recv in
            uhd::rx_metadata_t md;

            // Pointers to where to receive data data at to pass to receive function
            std::vector<std::complex<short> *> buffer_pointers(channel_nums.size());
            for(size_t ch_i = 0; ch_i < channel_nums.size(); ch_i++) {
                buffer_pointers[ch_i] = &buffers[ch_i][num_received_samples];
            }

            // Receive data and store it in the buffer
            num_received_samples += rx_stream->recv(buffer_pointers, burst_size - num_received_samples, md, timeout);

            // All later receives will have a shorter time out
            timeout = 2 * burst_period;

            switch(md.error_code) {
                case uhd::rx_metadata_t::ERROR_CODE_NONE:
                    // no error
                    break;

                case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
                    UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "Timeout while receving burst " + std::to_string(bursts_sent) + ". Stopping streaming\n");
                    stop_signal_called = true;
                    break;

                case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
                    UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "Overflow while receving burst " + std::to_string(bursts_sent) + ". The most likely cause is insufficient disk speed. Stopping streaming\n");
                    stop_signal_called = true;
                    break;

                default:
                    UHD_LOG_ERROR("RX_MULTI_RATES_TO_FILE", "recv error: " + md.strerror() + ". Overflow while receving burst " + std::to_string(bursts_sent) + ". The most likely cause is insufficient disk speed. Stopping streaming\n");
                    stop_signal_called = true;
                    break;
            }
        }
        for (size_t ch_i = 0; ch_i < channel_nums.size(); ch_i++) {
            result_files[ch_i].write((const char *) buffers[ch_i].data(), num_received_samples * sizeof(std::complex<short>));
        }
    }
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, wave_type, ref, pps, channel_list, result_folder;
    uint64_t burst_count, burst_size;
    double rate, freq, tx_gain, rx_gain, wave_freq;
    float ampl;

    double first, burst_period;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("folder", po::value<std::string>(&result_folder)->default_value("results"), "name of the file to write binary samples to (sc16)")
        ("burst_size", po::value<uint64_t>(&burst_size)->default_value(10000), "samples per burst")
        ("burst_count", po::value<uint64_t>(&burst_count)->default_value(0), "number of bursts to send. 0 indicates send forever")
        ("first", po::value<double>(&first)->default_value(5), "Time of the first burst")
        ("burst_period", po::value<double>(&burst_period)->default_value(1), "Time between the start of each bursts")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq)->default_value(0), "RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 1]")
        ("tx_gain", po::value<double>(&tx_gain)->default_value(0), "gain for the TX RF chain")
        ("rx_gain", po::value<double>(&rx_gain)->default_value(0), "gain for the RX RF chain")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("SINE"), "waveform type (CONST, SQUARE, RAMP, SINE, SINE_NO_Q)")
        //SIN_NO_Q can also be used to generate a sinwave without the q component, which is useful when debugging the FPGA
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("ref", po::value<std::string>(&ref), "clock reference (internal, external, mimo, gpsdo)")
        ("pps", po::value<std::string>(&pps)->default_value("internal"), "PPS source (internal, external, mimo, gpsdo, bypass)")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
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
    usrp->set_clock_source(ref);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;

    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;


    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tx_tune_request(freq);
        usrp->set_tx_freq(tx_tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t rx_tune_request(freq);
        usrp->set_rx_freq(rx_tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        //set the rf gain
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
        usrp->set_tx_gain(tx_gain, channel_nums[ch]);
        std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;

        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
        usrp->set_rx_gain(rx_gain, channel_nums[ch]);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain(channel_nums[ch]) << std::endl << std::endl;
    }


    //for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST"){
        wave_freq = usrp->get_tx_rate()/2;
    }

    //error when the waveform is not possible to generate
    if (std::abs(wave_freq) > usrp->get_tx_rate()/2){
        throw std::runtime_error("wave freq out of Nyquist zone");
    }

    //Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    const size_t tx_sensor_chan = channel_nums.empty() ? 0 : channel_nums[0];
    sensor_names = usrp->get_tx_sensor_names(tx_sensor_chan);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", tx_sensor_chan);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    const size_t mboard_sensor_idx = 0;
    sensor_names = usrp->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    if(pps != "bypass") {
        if (channel_nums.size() > 1)
        {
            // Sync times
            if (pps == "mimo")
            {
                UHD_ASSERT_THROW(usrp->get_num_mboards() == 2);

                //make mboard 1 a slave over the MIMO Cable
                usrp->set_time_source("mimo", 1);

                //set time on the master (mboard 0)
                usrp->set_time_now(uhd::time_spec_t(0.0), 0);

                //sleep a bit while the slave locks its time to the master
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            else
            {
                if (pps == "internal" or pps == "external" or pps == "gpsdo")
                    usrp->set_time_source(pps);
                usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
                std::this_thread::sleep_for(std::chrono::seconds(1)); //wait for pps sync pulse
            }
        }
        else
        {
            usrp->set_time_now(0.0);
        }
    } else {
        std::cout << "Bypassing setting clock, this may interfere with start time" << std::endl;
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    std::thread tx_thread(tx_function, usrp, channel_nums, burst_size, wave_freq, rate, wave_type, ampl, first, burst_period, burst_count);

    std::thread rx_thread(rx_function, usrp, result_folder, channel_nums, burst_size, first, burst_period, burst_count);

    tx_thread.join();
    rx_thread.join();

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
