//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
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
#include <random>

//wait for user to press cntrl c before closing
//#define DELAYED_EXIT

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){
#ifdef DEBUG_TX_WAVE
    std::cout << "stop_signal_called" << std::endl;
#endif
    stop_signal_called = true;
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, wave_type, ant, subdev, ref, pps, otw, channel_list;
    uint64_t total_num_samps;
    size_t spb;
    double rate, freq, gain, wave_freq, bw;
    float ampl;

    double first, last, increment, comb_spacing;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("nsamps", po::value<uint64_t>(&total_num_samps)->default_value(0), "total number of samples to transmit")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("SINE"), "waveform type (CONST, SQUARE, RAMP, SINE, SINE_NO_Q, COMB)")
        //SIN_NO_Q can also be used to generate a sinwave without the q component, which is useful when debugging the FPGA
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("comb-spacing", po::value<double>(&comb_spacing), "Comb spacing in Hz. Only used with wave-type=COMB")
        ("ref", po::value<std::string>(&ref), "clock reference (internal, external, mimo, gpsdo)")
        ("pps", po::value<std::string>(&pps)->default_value("internal"), "PPS source (internal, external, mimo, gpsdo, bypass)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("int-n", "tune USRP with integer-N tuning")
        ("first", po::value<double>(&first)->default_value(5), "Time for first stacked command")
        ("last", po::value<double>(&last), "Time for last stacked command")
        ("increment", po::value<double>(&increment)->default_value(1), "Increment for stack commands between <first> and <last> times")
        ("constant_time", "When set, device time gets set to 0, and first and last's exact values are used. Otherwise first and last are relative to the time when initialization finished. In both cases the device time is set to 0 during init unless pps is bypassed")
        ("random-spb", "Intended for internal debuging only. Randomize the number of samples sent per buffer to be 0:spb")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
    }

    if(!vm["wave-freq"].defaulted() && wave_type == "COMB") {
        UHD_LOGGER_WARNING("TX_WAVEFORMS") << "wave-freq specified in comb mode. It will be ignored";
    }

    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

    if(!vm.count("comb-spacing") && wave_type == "COMB") {
        UHD_LOGGER_ERROR("TX_WAVEFORMS") << "COMB wave requested but no comb-spacing specified";
        return ~0;
    }

    bool use_constant_time = vm.count("constant_time");

    bool random_spb = vm.count("random-spb");

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_tx_subdev_spec(subdev);

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
    if(vm.count("ref")) {
        usrp->set_clock_source(ref);
    }

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    double actual_rate = usrp->get_tx_rate();
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (actual_rate/1e6) << std::endl << std::endl;

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_tx_freq(tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        //set the rf gain
        if (vm.count("gain")){
            std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
            usrp->set_tx_gain(gain, channel_nums[ch]);
            std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;
        }

        //set the analog frontend filter bandwidth
        if (vm.count("bw")){
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % bw << std::endl;
            usrp->set_tx_bandwidth(bw, channel_nums[ch]);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...") % usrp->get_tx_bandwidth(channel_nums[ch]) << std::endl << std::endl;
        }

        //set the antenna
        if (vm.count("ant")) usrp->set_tx_antenna(ant, channel_nums[ch]);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1)); //allow for some setup time

    //for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST"){
        wave_freq = usrp->get_tx_rate()/2;
    }

    //error when the waveform is not possible to generate
    if (std::abs(wave_freq) > usrp->get_tx_rate()/2){
        throw std::runtime_error("wave freq out of Nyquist zone");
    }

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("sc16", otw);
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // Normally default spb to 10 packets worth
    if (spb == 0 && !random_spb) {
        spb = tx_stream->get_max_num_samps()*10;
    // If using randomized spb default the spb to be up to 1 packet by default
    } else if(spb == 0) {
        spb = tx_stream->get_max_num_samps();
    }

    // Wave generator used to generate samples
    wave_generator<short> wave_generator(wave_type, ampl, actual_rate, (wave_type == "COMB") ? comb_spacing : wave_freq );

    // How many samples are needed to create a lookup table that will perfectly replicate a wave
    size_t fundamental_period = wave_generator.get_fundamental_period();

    // Limit the size of the sample buffer to avoid excessive resource use
    // Most waves are limited to 2e9 samples (8Gb of RAM)
    // Comb waves are limited to 100e3 samples due to how long lookup table generation takes. If/when generation is optimize it can be increased
    if(wave_type != "COMB") {
        const size_t MAX_COMMON_LUT_SIZE = 2000000000;
        if(fundamental_period > MAX_COMMON_LUT_SIZE) {
        std::cout << "The fundamental period with a wave frequency of " << wave_freq / 1e6 << "MHz and a sample rate of " << actual_rate / 1e6 << "Msps is very large. The lookup table will be limited to " << MAX_COMMON_LUT_SIZE << " samples. This will cause a discontinuity every " << MAX_COMMON_LUT_SIZE / actual_rate << " seconds.\n";
        }
    } else {
        const size_t MAX_COMB_LUT_SIZE = 100000;
        if(fundamental_period > MAX_COMB_LUT_SIZE) {
        std::cout << "The fundamental period with a comb spacing of " << comb_spacing / 1e6 << "MHz and a sample rate of " << actual_rate / 1e6 << "Msps is very large. The lookup table will be limited to " << MAX_COMB_LUT_SIZE << " samples. This will cause a discontinuity every " << MAX_COMB_LUT_SIZE / actual_rate << " seconds.\n";
        }
    }

    std::vector<std::complex<short> > buff(spb + fundamental_period);
    std::vector<std::complex<short> *> buffs(channel_nums.size(), &buff.front());

    //fill the buffer with the waveform
    for (size_t n = 0; n < buff.size(); n++){
        buff[n] = wave_generator(n);
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

    if(!use_constant_time) {
        // Clock sync is lost when setting the time. Getting the time blocks until sync is established
        uhd::time_spec_t converged_time = usrp->get_time_now();

        // Shifts the requested start and stop times to match
        double time_offset = converged_time.get_real_secs();
        first+=time_offset;
        last+=time_offset;
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    bool ignore_last = !vm.count("last");
    bool first_loop = true;

    for(double time = first; ((ignore_last && first_loop ) || time <= last) && !stop_signal_called ; time += increment)
    {
        first_loop = false;
        // Set up metadata. We start streaming a bit in the future
        // to allow MIMO operation:
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst   = false;
        md.has_time_spec  = true;
        md.time_spec = uhd::time_spec_t(time);

        // Initialize rng
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> uniform_distribution(0, spb);

        //send data until the signal handler gets called
        //or if we accumulate the number of samples specified (unless it's 0)
        uint64_t num_acc_samps = 0;
        while(true){

            if (stop_signal_called)
                break;

            if (total_num_samps > 0 and num_acc_samps >= total_num_samps)
                break;

            // Locates where in the buffer to use samples from
            for(auto& buff_ptr : buffs) {
                if(fundamental_period != 0) {
                    buff_ptr = &buff[num_acc_samps % fundamental_period];
                } else {
                    buff_ptr = &buff.front();
                }
            }

            size_t max_samples_to_send;
            if(random_spb) [[unlikely]] {
                // Send up to a random number of samples
                max_samples_to_send = uniform_distribution(gen);
            } else {
                // Send up to spb
                max_samples_to_send = spb;
            }
            // Determine how many samples to send
            size_t nsamps_this_send;
            if(total_num_samps != 0) {
                nsamps_this_send = std::min(max_samples_to_send, total_num_samps - num_acc_samps);
            } else {
                nsamps_this_send = max_samples_to_send;
            }

#ifdef DEBUG_TX_WAVE
            std::cout << "Sending samples" << std::endl;
#endif
            //this statement will block until the data is sent
            //send the entire contents of the buffer
            num_acc_samps += tx_stream->send(buffs, nsamps_this_send, md);
#ifdef DEBUG_TX_WAVE
            std::cout << "Sent samples" << std::endl;
#endif
            md.start_of_burst = false;
            md.has_time_spec = false;
        }
        //send a mini EOB packet
        md.end_of_burst = true;
        tx_stream->send("", 0, md);
    }
#ifdef DELAYED_EXIT
//waits until told to stop before continuing (allows closing tasks to be delayed)
    while(!stop_signal_called) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
#endif
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
