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
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include "wavetable.hpp"
#include <uhd/utils/static.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <unistd.h>
#include <algorithm>
#include <semaphore.h>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

// Ideally there should be a lock to prevent caching resulting in data not being shared, however I cannot find a way to specify only what needs to be locked
// Due to the amount of data being shared any chaches will probably get emptied
//std::shared_timed_mutex mtx;
//number of buffers to use, must be a power of 2
const size_t num_buffers = 8;
const size_t valid_index_mask = num_buffers - 1;
//how much percent of required samples between rx and tx buffer are used for each loopback
const double host_buff_size = 0.25;
// space at the end of buffer that is used to make it easier to avoid issues when samples can only be sent in specific amounts
const size_t spare_buffer_space = 10000;
//TODO change send function so that it will partially send instead of sending the entire buffer when the device's buffer would dip below the setpoint
const size_t max_samples_per_tx = 8000;
// The inner buffer is a buffer containing data received from each channel
// The middle layer groups the buffer for each ch together
// The outer layer exists so that the rx can receive to once set of buffers and tx the other, so that they don't need to lock and unlock every send/recv
std::vector<std::vector<std::vector<std::complex<short>>>> buffers;
std::vector<std::atomic<size_t>> buffer_used(num_buffers);
std::vector<sem_t> buff_ready(num_buffers);

void rx_run(uhd::rx_streamer::sptr rx_stream, double start_time, size_t total_num_samps) {
    uhd::set_thread_priority_safe();
    double timeout = start_time + 0.1; // timeout (delay before receive + padding)
    size_t num_acc_samps = 0;
    size_t active_buffer_index = 0;
    size_t num_channels = buffers[0].size();
    size_t max_samples_per_buffer = buffers[0][0].size();
    uhd::rx_metadata_t rx_md;

    std::vector<std::vector<std::complex<short>>> *active_buffer;
    std::vector<std::complex<short>*> active_buff_ptrs(num_channels);

    size_t samples_this_buffer = 0;

    active_buffer = &buffers[active_buffer_index];
    //std::unique_lock<std::shared_timed_mutex> lock(mtx);

    while ((num_acc_samps < total_num_samps || total_num_samps == 0) && !stop_signal_called) {

        for(size_t n = 0; n < num_channels; n++) {
            active_buff_ptrs[n] = &(*active_buffer)[n][samples_this_buffer];
        }

        // receive a single packet
        size_t samps_received = rx_stream->recv(active_buff_ptrs, max_samples_per_buffer - samples_this_buffer, rx_md, timeout, true);

        num_acc_samps+=samps_received;
        samples_this_buffer+=samps_received;

        if(samples_this_buffer + spare_buffer_space > max_samples_per_buffer) {
            //lock.unlock();
            buffer_used[active_buffer_index] = samples_this_buffer;
            sem_post(&buff_ready[active_buffer_index]);
            active_buffer_index++;
            //caps the active to be less than the number of buffers
            active_buffer_index = active_buffer_index & valid_index_mask;

            active_buffer = &buffers[active_buffer_index];

            samples_this_buffer = 0;
            //lock.lock();
        }
    }

    for(size_t n = 0; n < num_buffers; n++) {
        sem_post(&buff_ready[n]);
    }

    //lock.unlock();

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stream_cmd);
}
void tx_run( uhd::tx_streamer::sptr tx_stream, double start_time, size_t total_num_samps) {
    uhd::set_thread_priority_safe();
    size_t num_acc_samps = 0;
    size_t active_buffer_index = 0;
    size_t num_channels = buffers[0].size();
    size_t max_samples_per_buffer = std::min(buffers[0][0].size(), max_samples_per_tx);
    // Set up Tx metadata. We start streaming a bit in the future
    uhd::tx_metadata_t tx_md;
    tx_md.start_of_burst = true;
    tx_md.end_of_burst   = false;
    tx_md.has_time_spec  = true;
    tx_md.time_spec = uhd::time_spec_t(start_time);

    std::vector<std::complex<short>*> active_buff_ptrs(num_channels);

    std::vector<std::vector<std::complex<short>>> *active_buffer;
    active_buffer = &buffers[active_buffer_index];
    size_t samples_this_buffer = 0;

    sem_wait(&buff_ready[active_buffer_index]);
    size_t samples_to_send_this_buffer = buffer_used[active_buffer_index];
    //std::shared_lock<std::shared_timed_mutex> lock(mtx);
    
    while ((num_acc_samps < total_num_samps || total_num_samps == 0) && !stop_signal_called) {

        if(samples_this_buffer >= samples_to_send_this_buffer) {
            //lock.unlock();
            active_buffer_index++;
            active_buffer_index = active_buffer_index & valid_index_mask;
            samples_this_buffer = 0;
            samples_to_send_this_buffer = buffer_used[active_buffer_index];
            active_buffer = &buffers[active_buffer_index];
            sem_wait(&buff_ready[active_buffer_index]);
            //lock.lock();
        }

        for(size_t n = 0; n < num_channels; n++) {
            active_buff_ptrs[n] = &(*active_buffer)[n][samples_this_buffer];
        }

        size_t samples_sent = tx_stream->send(active_buff_ptrs, std::min(samples_to_send_this_buffer - samples_this_buffer, max_samples_per_buffer), tx_md);
        samples_this_buffer+=samples_sent;
        num_acc_samps+=samples_sent;

        tx_md.start_of_burst = false;
        tx_md.has_time_spec = false;
    }
    tx_md.end_of_burst = true;
    tx_stream->send("", 0, tx_md);
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, channel_list, ref;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps;
    double rate,freq,tx_gain, rx_gain, offset;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to begin receiving")
        ("rx_lead", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming (Rx) and outgoing (Tx) samples")
        //("dilv", "specify to disable inner-loop verbose")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        //("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("tx_gain", po::value<double>(&tx_gain), "gain for the Tx RF chain")
        ("rx_gain", po::value<double>(&rx_gain), "gain for the Rx RF chain")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "Number of samples to relay between tx and rx. Leave at 0 for run continuously")
        ("offset", po::value<double>(&offset)->default_value(1), "Delay between rx and tx in seconds")
        //("ant", po::value<std::string>(&ant), "antenna selection")
        //("subdev", po::value<std::string>(&subdev), "subdevice specification")
        //("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo, gpsdo)")
        //("pps", po::value<std::string>(&pps)->default_value("internal"), "PPS source (internal, external, mimo, gpsdo)")
        //("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        //("int-n", "tune USRP with integer-N tuning")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX_TX continuous stream %s") % desc << std::endl;
        return ~0;
    }

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    //std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;
    

    // detect which channels to use
    std::vector<std::string> channel_strings;
        std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < channel_strings.size(); ch++) { 
        std::cout <<"works up to here 1.2!" << std::endl;
        size_t chan = std::stoi(channel_strings[ch]) ;
        usrp->get_tx_num_channels();
        usrp->get_rx_num_channels();
        if(chan >= usrp->get_tx_num_channels() || chan >= usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else {
            channel_nums.push_back(std::stoi(channel_strings[ch]));
        }
    }
    std::cout <<"works up to here 2!" << std::endl;    
        
//         usrp->get_rx_num_channels(); 
//         usrp->get_tx_num_channels();
//         std::cout <<"works up to here 1.51!" << std::endl;
//         //if (chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()) {
//         if (chan >= usrp->get_tx_num_channels()) {
//         std::cout <<"works up to here 1.6!" << std::endl;
//             throw std::runtime_error("Invalid channel(s) specified.");
//         std::cout <<"works up to here 1.7!" << std::endl;
//         }else
//             channel_nums.push_back(std::stoi(channel_strings[ch]));

    // set the tx sample rate
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_tx_rate(rate);
    double actual_tx_rate = usrp->get_tx_rate();
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (actual_tx_rate / 1e6)
              << std::endl
              << std::endl;

    // set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_rx_rate(rate);
    double actual_rx_rate = usrp->get_rx_rate();
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (actual_rx_rate / 1e6)
              << std::endl
              << std::endl;

    if(actual_rx_rate != actual_tx_rate) {
        std::cerr << "Tx and Rx rate mismatch, basing future calculations of of tx" << std::endl;
    }
    rate = actual_tx_rate;

    for(size_t ch = 0; ch < channel_nums.size(); ch++){
        //set the Rx rf gain
        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
        usrp->set_rx_gain(rx_gain, channel_nums[ch]);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain(channel_nums[ch]) << std::endl << std::endl;
    
        //set the Tx rf gain
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
        usrp->set_tx_gain(tx_gain, channel_nums[ch]);
        std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;
        
    }

    // Lock mboard clocks
    usrp->set_clock_source(ref);

    // create a receive streamer
    uhd::stream_args_t stream_args("sc16"); //short complex
    stream_args.channels             = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    
    // setup streaming
    std::cout << std::endl;
    //std::cout << boost::format("Begin streaming %u samples, %f seconds in the future...")
    //                 % total_num_samps % seconds_in_future
    //          << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS); 
    stream_cmd.num_samps  = total_num_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(seconds_in_future);
    
    size_t num_channels = channel_nums.size();
    // space at the end of buffer that is used to make it easier to avoid issues when samples can only be sent in specific amounts
    //size of the buffers used to store data from individual send/receive commands
    size_t buffer_size = std::ceil(offset * rate * host_buff_size + spare_buffer_space);
    // allocate buffer to receive with samples
    // This program uses a rung buffer of buffers to send a receive data
    // [which buffer][channel][location in data buffer]
    // Data is sent to send/recv functions from the data buffer
    buffers = std::vector<std::vector<std::vector<std::complex<short>>>>(num_buffers);
    for(size_t a = 0; a < num_buffers; a++) {
        buffers[a] = std::vector<std::vector<std::complex<short>>>(num_channels);
        for(size_t b = 0; b < num_channels; b++) {
            buffers[a][b] = std::vector<std::complex<short>>(buffer_size);
        }
    }

    for(size_t n = 0; n < num_buffers; n++) {
        sem_init(&buff_ready[n], 0, 0);
    }
    
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));

    rx_stream->issue_stream_cmd(stream_cmd);

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    //the arrays passed here need to be changed to be passed by reference, and locking added
    std::thread rx_thread(rx_run, rx_stream, seconds_in_future, total_num_samps);

    //the arrays passed here need to be changed to be passed by reference, and locking added
    std::thread tx_thread(tx_run, tx_stream, seconds_in_future + offset, total_num_samps);

    std::cout << "D1\n";
    
    rx_thread.join();
    tx_thread.join();
    
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
    

//********************************************************************************************************************************************************//
                        //TX OUTPUTTING RX SAMPLE BUFFER
//********************************************************************************************************************************************************//

/*    
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
    
    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    //allocate a buffer which we re-use for each channel
    if (spb == 0) {
        spb = tx_stream->get_max_num_samps()*10;
    }
    std::vector<std::complex<float> > buff(spb);
    std::vector<std::complex<float> *> buffs(channel_nums.size(), &buff.front());
#ifdef DELAYED_EXIT
//waits until told to stop before continuing (allows closing tasks to be delayed)
    while(!stop_signal_called) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    
#endif
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
    */
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /*
    
    
    
    

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T7: Type any letter then enter to continue" << std::endl;
    std::string tmp;
    std::cin >> tmp;
#endif

    //Lock mboard clocks
    usrp->set_clock_source(ref);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T6: Type any letter then enter to continue" << std::endl;
    std::cin >> tmp;
#endif

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;

    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T5: Type any letter then enter to continue" << std::endl;
    std::cin >> tmp;
#endif

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_tx_freq(tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        //set the Tx rf gain
        if (vm.count("gain")){
            std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
            usrp->set_tx_gain(tx_gain, channel_nums[ch]);
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

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T4: Type any letter then enter to continue" << std::endl;
    std::cin >> tmp;
#endif

    std::this_thread::sleep_for(std::chrono::seconds(1)); //allow for some setup time

//     //for the const wave, set the wave freq for small samples per period
//     if (wave_freq == 0 and wave_type == "CONST"){
//         wave_freq = usrp->get_tx_rate()/2;
//     }
// 
//     //error when the waveform is not possible to generate
//     if (std::abs(wave_freq) > usrp->get_tx_rate()/2){
//         throw std::runtime_error("wave freq out of Nyquist zone");
//     }
//     if (usrp->get_tx_rate()/std::abs(wave_freq) > wave_table_len/2){
//         throw std::runtime_error("wave freq too small for table");
//     }
// 
//     //pre-compute the waveform values
//     const wave_table_class wave_table(wave_type, ampl);
//     const size_t step = boost::math::iround(wave_freq/rate * wave_table_len);
//     size_t index = 0;

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    //allocate a buffer which we re-use for each channel
    if (spb == 0) {
        spb = tx_stream->get_max_num_samps()*10;
    }
    std::vector<std::complex<float> > buff(spb);
    std::vector<std::complex<float> *> buffs(channel_nums.size(), &buff.front());

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T3: Type any letter then enter to continue" << std::endl;
    std::cin >> tmp;
#endif

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
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

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T2: Type any letter then enter to continue" << std::endl;
    std::cin >> tmp;
#endif

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

#ifdef DEBUG_TX_WAVE_STEP
    std::cout << "Manually configure the state tree now (if necessary)" << std::endl;
    std::cout << "T1: Type any letter then enter to continue" << std::endl;
    std::cin >> tmp;
#endif

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    usrp->set_time_now(0.0);

    bool ignore_last = first > last;
    

    for(double time = first; (ignore_last || time <= last) && !stop_signal_called ; time += increment)
    {
        // Set up metadata. We start streaming a bit in the future
        // to allow MIMO operation:
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst   = false;
        md.has_time_spec  = true;
        md.time_spec = uhd::time_spec_t(time);

        //send data until the signal handler gets called
        //or if we accumulate the number of samples specified (unless it's 0)
        uint64_t num_acc_samps = 0;
        while(true){

            if (stop_signal_called)
                break;

            if (total_num_samps > 0 and num_acc_samps >= total_num_samps)
                break;

            //fill the buffer with the waveform
            size_t n = 0;
            for (n = 0; n < buff.size() && (num_acc_samps + n < total_num_samps || total_num_samps == 0); n++){
                buff[n] = wave_table(index += step);
            }
#ifdef DEBUG_TX_WAVE
            std::cout << "Sending samples" << std::endl;
#endif
            //this statement will block until the data is sent
            //send the entire contents of the buffer
            num_acc_samps += tx_stream->recv(buffs, buff.size(), md, timeout, true)  // changing buffer to that which was received
#ifdef DEBUG_TX_WAVE
            std::cout << "Sent samples" << std::endl;
#endif
            md.start_of_burst = false;
            md.has_time_spec = false;
        }
#ifdef DEBUG_TX_WAVE
        std::cout << "Creating EOB packet" << std::endl;
#endif
        //send a mini EOB packet
        md.end_of_burst = true;
#ifdef DEBUG_TX_WAVE
        std::cout << "Sending EOB packet" << std::endl;
#endif
        tx_stream->send("", 0, md);
#ifdef DEBUG_TX_WAVE
        std::cout << "Sent EOB packet" << std::endl;
#endif
    }
#ifdef DELAYED_EXIT
//waits until told to stop before continuing (allows closing tasks to be delayed)
    while(!stop_signal_called) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
#endif
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}*/
