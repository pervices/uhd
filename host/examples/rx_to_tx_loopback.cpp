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
#include <unistd.h>
#include <algorithm>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

// Ideally there should be a lock to prevent caching resulting in data not being shared, however I cannot find a way to specify only what needs to be locked
// Due to the amount of data being shared any chaches will probably get emptied
//number of buffers to use, must be a power of 2
const size_t num_buffers = 8;
const size_t valid_index_mask = num_buffers - 1;
//how much percent of required samples between rx and tx buffer are used for each loopback
const double host_buff_size = 0.25;
// space at the end of buffer that is used to make it easier to avoid issues when samples can only be sent in specific amounts
const size_t spare_buffer_space = 10000;
//TODO change send function so that it will partially send instead of sending the entire buffer when the device's buffer would dip below the setpoint
const size_t max_samples_per_tx = 6000;
// The inner buffer is a buffer containing data received from each channel
// The middle layer groups the buffer for each ch together
// The outer layer exists so that the rx can receive to once set of buffers and tx the other, so that they don't need to lock and unlock every send/recv
std::vector<std::vector<std::vector<std::complex<short>>>> buffers;
std::vector<std::atomic<size_t>> buffer_used(num_buffers);
std::vector<std::atomic<bool>> buff_ready(num_buffers);

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

    while ((num_acc_samps < total_num_samps || total_num_samps == 0) && !stop_signal_called) {

        for(size_t n = 0; n < num_channels; n++) {
            active_buff_ptrs[n] = &(*active_buffer)[n][samples_this_buffer];
        }

        // receive a single packet
        size_t samps_received = rx_stream->recv(active_buff_ptrs, max_samples_per_buffer - samples_this_buffer, rx_md, timeout, true);

        num_acc_samps+=samps_received;
        samples_this_buffer+=samps_received;

        if(samples_this_buffer + spare_buffer_space > max_samples_per_buffer) {
            buffer_used[active_buffer_index] = samples_this_buffer;
            buff_ready[active_buffer_index] = true;
            active_buffer_index++;
            //caps the active to be less than the number of buffers
            active_buffer_index = active_buffer_index & valid_index_mask;

            active_buffer = &buffers[active_buffer_index];

            samples_this_buffer = 0;
        }
    }

    for(size_t n = 0; n < num_buffers; n++) {
        buff_ready[n] = true;
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
    while(!buff_ready[active_buffer_index]) {

    }
    size_t samples_to_send_this_buffer = buffer_used[active_buffer_index];
    
    while ((num_acc_samps < total_num_samps || total_num_samps == 0) && !stop_signal_called) {

        if(samples_this_buffer >= samples_to_send_this_buffer) {
            active_buffer_index++;
            active_buffer_index = active_buffer_index & valid_index_mask;
            samples_this_buffer = 0;
            samples_to_send_this_buffer = buffer_used[active_buffer_index];
            active_buffer = &buffers[active_buffer_index];
            while(!buff_ready[active_buffer_index]) {

            }
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
        buff_ready[n] = false;
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
    
    rx_thread.join();
    tx_thread.join();
    
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
