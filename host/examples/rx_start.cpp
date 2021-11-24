//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <sys/wait.h>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

int run_exec(std::string argument);

typedef std::function<uhd::sensor_value_t(const std::string&)> get_sensor_fn_t;

bool check_locked_sensor(
    std::vector<std::string> sensor_names,
    const char* sensor_name,
    get_sensor_fn_t get_sensor_fn,
    double setup_time
) {
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
        return false;

    auto setup_timeout =
        std::chrono::steady_clock::now()
        + std::chrono::milliseconds(int64_t(setup_time * 1000));
    bool lock_detected = false;

    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();

    while (true) {
        if (lock_detected and
            (std::chrono::steady_clock::now() > setup_timeout)) {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()) {
            std::cout << "+";
            std::cout.flush();
            lock_detected = true;
        }
        else {
            if (std::chrono::steady_clock::now() > setup_timeout) {
                std::cout << std::endl;
                throw std::runtime_error(str(
                    boost::format("timed out waiting for consecutive locks on sensor \"%s\"")
                    % sensor_name
                ));
            }
            std::cout << "_";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << std::endl;
    return true;
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, type, ant, subdev, ref, wirefmt, channel_list;
    size_t spb;
    double rate, gain, bw, setup_time, lo_freq, dsp_freq;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("rate", po::value<double>(&rate)->default_value(1e6), "rate of incoming samples")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8, sc16 or s16)")
        ("skip-lo", "skip checking LO lock status")
        ("int-n", "tune USRP with integer-N tuning")

        //stuff added for rx_stream
        ("lo-freq", po::value<double>(&lo_freq), "To amount to shift the signal's frequency down using the lo mixer. The sum of dsp and lo must be greater than the minimum frequency of the second lowest band for lo to work")
        ("dsp-freq", po::value<double>(&dsp_freq), "The amount to shift the signal's frequency using the cordic mixer. A positive value shift the frequency down")
        ("secs", po::value<double>(&setup_time)->default_value(5.0), "seconds of setup time")

    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX stream init %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This application streams data from a single channel of a USRP device, and launches programs to process it.\n"
            << std::endl;
        return ~0;
    }

    bool enable_size_map = vm.count("sizemap") > 0;

    if (enable_size_map)
        std::cout << "Packet size tracking enabled - will only recv one packet at a time!" << std::endl;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //detect which channels to use
    std::vector<size_t> channel_nums;
    size_t start_index = 0;;
    while(start_index < channel_list.size()) {
        while(channel_list[start_index] < '0' || channel_list[start_index] > '9') {
            start_index++;
            if(start_index == channel_list.size()) {
                break;
            }
        }
        size_t stop_index = start_index;
        while(channel_list[stop_index] >= '0' && channel_list[stop_index] <= '9') {
            stop_index++;
            if(stop_index==channel_list.size()) {
                break;
            }
        }

        if(stop_index>start_index) {

            size_t channel = std::stoi(channel_list.substr(start_index, stop_index));
            if(channel >= usrp->get_rx_num_channels()){
                std::string error_msg = "Invalid channel specified: ";
                error_msg.append(std::to_string(channel));
                throw std::runtime_error(error_msg);
            } else {
                channel_nums.push_back(channel);
            }
        } else {
            break;
        }
        start_index = stop_index;
    }

    //Lock mboard clocks
    usrp->set_clock_source(ref);

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (rate <= 0.0){
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return ~0;
    }

    if (vm.count("rate")) {
        for(size_t n = 0; n < channel_nums.size(); n++) {
            size_t channel = channel_nums[n];
            std::cout << boost::format("Setting ch%i RX Rate: %f") % channel % rate << std::endl;
            usrp->set_rx_rate(rate, channel);
            std::cout << boost::format("Actual ch%i RX Rate: %f") % channel % usrp->get_rx_rate(channel) << std::endl << std::endl;
        }
    }

    //set the center frequency
    if (vm.count("lo-freq") && vm.count("dsp-freq")) { //with default of 0.0 this will always be true
        for(size_t n = 0; n < channel_nums.size(); n++) {
            size_t channel = channel_nums[n];
            double freq = lo_freq+dsp_freq;
            std::cout << boost::format("Setting ch%i RX Freq: %f MHz...") % channel % (freq/1e6) << std::endl;
            // the overload used required an extra argument to avoid conflict with a different constructor, the 0 does nothing else
            uhd::tune_request_t tune_request(-dsp_freq, lo_freq, 0);
            if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
            usrp->set_rx_freq(tune_request, channel);
            std::cout << boost::format("Actual ch%i RX Freq: %f MHz...") % channel % (usrp->get_rx_freq(channel)/1e6) << std::endl << std::endl;
        }
    } else {
        std::cerr << "Please specify a dsp shift and lo frequency" << std::endl;
        return ~0;
    }

    //set the rf gain
    if (vm.count("gain")) {
        for(size_t n = 0; n < channel_nums.size(); n++) {
            size_t channel = channel_nums[n];
            std::cout << boost::format("Setting ch%i RX Gain: %f dB...") % channel % gain << std::endl;
            usrp->set_rx_gain(gain, channel);
            std::cout << boost::format("Actual ch%i RX Gain: %f dB...") % channel % usrp->get_rx_gain(channel) << std::endl << std::endl;
        }
    }

    //set the IF filter bandwidth
    if (vm.count("bw")) {
        for(size_t n = 0; n < channel_nums.size(); n++) {
            size_t channel = channel_nums[n];
            std::cout << boost::format("Setting ch%i RX Bandwidth: %f MHz...") % channel % (bw/1e6) << std::endl;
            usrp->set_rx_bandwidth(bw, channel);
            std::cout << boost::format("Actual ch%i RX Bandwidth: %f MHz...") % channel % (usrp->get_rx_bandwidth(channel)/1e6) << std::endl << std::endl;
        }
    }

    //set the antenna
    if (vm.count("ant")) {
        for(size_t n = 0; n < channel_nums.size(); n++) {
            size_t channel = channel_nums[n];
            usrp->set_rx_antenna(ant, channel);
        }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(int64_t(1000 * setup_time))
    );

    //check Ref and LO Lock detect
    if (not vm.count("skip-lo")){
        for(size_t n = 0; n < channel_nums.size(); n++) {
            size_t channel = channel_nums[n];
            check_locked_sensor(
                usrp->get_rx_sensor_names(channel),
                "lo_locked",
                [usrp,channel](const std::string& sensor_name){
                    return usrp->get_rx_sensor(sensor_name, channel);
                },
                setup_time
            );
            if (ref == "mimo") {
                check_locked_sensor(
                    usrp->get_mboard_sensor_names(0),
                    "mimo_locked",
                    [usrp](const std::string& sensor_name){
                        return usrp->get_mboard_sensor(sensor_name);
                    },
                    setup_time
                );
            }
            if (ref == "external") {
                check_locked_sensor(
                    usrp->get_mboard_sensor_names(0),
                    "ref_locked",
                    [usrp](const std::string& sensor_name){
                        return usrp->get_mboard_sensor(sensor_name);
                    },
                    setup_time
                );
            }
        }
    }

    //start streaming. THis method is different from the conventional method
    for(size_t n = 0; n <channel_nums.size(); n ++) {
        size_t channel = channel_nums[n];
        std::string path_buffer = "/mboards/0/rx/";
        path_buffer.append(std::to_string(channel));
        path_buffer.append("/force_stream");
        usrp->set_tree_value(path_buffer, 1);
    }

    return EXIT_SUCCESS;
}
