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

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")

    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX stream init %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This application stops rx from streaming data by through register writes.\n"
            << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //Lock mboard clocks
    usrp->set_clock_source(ref);

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //detect which channels to use
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
        int stop_index = start_index;
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

    //start streaming. THis method is different from the conventional method
    for(int n =0; n <channel_nums.size();n++) {
        std::string path_buffer = "/mboards/0/rx/";
        path_buffer.append(std::to_string(channel_nums[n]));
        path_buffer.append("/force_stream");
        usrp->set_tree_value(path_buffer, 0);
    }

    return EXIT_SUCCESS;
}
