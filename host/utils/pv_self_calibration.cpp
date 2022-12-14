//
// Copyright 2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <iostream>
#include <thread>
#include <algorithm>

namespace po = boost::program_options;

static const size_t self_calibration_nsamps = 2000;

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("force", "Run self calibration, even if it has already be performed")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX Multi Samples %s") % desc << std::endl;
        std::cout
            << "This program performs self calibration"
            << std::endl;
        return ~0;
    }

    bool force = vm.count("force");

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args
              << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    if(!force) {
        if(usrp->is_device_is_self_calibration_required()) {
            std::cout << "Self calibration is either not required for this device or has already been run" << std::endl;
            std::cout << "To run self callibration on a unit where self calibration has already been performed run this program with --force" << std::endl;
            std::exit(0);
        }
    }

    // detect which channels to use
    std::vector<size_t> channel_nums;
    size_t num_rx_channels = usrp->get_rx_num_channels();
    for (size_t ch = 0; ch < num_rx_channels; ch++) {
            channel_nums.push_back(ch);
    }

    // create a receive streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("sc16"); // complex floats
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    std::cout << "Begining self calibration" << std::endl;


    // Set band to invald band (equivalent to not connected)
    for(size_t ch = 0; ch < num_rx_channels; ch++){
        uhd::tune_request_t rx_tune_request(0);
        usrp->set_rx_freq(rx_tune_request, ch);
    }

    for(size_t ch = 0; ch < num_rx_channels; ch++){
        //set the Rx rf gain
        usrp->set_rx_gain(0, ch);
    }

    // set the rx sample rate to maximum
    usrp->set_rx_rate(usrp->get_max_rate_mhz());

    std::cout << "Setting device timestamp to 0" << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));

    // setup streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = self_calibration_nsamps;
    stream_cmd.stream_now = true;
    stream_cmd.time_spec  = uhd::time_spec_t(0.0);
    rx_stream->issue_stream_cmd(stream_cmd); // tells all channels to stream

    // meta-data will be filled in by recv()
    uhd::rx_metadata_t md;

    // allocate buffers to receive with samples (one buffer per channel)
    const size_t samps_per_buff = std::min(rx_stream->get_max_num_samps(), self_calibration_nsamps);
    std::vector<std::vector<std::complex<float>>> buffs(
        usrp->get_rx_num_channels(), std::vector<std::complex<float>>(samps_per_buff));


    size_t num_acc_samps = 0; // number of accumulated samples
    double timeout = 5;

    while(true) {
        while (num_acc_samps < self_calibration_nsamps) {

            // create a vector of pointers to point to each of the channel buffers
            std::vector<std::complex<float>*> buff_ptrs;
            for (size_t i = 0; i < buffs.size(); i++) {
                buff_ptrs.push_back(&buffs[i].at(num_acc_samps));
            }
            // receive a single packet
            size_t num_rx_samps = rx_stream->recv(buff_ptrs, std::min(samps_per_buff, self_calibration_nsamps - num_acc_samps), md, timeout);

            // use a small timeout for subsequent packets
            timeout = 0.1;

            // handle the error code
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
                break;
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                throw std::runtime_error(
                    str(boost::format("Receiver error %s") % md.strerror()));
            }

            num_acc_samps += num_rx_samps;
        }
        if (num_acc_samps < self_calibration_nsamps) {
            std::cerr << "Error while receving samples, re-attempting" << std::endl;
        }
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
