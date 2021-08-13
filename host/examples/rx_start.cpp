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

int run_exec(std::string argument);

template<typename samp_type> void recv_to_file(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &cpu_format,
    const std::string &wire_format,
    const size_t &channel,
    size_t samps_per_buff,
    bool enable_size_map = false,
    bool continue_on_bad_packet = false,
    double rate = 0
){
    //create a receive streamer
//     uhd::stream_args_t stream_args(cpu_format,wire_format);
//     std::vector<size_t> channel_nums;
//     channel_nums.push_back(channel);
//     stream_args.channels = channel_nums;
//
//     uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
//
//     uhd::rx_metadata_t md;
//     std::vector<samp_type> buff(samps_per_buff);
//
//      uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
//
//     stream_cmd.stream_now = true;
//     stream_cmd.time_spec = uhd::time_spec_t();
//
//     rx_stream->issue_stream_cmd(stream_cmd);
}

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

    std::cout << "T-10" << std::endl;

    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, type, ant, subdev, ref, wirefmt;
    size_t channel, spb;
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
        ("channel", po::value<size_t>(&channel)->default_value(0), "which channel to use")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8, sc16 or s16)")
        ("continue", "don't abort on a bad packet")
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

    std::cout << "T-7" << std::endl;

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
    bool continue_on_bad_packet = vm.count("continue") > 0;

    if (enable_size_map)
        std::cout << "Packet size tracking enabled - will only recv one packet at a time!" << std::endl;

    std::cout << "T-2" << std::endl;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;

    std::cout << "T0" << std::endl;

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    std::cout << "T100" << std::endl;

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

    //set the center frequency
    if (vm.count("lo-freq") && vm.count("dsp-freq")) { //with default of 0.0 this will always be true
        double freq = lo_freq+dsp_freq;
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        //The way that tune request parameters work does not match their names
        //Inspect how the code that uses tune requests (tune.cpp) actually interacts with *impl.cpp
        uhd::tune_request_t tune_request(dsp_freq+lo_freq, lo_freq, 0, 0, -dsp_freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request, channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq(channel)/1e6) << std::endl << std::endl;
    } else {
        std::cerr << "Please specify a dsp shift and lo frequency" << std::endl;
        return ~0;
    }

    //set the rf gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain, channel);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain(channel) << std::endl << std::endl;
    }

    //set the IF filter bandwidth
    if (vm.count("bw")) {
        std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw/1e6) << std::endl;
        usrp->set_rx_bandwidth(bw, channel);
        std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % (usrp->get_rx_bandwidth(channel)/1e6) << std::endl << std::endl;
    }

    //set the antenna
    if (vm.count("ant")) usrp->set_rx_antenna(ant, channel);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(int64_t(1000 * setup_time))
    );

    //check Ref and LO Lock detect
    if (not vm.count("skip-lo")){
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

#define recv_to_file_args(format) \
    (usrp, format, wirefmt, channel, spb, enable_size_map, continue_on_bad_packet, rate)
    //recv to file
    if (wirefmt == "s16") {
        if (type == "double") recv_to_file<double>recv_to_file_args("f64");
        else if (type == "float") recv_to_file<float>recv_to_file_args("f32");
        else if (type == "short") recv_to_file<short>recv_to_file_args("s16");
        else throw std::runtime_error("Unknown type " + type);
    } else {
        if (type == "double") recv_to_file<std::complex<double> >recv_to_file_args("fc64");
        else if (type == "float") recv_to_file<std::complex<float> >recv_to_file_args("fc32");
        else if (type == "short") recv_to_file<std::complex<short> >recv_to_file_args("sc16");
        else throw std::runtime_error("Unknown type " + type);
    }

    if (vm.count("rate")) {
        std::cout << boost::format("Setting RX Rate: %f") % rate << std::endl;
        usrp->set_rx_rate(rate, channel);
        std::cout << boost::format("Actual RX Rate: %f") % usrp->get_rx_rate(channel) << std::endl << std::endl;
    }

    //start streaming. THis method is different from the conventional method
    std::string path_buffer = "/mboards/0/rx/";
    path_buffer.append(std::to_string(channel));
    path_buffer.append("/force_stream");
    usrp->set_tree_value(path_buffer, 1);

    return EXIT_SUCCESS;
}
