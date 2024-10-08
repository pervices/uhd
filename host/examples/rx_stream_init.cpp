//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2021 Per Vices Corporation
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
    size_t num_requested_samples = 0,
    double time_requested = 0.0,
    bool enable_size_map = false,
    bool continue_on_bad_packet = false,
    double rate = 0,
    std::string pre_exec_file = ""
){
    //create a receive streamer
    uhd::stream_args_t stream_args(cpu_format,wire_format);
    std::vector<size_t> channel_nums;
    channel_nums.push_back(channel);
    stream_args.channels = channel_nums;

    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    uhd::rx_metadata_t md;
    std::vector<samp_type> buff(samps_per_buff);

    //setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)?
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
        uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
    );
    stream_cmd.num_samps = size_t(num_requested_samples);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();

    int pre_pid = 0;

    if(!pre_exec_file.empty()) {
        pre_pid = run_exec(pre_exec_file);
    }

    rx_stream->issue_stream_cmd(stream_cmd);

    typedef std::map<size_t,size_t> SizeMap;
    SizeMap mapSizes;

    rx_stream->recv(&buff.front(), buff.size(), md, 3.0, enable_size_map);

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cerr << boost::format("First packet timed out. It is likely that no data will be received") << std::endl;
        }

    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
        std::string error = str(boost::format("Receiver error: %s") % md.strerror());
        if (continue_on_bad_packet){
            std::cerr << error << std::endl;
        }
        else
            throw std::runtime_error(error);
    }

    //waits for samples to be received
    if(num_requested_samples > 0) {
        //This line could be optimized to minimize floating point rounding errors
        std::this_thread::sleep_for(std::chrono::microseconds((int)((num_requested_samples/rate + 1)*(1e6))));
    } else if(time_requested>0) {
        std::this_thread::sleep_for(std::chrono::microseconds((int)(time_requested*1e6)));
    }

    if(pre_pid != 0) {
        kill(pre_pid, SIGTERM);
        int status;
        waitpid(pre_pid, &status, 0);
    }

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

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

//pass this a the string that would be used if launching the program from the command line
int run_exec(std::string argument) {
    std::vector<std::string> args_builder{};

    std::string arg_builder = "";

    bool is_escaped = false;

    bool is_in_quotes = false;

    for(uint64_t n = 0; n <argument.length(); n++) {
        if(is_escaped) {
            arg_builder.push_back(argument.at(n));
            is_escaped = false;
        } else if(argument.at(n)=='\\') {
            is_escaped = true;
        } else if(argument.at(n)=='\"') {
            if(is_escaped) {
                arg_builder.push_back(argument.at(n));
            } else {
                is_in_quotes = !is_in_quotes;
            }
        } else if(argument.at(n)==' ' && !(is_escaped||is_in_quotes)) {
            args_builder.push_back(arg_builder);
            arg_builder = "";
        }
        else {
            arg_builder.push_back(argument.at(n));
        }
    }

    if(!arg_builder.empty()) {
        args_builder.push_back(arg_builder);
    }

    char *args[args_builder.size()+1];

    for(uint64_t n = 0; n <args_builder.size(); n++) {
        args[n] = new char[args_builder[n].size()+1];
        std::strcpy(args[n], args_builder[n].c_str());
    }

    args[args_builder.size()] = NULL;



    int child_pid = fork();

    if(child_pid == 0) {
        execvp(args[0], args);
        std::cerr << boost::format("Failed to launch: %s" ) % args[0] << std::endl;
        int n = 0;
        while(args[n]!=NULL) {
            std::cout << args[n] << std::endl;
            n++;
        }
        std::exit(1);
        return 0;
    }
    else {
        return child_pid;
    }
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, type, ant, subdev, ref, wirefmt, pre_exec_file, post_exec_file;
    size_t channel, total_num_samps, spb;
    double rate, gain, bw, total_time, setup_time, lo_freq, dsp_freq;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive. Noted: preexec will be stopped at the end of the duration. If preexec has its own set duration and does post processing, make sure this is set to be long enough for preexec to finish.")
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
        ("preexecfile", po::value<std::string>(&pre_exec_file), "The file that should start immediately prior to the device starting to stream data. Must specify the path (ie starting with ./, ~/, /)")
        ("postexecfile", po::value<std::string>(&post_exec_file), "The file that should be run after finishing streaming data. Must specify the path (ie starting with ./, ~/, /)")
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
    bool continue_on_bad_packet = vm.count("continue") > 0;

    if (enable_size_map)
        std::cout << "Packet size tracking enabled - will only recv one packet at a time!" << std::endl;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

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

    if (total_num_samps == 0){
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

#define recv_to_file_args(format) \
    (usrp, format, wirefmt, channel, spb, total_num_samps, total_time, enable_size_map, continue_on_bad_packet, rate, pre_exec_file)
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

    //launches post exec and waits for it to finish
    int post_pid = 0;

    if(!post_exec_file.empty()) {
        post_pid = run_exec(post_exec_file);
    }
    int status = 0;
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    //makes sure the other process has had a chance to run
    //do{
    waitpid(post_pid, &status, 0);
    //} while

    return EXIT_SUCCESS;
}
