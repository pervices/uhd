//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2023 Per Vices Corporation
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
#include <format>
#include <stdint.h>
#include <iostream>
#include <csignal>
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include <filesystem>
#include <atomic>

#include <uhd/types/tune_request.hpp>

namespace po = boost::program_options;

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    // TODO: create variables

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("rx_sob" , po::value<double >(&rx_sob)->default_value(5.0), "(Seconds) Rx channels will stream after this time. If any trigger is set to input RX will stream after this time AND according to trigger settings.")
        ("tx_sob" , po::value<double >(&tx_sob)->default_value(5.0), "(Seconds) Tx channels will stream after this time. If any trigger is set to input TX will stream after this time AND according to trigger settings.")
        ("rate", po::value<double>(&rate), "rate of incomming/outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("tx_gain", po::value<double>(&tx_gain), "gain for the tx RF chain")
        ("rx_gain", po::value<double>(&rx_gain), "gain for the rx RF chain")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("CONST"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("global_trigpol", po::value<std::string>(&global_trigpol)->default_value("positive"), "Whether the global trigger is active high or low. Valid values: positive, negative")
        ("rx_sma_mode", po::value<std::string>(&rx_sma_mode_s)->default_value("edge"), "A list per channel for whether the sma trigger activates on edge or level.")
        ("tx_sma_mode", po::value<std::string>(&rx_sma_mode_s)->default_value("edge"), "A list per channel for whether the sma trigger activates on edge or level.")
        ("rx_ufl_mode", po::value<std::string>(&rx_ufl_mode_s)->default_value("edge"), "A list per channel for whether the ufl trigger activates on edge or level.")
        ("tx_ufl_mode", po::value<std::string>(&rx_ufl_mode_s)->default_value("edge"), "A list per channel for whether the ufl trigger activates on edge or level.")
        ("rx_sma_sel", po::value<std::string>(&rx_sma_sel_s)->default_value("input"), "A list per channel for whether the the channel is inputting or outputing to the trigger. A single channel can only do one of input or output on the sma at a time regardless of whether the device support have separate input and output pins. Valid values: input, output, none")
        ("tx_sma_sel", po::value<std::string>(&tx_sma_sel_s)->default_value("input"), "A list per channel for whether the the channel is inputting or outputing to the trigger. A single channel can only do one of input or output on the sma at a time regardless of whether the device support have separate input and output pins. Valid values: input, output, none")
        ("rx_ufl_sel", po::value<std::string>(&rx_ufl_sel_s)->default_value("none"), "A list per channel for whether the the channel is inputting or outputing to the trigger. Valid values: input, output, none")
        ("tx_ufl_sel", po::value<std::string>(&tx_ufl_sel_s)->default_value("none"), "A list per channel for whether the the channel is inputting or outputing to the trigger. Valid values: input, output, none")
        ("rx_edge_backoff", po::value<std::string>(&rx_edge_backoff_s)->default_value("0"), "A list per channel for how long in ticks edges should be ignored after receiving an edge. Does nothing for triggers in level mode.")
        ("tx_edge_backoff", po::value<std::string>(&tx_edge_backoff_s)->default_value("0"), "A list per channel for how long in ticks edges should be ignored after receiving an edge. Does nothing for triggers in level mode.")
        ("rx_edge_sample_num", po::value<std::string>(&rx_edge_sample_num)->default_value("0"), "A list per channel for how long in samples after edges an input trigger is considered active. Streaming only occurs when all input triggers and time commands say to stream.")
        ("tx_edge_sample_num", po::value<std::string>(&tx_edge_sample_num)->default_value("0"), "A list per channel for how long in samples after edges an input trigger is considered active. Streaming only occurs when all input triggers and time commands say to stream.")
        ("sma_edge_output", po::value<std::string>(&sma_edge_output)->default_value("0"), "A list per channel for how long in samples after a channel starts streaming the trigger is considered active in edge mode. Each individual channel set to sma output will be the output high for sma_edge_output samples. 0 stays active forever. All the individual channel outputs are ORed together.")
        ("rx_ufl_edge_output", po::value<std::string>(&rx_ufl_edge_output)->default_value("0"), "A list per channel for how long in samples after a channel starts streaming is considered active in edge mode. 0 stays active forever.")
        ("tx_ufl_edge_output", po::value<std::string>(&tx_ufl_edge_output)->default_value("0"), "A list per channel for how long in samples after a channel starts streaming is considered active in edge mode. 0 stays active forever.")


        ("num_trigger", po::value<uint64_t>(&num_trigger)->default_value(0), "Number of trigger event results to record. Set to 0 for continuous")
        ("results_dir", po::value<std::string>(&results_directory)->default_value("results"), "Directory to save results into")
        ("tx_only", "Do not use rx")
        ("rx_only", "Do not use tx")
        ("overwrite", "Overwrite the results files every trigger, instead of creating a new one each time")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << "UHD TX/RX Advanced Trigger" << desc << std::endl;
        return ~0;
    }

    // TODO: parse the arguments

    // TODO: Apply frequencies

    // TODO: Apply gains

    // TODO: Apply rates

    // TODO: Configure trigger

    // TODO: Create rx and tx threads. Give each channel it's own streamer to avoid conflicts in the number of samples

    // TODO: cleanup trigger


    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
