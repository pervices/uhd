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

namespace po = boost::program_options;

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, ref;
    size_t mboard;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("mboard", po::value<size_t>(&mboard)->default_value(0), "Which mainboard to set the clock ref for")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("set_clk_reference %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This application attempts to set the clk reference, then attempts to read it back.\n"
            << std::endl;
        return ~0;
    }

    std::cout << boost::format("Starting: %s") % argv[0] << std::endl;

    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //Sets the clock source
    std::cout << boost::format("Setting clock source to: %s") % ref << std::endl;
    usrp->set_clock_source(ref, mboard);

    std::string actual_ref = usrp->get_clock_source(mboard);

    std::cout << boost::format("The clock source is %s") % actual_ref << std::endl;

    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
