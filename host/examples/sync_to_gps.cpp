//
// Copyright 2016 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <format>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <thread>

namespace po = boost::program_options;

void print_notes(void)
{
    // Helpful notes
    std::cout <<
        "**************************************Helpful Notes on Clock/PPS "
        "Selection**************************************\n";
    std::cout << "As you can see, the default 10 MHz Reference and 1 PPS "
                               "signals are now from the GPSDO.\n";
    std::cout <<
        "If you would like to use the internal reference(TCXO) in other applications, "
        "you must configure that explicitly.\n";
    std::cout <<
        "You can no longer select the external SMAs for 10 MHz or 1 PPS signaling.\n";
    std::cout <<
        "********************************************************************************"
        "********************************\n";
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args;

    // Set up program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
    ("help", "help message")
    ("args", po::value<std::string>(&args)->default_value(""), "USRP device arguments")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Print the help message
    if (vm.count("help")) {
        std::cout << "Synchronize USRP to GPS " << desc << std::endl;
        return EXIT_FAILURE;
    }

    // Create a USRP device
    std::cout << std::format("\nCreating the USRP device with: {}...\n", args);
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << std::format("Using Device: {}\n", usrp->get_pp_string());

    try {
        size_t num_mboards    = usrp->get_num_mboards();
        size_t num_gps_locked = 0;
        for (size_t mboard = 0; mboard < num_mboards; mboard++) {
            std::cout << "Synchronizing mboard " << mboard << ": "
                      << usrp->get_mboard_name(mboard) << std::endl;

            // Set references to GPSDO
            usrp->set_clock_source("gpsdo", mboard);
            usrp->set_time_source("gpsdo", mboard);

            std::cout << std::endl;
            print_notes();
            std::cout << std::endl;

            // Check for 10 MHz lock
            std::vector<std::string> sensor_names = usrp->get_mboard_sensor_names(mboard);
            if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end()) {
                std::cout << "Waiting for reference lock..." << std::flush;
                bool ref_locked = false;
                for (int i = 0; i < 30 and not ref_locked; i++) {
                    ref_locked = usrp->get_mboard_sensor("ref_locked", mboard).to_bool();
                    if (not ref_locked) {
                        std::cout << "." << std::flush;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }
                if (ref_locked) {
                    std::cout << "LOCKED" << std::endl;
                } else {
                    std::cout << "FAILED" << std::endl;
                    std::cout << "Failed to lock to GPSDO 10 MHz Reference. Exiting."
                              << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                std::cout <<
                    "ref_locked sensor not present on this board.\n";
            }

            // Wait for GPS lock
            bool gps_locked = usrp->get_mboard_sensor("gps_locked", mboard).to_bool();
            if (gps_locked) {
                num_gps_locked++;
                std::cout << "GPS Locked\n";
            } else {
                std::cerr
                    << "WARNING:  GPS not locked - time will not be accurate until locked"
                    << std::endl;
            }

            // Set to GPS time
            uhd::time_spec_t gps_time = uhd::time_spec_t(
                int64_t(usrp->get_mboard_sensor("gps_time", mboard).to_int()));
            usrp->set_time_next_pps(gps_time + 1.0, mboard);

            // Wait for it to apply
            // The wait is 2 seconds because N-Series has a known issue where
            // the time at the last PPS does not properly update at the PPS edge
            // when the time is actually set.
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Check times
            gps_time = uhd::time_spec_t(
                int64_t(usrp->get_mboard_sensor("gps_time", mboard).to_int()));
            uhd::time_spec_t time_last_pps = usrp->get_time_last_pps(mboard);

            std::cout << std::format("USRP time: {:.9f}\n", time_last_pps.get_real_secs());
            std::cout << std::format("GPSDO time: {:.9f}\n", gps_time.get_real_secs());

            if (gps_time.get_real_secs() == time_last_pps.get_real_secs())
                std::cout << std::endl
                          << "SUCCESS: USRP time synchronized to GPS time" << std::endl
                          << std::endl;
            else
                std::cerr << std::endl
                          << "ERROR: Failed to synchronize USRP time to GPS time"
                          << std::endl
                          << std::endl;
        }

        if (num_gps_locked == num_mboards and num_mboards > 1) {
            // Check to see if all USRP times are aligned
            // First, wait for PPS.
            uhd::time_spec_t time_last_pps = usrp->get_time_last_pps();
            while (time_last_pps == usrp->get_time_last_pps()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // Sleep a little to make sure all devices have seen a PPS edge
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Compare times across all mboards
            bool all_matched              = true;
            uhd::time_spec_t mboard0_time = usrp->get_time_last_pps(0);
            for (size_t mboard = 1; mboard < num_mboards; mboard++) {
                uhd::time_spec_t mboard_time = usrp->get_time_last_pps(mboard);
                if (mboard_time != mboard0_time) {
                    all_matched = false;
                    std::cerr << std::format("ERROR: Times are not aligned: USRP 0={:.9f}, USRP {}={:.9f}\n",
                            mboard0_time.get_real_secs(),
                            mboard,
                            mboard_time.get_real_secs()
                    );
                }
            }
            if (all_matched) {
                std::cout << "SUCCESS: USRP times aligned" << std::endl << std::endl;
            } else {
                std::cout << "ERROR: USRP times are not aligned" << std::endl
                          << std::endl;
            }
        }
    } catch (std::exception& e) {
        std::cout << std::format("\nError: {}", e.what());
        std::cout <<
            "This could mean that you have not installed the GPSDO correctly.\n\n";
        std::cout << "Visit one of these pages if the problem persists:\n";
        std::cout <<
            " * N2X0/E1X0: http://files.ettus.com/manual/page_gpsdo.html";
        std::cout <<
            " * X3X0: http://files.ettus.com/manual/page_gpsdo_x3x0.html\n\n";
        std::cout <<
            " * E3X0: http://files.ettus.com/manual/page_usrp_e3x0.html#e3x0_hw_gps\n\n";
        exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}
