//
// Copyright 2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/performance_mode.hpp>

#include <filesystem>
#include<algorithm>
#include<fstream>
#include<sstream>
#include <uhd/utils/log.hpp>
#include <atomic>

std::vector<std::string> uhd::get_performance_governors(void)
{
    std::vector<std::string> paths_to_check;

    // TODO: simulate cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

    // Get all paths in /sys/devices/system/cpu/cpu* that correspond to a cpu core
    const std::filesystem::path core_info{"/sys/devices/system/cpu/"};
    for (auto const& dir_entry : std::filesystem::directory_iterator{core_info}) {

        std::string path_s = dir_entry.path().string();
        std::string partial_path = "/sys/devices/system/cpu/cpu";

        // CHeck to prevent crashing for paths that are to short
        if(path_s.length() <= partial_path.length()) {
            continue;
            // Verify the path is for a CPU core
        } else if (path_s.find(partial_path) == std::string::npos || path_s[partial_path.size()] < '0' || path_s[partial_path.size()] > '9') {
            continue;
        }

        paths_to_check.push_back(path_s);
    }

    // Sort paths in order of core number
    std::sort(paths_to_check.begin(),paths_to_check.end(), [](std::string &a, std::string &b){ return (a.size() < b.size()) || (a.size() == b.size() && a.compare(b) < 0); });

    std::vector<std::string> ret(paths_to_check.size());
    for(size_t n = 0; n < paths_to_check.size(); n++) {

        std::ifstream file(paths_to_check[n] + "/cpufreq/scaling_governor");
        if(file) {
            std::ostringstream converter;
            converter << file.rdbuf(); // reading data
            ret[n] = converter.str();
        } else {
            // Return empty vector if unable to check the CPU governor
            return std::vector<std::string>(0);
        }
    }

    return ret;
}

std::atomic<uint8_t> governor_warning_printed = 0;

bool uhd::check_if_only_using_governor(std::string desired_governor) {
    // Check if any core is not set to performance mode, used to decide if an info message should be printed if overflows occur
    bool using_desired_governor = true;
    bool governor_known;
    std::vector<std::string> governors = uhd::get_performance_governors();
    if(governors.size() != 0) {
        governor_known = true;
        for(auto& g : governors) {
            if(g.find(desired_governor) == std::string::npos) {
                using_desired_governor = false;
                break;
            }
        }
    } else {
        governor_known = false;
    }

    // Only print the warning once
    if(!governor_warning_printed) {
        if(!governor_known) {
            UHD_LOG_WARNING("GOVERNOR", "Unable to check CPU governor. Ensure " + desired_governor + " is in use. Performance may be affected");
        } else if(!using_desired_governor) {
            UHD_LOG_WARNING("GOVERNOR", "A CPU governor other than " + desired_governor + " is in use. Performance may be affected");
        }
    }

    return using_desired_governor && governor_known;
}
