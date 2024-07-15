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
