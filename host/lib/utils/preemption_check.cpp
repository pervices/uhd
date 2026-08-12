//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/preemption_check.hpp>
#include <uhd/utils/log.hpp>
#include <cstdio>
#include <string.h>

uhd::preempt_mode_t uhd::check_preemption(std::string log_header) {
    // The paths to use to check preemption mode
    std::string debug_fs_path = "/sys/kernel/debug/sched/preempt";
    std::string mirror_path = "/run/sched_preempt_status";
    FILE *file;


    file = fopen(mirror_path.c_str(), "r");
    // The user has not created a process to mirror debug_fs_path to mirror_path
    // Fallback to attempting to read debugfs directly
    if(file == NULL) {
        file = fopen(debug_fs_path.c_str(), "r");
    }

    // Unable to check preemption mode
    if(file == NULL) {
        UHD_LOG_WARNING(log_header,
            "Unable to check preemption setting. Preemption modes other than none, voluntary, and lazy may cause performance issues.\n"
            "UHD attempts to check \"" + mirror_path + "\" and falls back to \"" + debug_fs_path + "\" to get the current preemption setting.\n"
                "\tVerify debugfs is mounted at /sys/kernel/debug . This is the default on most distros\n"
                "\tIf UHD was installed through Per Vices packages:\n"
                    "\t\tVerify the systemd units preempt-monitor.path and preempt-monitor.service are working properly.\n"
                "\tIf UHD was compiled manually:\n"
                    "\t\tInstall and enable the systemd units: https://github.com/pervices/uhd/raw/refs/heads/master/host/cmake/common_packaging/preempt-monitor.path and https://github.com/pervices/uhd/raw/refs/heads/master/host/cmake/common_packaging/preempt-monitor.service\n"
                "\tAlternative (unsecure) action: mount/remount debugfs to give read access: \"sudo mount -o remount,mode=0755 -t debugfs none /sys/kernel/debug/\" . Remount is required to update permissions."
        );

        return PREEMPT_DYNAMIC_UNDEFINED;
    }

    char buffer[25];
    char* r = fgets(buffer, 25, file);
    std::string value;
    if(r != nullptr) {
        value = std::string(buffer);
    } else {
        value = "";
    }

    uhd::preempt_mode_t mode;

    // Parse the string for the current preemption mode
    if(value.find("(none)") != std::string::npos) {
        mode = PREEMPT_DYNAMIC_NONE;
    } else if(value.find("(voluntary)") != std::string::npos) {
        mode = PREEMPT_DYNAMIC_VOLUNTARY;
    } else if(value.find("(full)")  != std::string::npos) {
        mode = PREEMPT_DYNAMIC_FULL;
    } else if(value.find("(lazy)") != std::string::npos) {
        mode = PREEMPT_DYNAMIC_LAZY;
    } else {
        mode = PREEMPT_DYNAMIC_UNDEFINED;
    }

    if(mode == PREEMPT_DYNAMIC_NONE || mode == PREEMPT_DYNAMIC_VOLUNTARY || mode == PREEMPT_DYNAMIC_LAZY) {
        // None, voluntary, and lazy are all good preemption modes. No action required
        return mode;

    } else if(mode == PREEMPT_DYNAMIC_FULL) {
        // Warn the user they are using a suboptimal preemption mode
        UHD_LOG_WARNING(log_header,
            "Preemption is currently set to full; this will cause unreliable performance.\n"
                "\tTo set the preemption mode permanently follow the instructions at: https://support.pervices.com/how-to/pvht-11-performancetuning/#setting-default-preemption-mode .\n"
                "\tTo set the preemption mode temporarily mount debugfs to /sys/kernel/debug/ (if not already done by default) "
                "and echo none (kernel < 7.0) or lazy (kernel >= 7.0) to /sys/kernel/debug/sched/preempt as root.\n"
                "\tChanging the preemption mode must be done as root. sudo alone will not work."
        );
        return mode;

    } else /*PREEMPT_DYNAMIC_UNDEFINED*/ {
        // Warn the user we were unable to detect a preemption mode
        // This should never happen unless a new mode is added
        UHD_LOG_WARNING(log_header,
            "Unrecognized preemption mode; this may cause unreliable performance.\n"
                "\tTo set the preemption mode permanently follow the instructions at: https://support.pervices.com/how-to/pvht-11-performancetuning/#setting-default-preemption-mode .\n"
                "\tTo set the preemption mode temporarily mount debugfs to /sys/kernel/debug/ (if not already done by default) "
                "and echo none (kernel < 7.0) or lazy (kernel >= 7.0) to /sys/kernel/debug/sched/preempt as root.\n"
                "\tChanging the preemption mode must be done as root. sudo alone will not work."
        );
        return PREEMPT_DYNAMIC_UNDEFINED;
    }
}
