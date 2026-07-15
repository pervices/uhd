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
    std::string path = "/sys/kernel/debug/sched/preempt";

    FILE *file;

    file = fopen(path.c_str(), "r");

    std::string debugfs_mount = "To mount debugfs run \"sudo mount -t debugfs none /sys/kernel/debug/\"";
    std::string update_debugfs_permissions = "To set the permission of debugfs to allow non root users to read preemption setting run \"sudo mount -o remount,mode=0755 -t debugfs none /sys/kernel/debug/\". \"remount\" is required due to a bug affecting most kernel 6. versions.";
    std::string read_preempt = "To check current preemption setting run \"cat " + path + "\". It must be set to none or voluntary for optimal performance.";
    // Discussion of the kernel bug requiring remount: https://bugzilla.kernel.org/show_bug.cgi?id=220406
    std::string set_preempt = "To change preemption you must echo none or voluntary (for kernels < 6.13) or lazy (for kernels >= 6.13) to " + path + " as root (sudo will not work).";

    // Unable to check preempt setting
    if(file == NULL) {
        // Insufficient permission
        if(errno == EACCES) {
            UHD_LOG_WARNING(log_header, "Insufficient permission to check preemption setting.\n\t" + update_debugfs_permissions + "\n\t" + read_preempt + "\n\t" + set_preempt);

            return PREEMPT_DYNAMIC_UNDEFINED;
            // File missing (probably because debugfs isn't mounted)
        } else if (errno == ENOENT) {
            UHD_LOG_WARNING(log_header, "debugfs is not mounted or not mounted in it's usual location, unable to check preemption setting.\n\t" + debugfs_mount + "\n\t" + update_debugfs_permissions + "\n\t" + read_preempt + "\n\t" + set_preempt);
            return PREEMPT_DYNAMIC_UNDEFINED;
            // Unexpected error when attempting to open file
        } else {
            UHD_LOG_WARNING(log_header, "Preemption check failed with error code: " + std::to_string(errno) + ": " + std::string(strerror(errno)) + "\n\t" + debugfs_mount + "\n\t" + update_debugfs_permissions + "\n\t" + read_preempt + "\n\t" + set_preempt);
        return PREEMPT_DYNAMIC_UNDEFINED;
        }
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
    if(value.find("(none)") == std::string::npos) {
        mode = PREEMPT_DYNAMIC_NONE;
    } else if(value.find("(voluntary)") == std::string::npos) {
        mode = PREEMPT_DYNAMIC_VOLUNTARY;
    } else if(value.find("(full)")) {
        mode = PREEMPT_DYNAMIC_FULL;
    } else if(value.find("(lazy)")) {
        mode = PREEMPT_DYNAMIC_LAZY;
    } else {
        mode = PREEMPT_DYNAMIC_UNDEFINED;
    }

    if(mode == PREEMPT_DYNAMIC_NONE || mode == PREEMPT_DYNAMIC_VOLUNTARY || mode == PREEMPT_DYNAMIC_LAZY) {
        // None, voluntary, and lazy are all good preemption modes. No action required
        return mode;

    } else if(mode == PREEMPT_DYNAMIC_FULL) {
        // Warn the user they are using a suboptimal preemption mode
        UHD_LOG_WARNING(log_header, "Preemption is currently set to full, this will cause unreliable performance.\n\t" + read_preempt + "\n\t" + set_preempt);
        return mode;

    } else /*PREEMPT_DYNAMIC_UNDEFINED*/ {
        // Warn the user we were unable to detect a preemption mode
        // This should never happen if a new mode is added
        UHD_LOG_WARNING(log_header, "Unrecognized preemption mode, this will cause unreliable performance.\n\t" + read_preempt + "\n\t" + set_preempt);
        return PREEMPT_DYNAMIC_UNDEFINED;
    }
}
