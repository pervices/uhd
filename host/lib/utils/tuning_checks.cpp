//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/tuning_checks.hpp>

#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>

#include <numa.h>
#include <cerrno>
#include <cstring>
#include <string>

int uhd::check_numa(const cpu_set_t affinity_mask, int socket_fd[], size_t socket_fd_len) {

    // Check if the kernel supports NUMA
    if(numa_available() == -1) [[unlikely]] {
        // TODO: add a parameter to take a context specific error message
        // TODO: decide which level of message to make this message
        UHD_LOG_WARNING("CHECK_NUMA", "The kernel is not configured with support for NUMA. Automatic checks for optimal NUMA configuartion will not work.");
        return -1;
    }

    // This system only has one NUMA node, and incorrect configuration is impossible
    if(numa_num_configured_nodes() == 1) {
        return 0;
    }

    // The numa node allowed by affinity_mask
    int numa_node = -1;

    for(int cpu = 0; cpu < CPU_SETSIZE; cpu++) {
        // Skip CPUs not allowed by the mask
        if(!CPU_ISSET(cpu, &affinity_mask)) {
            continue;
        }

        // Get the numa node of the CPU
        int cpu_numa_node = numa_node_of_cpu(cpu);
        if(cpu_numa_node < 0) [[unlikely]] {
            std::string get_numa_error_message = "Unable to get NUMA node for CPU " + std::to_string(cpu) + ". numa_node_of_cpu failed with error code: " + std::string(strerror(errno));

            // TODO: add a parameter to take a context specific error message
            UHD_LOG_WARNING("CHECK_NUMA", get_numa_error_message);
            throw uhd::system_error(get_numa_error_message);
        }

        // Record the NUMA node the of the first CPU in the mask
        if(numa_node == -1) {
            numa_node = cpu_numa_node;
        }
        // Check if later CPUs have the same NUMA node
        else if(cpu_numa_node != numa_node) {
            // TODO: add a parameter to take a context specific error message
            UHD_LOG_WARNING("CHECK_NUMA", "The affinity mask of the tested thread can run on multiple NUMA nodes");
            return -1;
        }
    }

    // TODO: get the network interfaces of socket_fd
    std::vector<std::string> network_interfaces;


    // TODO: get the numa node of said interfaces

    // TODO: warn the user and return 1 if the interfaces are on different nodes

    // TODO: check if mask's node matches that of the sockets
}