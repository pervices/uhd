//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/tuning_checks.hpp>

#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>
#include <uhdlib/utils/network_config.hpp>

#include <numa.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

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

    // Get the network interface used by each socket
    std::vector<std::string> network_interfaces;
    network_interfaces.reserve(socket_fd_len);

    for(size_t n = 0; n < socket_fd_len; n++) {
        // Gets the local address the socket is bound to (either explicitly via bind(),
        // or implicitly assigned by the kernel when connect() selected a route)
        struct sockaddr_in local_addr;
        socklen_t addr_len = sizeof(local_addr);
        if(getsockname(socket_fd[n], (struct sockaddr*)&local_addr, &addr_len) < 0) [[unlikely]] {
            std::string get_sockname_error_message = "Unable to get local address for socket " + std::to_string(socket_fd[n]) + ". getsockname failed with error code: " + std::string(strerror(errno));

            // TODO: add a parameter to take a context specific error message
            UHD_LOG_WARNING("CHECK_NUMA", get_sockname_error_message);
            throw uhd::system_error(get_sockname_error_message);
        }

        char ip_buff[INET_ADDRSTRLEN];
        if(inet_ntop(AF_INET, &local_addr.sin_addr, ip_buff, INET_ADDRSTRLEN) == nullptr) [[unlikely]] {
            std::string inet_ntop_error_message = "Unable to convert address for socket " + std::to_string(socket_fd[n]) + " to a string. inet_ntop failed with error code: " + std::string(strerror(errno));

            // TODO: add a parameter to take a context specific error message
            UHD_LOG_WARNING("CHECK_NUMA", inet_ntop_error_message);
            throw uhd::system_error(inet_ntop_error_message);
        }

        // Get the device used by the ip.
        // get_dev_from_ipv4 will throw errors if it fails
        std::string interface = uhd::get_dev_from_ipv4(std::string(ip_buff));

        network_interfaces.push_back(interface);
    }

    // TODO: get the numa node of said interfaces

    // TODO: warn the user and return 1 if the interfaces are on different nodes

    // TODO: check if mask's node matches that of the sockets
}