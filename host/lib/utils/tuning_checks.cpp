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
#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// Return code used by mask_to_numa_node to indicate that the mask spans multiple nodes
static constexpr int MASK_SPANS_MULTIPLE_NODES = INT_MIN;

// Get the NUMA node allowed by the mask
// Return -errno of numa_node_of_cpu if unable to get the node
// Return MASK_SPANS_MULTIPLE_NODES if the mask spans multiple nodes
static int mask_to_numa_node(const cpu_set_t affinity_mask) {

    // The NUMA node used by every CPU allowed by the mask
    int candidate_numa_node = -1;

    for(int cpu = 0; cpu < CPU_SETSIZE; cpu++) {
        // Skip CPUs not allowed by the mask
        if(!CPU_ISSET(cpu, &affinity_mask)) {
            continue;
        }

        // Get the numa node of the CPU
        int cpu_numa_node = numa_node_of_cpu(cpu);
        if(cpu_numa_node < 0) [[unlikely]] {
            int error_code = errno;
            std::string get_numa_error_message = "Unable to get NUMA node for CPU " + std::to_string(cpu) + ". numa_node_of_cpu failed with error code: " + std::string(strerror(error_code));

            // TODO: add a parameter to take a context specific error message
            UHD_LOG_WARNING("CHECK_NUMA", get_numa_error_message);
            return -errno;
        }

        // Record the NUMA node the of the first CPU in the mask
        if(candidate_numa_node == -1) {
            candidate_numa_node = cpu_numa_node;
        }
        // Check if later CPUs have the same NUMA node
        else if(candidate_numa_node != cpu_numa_node) {
            UHD_LOG_WARNING("CHECK_NUMA", "The affinity mask of the tested thread can run on multiple NUMA nodes");
            return MASK_SPANS_MULTIPLE_NODES;
        }
    }

    return candidate_numa_node;
}

// Gets the NUMA node a network interface is attached to by reading it from sysfs.
// libnuma has no API for this since it's a PCI/device concept, not a CPU/memory one.
static int get_numa_node_for_iface(std::string iface) {
    std::ifstream file("/sys/class/net/" + iface + "/device/numa_node");
    if(!file) [[unlikely]] {
        // No "device" symlink for this interface. Expected for interfaces with no
        // determinable NUMA affinity (e.g. loopback/virtual interfaces, USB NICs,
        // devices behind a non-NUMA-aware bus, or passthrough NICs in a VM), not
        // just an error case.
        // TODO: -1 means no known NUMA affinity for this interface. The comparison
        // logic being added for the following TODOs needs to treat -1 as "no
        // restriction" rather than a mismatch, mirroring how dpdk_common.cpp treats
        // DPDK's SOCKET_ID_ANY.
        UHD_LOG_WARNING("CHECK_NUMA", "Unable to determine NUMA node for interface " + iface);
        return -1;
    }

    std::string numa_node_str;
    std::getline(file, numa_node_str);

    try {
        // The file may itself contain -1 for devices with no determinable NUMA
        // affinity. See TODO above.
        return std::stoi(numa_node_str);
    } catch(const std::exception&) {
        UHD_LOG_WARNING("CHECK_NUMA", "Unable to parse NUMA node for interface " + iface + " from value: " + numa_node_str);
        return -1;
    }
}

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

    // Find NUMA node allowed by affinity_mask
    int numa_node = mask_to_numa_node(affinity_mask);

    if(numa_node == MASK_SPANS_MULTIPLE_NODES) {
        // The mask spans multiple nodes, return a positive value
        return 1;
    } else if(numa_node < 0) {
        // Unable to get the NUMA node enabled by the mask
        // Return a negative value to indicate a failure with the test itself
        return -1;
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

    // Remove duplicate network interfaces since multiple sockets can use the same interface
    std::sort(network_interfaces.begin(), network_interfaces.end());
    network_interfaces.erase(
        std::unique(network_interfaces.begin(), network_interfaces.end()),
        network_interfaces.end());

    // Get the NUMA node of each interface
    std::vector<int> interface_numa_nodes;
    interface_numa_nodes.reserve(network_interfaces.size());

    for(const std::string& iface : network_interfaces) {
        interface_numa_nodes.push_back(get_numa_node_for_iface(iface));
    }

    // Return value for a successful check
    // See @return for it's meaning
    int result = 0;
    
    for(size_t n = 0; n < interface_numa_nodes.size(); n++) {
        if(numa_node != interface_numa_nodes[n]) {
            UHD_LOG_WARNING("CHECK_NUMA", "The tests thread is allowed to run on NUMA node " +
                std::to_string(numa_node) + " but it using a socket using interface " + network_interfaces[n] +
                " which is connect to NUMA node " + std::to_string(interface_numa_nodes[n]) + "."
            );

            // The interfaces for not match
            result = 1;
        }
    }

    return result;
}