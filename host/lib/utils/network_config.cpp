//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/network_config.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/exception.hpp>
#include <errno.h>

#include <sys/types.h>
#include <ifaddrs.h>

std::string uhd::get_dev_from_ipv4(std::string ipv4) {
    struct ifaddrs *ifaddr;

    // Gets a linked list of information for all network interfaces
    int r = getifaddrs(&ifaddr);
    if(r == -1) {
        UHD_LOG_ERROR("NETWORK_CONFIG", "Unable to get device from ip. \"getifaddrs\" failed with error code: " + std::string(strerror(errno)));
        throw uhd::system_error("getifaddrs: " + std::string(strerror(errno)));
    }

    printf("T1\n");
    // Cycle through every element of the interface list
    while (ifaddr != NULL) {
        printf("ifa_name: %s\n", ifaddr->ifa_name);
        printf("ifa_addr.sa_data: %s\n", ifaddr->ifa_addr->sa_data);

        // Advance to the next element in the linked list
        ifaddr = ifaddr->ifa_next;
    }

    // TODO: free list

    return "";
}
