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
#include <arpa/inet.h>

std::string uhd::get_dev_from_ipv4(std::string ipv4) {
    struct ifaddrs *ifaddr;

    // Gets a linked list of information for all network interfaces
    int r = getifaddrs(&ifaddr);
    if(r == -1) {
        UHD_LOG_ERROR("NETWORK_CONFIG", "Unable to get device from ip. \"getifaddrs\" failed with error code: " + std::string(strerror(errno)));
        throw uhd::system_error("getifaddrs: " + std::string(strerror(errno)));
    }

    // Cycle through every element of the interface list
    while (ifaddr != NULL) {

        if(ifaddr->ifa_addr->sa_family != AF_INET) {
            //Skip non IPV4 addresses
        } else {
            // Buffer to store ip
            char ip_buff[INET_ADDRSTRLEN];
            struct sockaddr_in *sa = (struct sockaddr_in *) ifaddr->ifa_addr;
            // Extract ip
            const char* ip_buffer_r = inet_ntop(AF_INET, &sa->sin_addr, ip_buff, INET_ADDRSTRLEN);

            // Check if the ip address was extracted successfully. It should be impossible to this to fail
            if(ip_buffer_r != ip_buff) {
                throw uhd::runtime_error( "Failed to parse ipv4 address for device: " + std::string(ifaddr->ifa_name));
            }

            printf("ifa_name: %s\n", ifaddr->ifa_name);
            printf("ip_buff: %s\n", ip_buff);
        }


        // Advance to the next element in the linked list
        ifaddr = ifaddr->ifa_next;
    }

    // TODO: free list

    return "";
}
