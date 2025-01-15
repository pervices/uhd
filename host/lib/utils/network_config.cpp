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

// TODO: remove whichever of these 5 are not needed
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/netlink.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

// TODO: find include for this
#ifndef SIOCETHTOOL
#define SIOCETHTOOL     0x8946
#endif

std::string uhd::get_dev_from_ipv4(std::string ipv4) {
    // Start of a linked list containing info for each network interface
    struct ifaddrs *ifaddr_start;

    // Gets a linked list of information for all network interfaces
    int r = getifaddrs(&ifaddr_start);
    if(r == -1) {
        UHD_LOG_ERROR("NETWORK_CONFIG", "Unable to get device from ip. \"getifaddrs\" failed with error code: " + std::string(strerror(errno)));
        throw uhd::system_error("getifaddrs: " + std::string(strerror(errno)));
    }

    // Cycle through every element of the interface list
    // Current position on the list
    struct ifaddrs *ifaddr_head = ifaddr_start;
    while (ifaddr_head != NULL) {

        if(ifaddr_head->ifa_addr->sa_family != AF_INET) {
            //Skip non IPV4 addresses
        } else {
            // Buffer to store ip
            char ip_buff[INET_ADDRSTRLEN];
            struct sockaddr_in *sa = (struct sockaddr_in *) ifaddr_head->ifa_addr;
            // Extract ip
            const char* ip_buffer_r = inet_ntop(AF_INET, &sa->sin_addr, ip_buff, INET_ADDRSTRLEN);

            // Check if the ip address was extracted successfully. It should be impossible to this to fail
            if(ip_buffer_r != ip_buff) {
                freeifaddrs(ifaddr_start);
                throw uhd::runtime_error( "Failed to parse ipv4 address for device: " + std::string(ifaddr_head->ifa_name));
            }

            if(std::string(ip_buff) == ipv4) {
                freeifaddrs(ifaddr_start);
                return std::string(ifaddr_head->ifa_name);
            }
        }

        // Advance to the next element in the linked list
        ifaddr_head = ifaddr_head->ifa_next;
    }

    freeifaddrs(ifaddr_start);
    std::string not_found_error_message = "Unable to find device with ip " + ipv4 + ". Verify said ip is assigned to a network interface";
    UHD_LOG_ERROR("NETWORK_CONFIG", not_found_error_message);
    throw uhd::io_error( not_found_error_message );
}

// TODO: comment this function
static struct ethtool_ringparam get_ethtool_ringparam(std::string interface) {
    // Create a socket for use by ioctl
    int ioctl_fd = socket(AF_INET, SOCK_DGRAM, 0);
    // TODO: add error check for ioctl_fd

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    struct ethtool_ringparam ering;
    memset(&ering, 0, sizeof(ering));

    strcpy(ifr.ifr_name, interface.c_str());

    ifr.ifr_data = &ering;
    ering.cmd = ETHTOOL_GRINGPARAM;

    int r = ioctl(ioctl_fd, SIOCETHTOOL, &ifr);
    if(r == -1) {
        // TODO: add error check
        printf("ioctl error\n");
    }

    return ering;
}

uint32_t uhd::get_rx_ring_buffer_size(std::string interface) {
    // Get info about the ethernet interface
    struct ethtool_ringparam eth_info = get_ethtool_ringparam(interface);

    return eth_info.rx_pending;
}

uint32_t uhd::get_rx_ring_buffer_max_size(std::string interface) {
    // Get info about the ethernet interface
    struct ethtool_ringparam eth_info = get_ethtool_ringparam(interface);

    return eth_info.rx_max_pending;
}
