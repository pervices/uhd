//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
#include <string>
#include <cstdint>

namespace uhd {
    /**
    * Gets the network interface with the specified IP.
    * @param ipv4 The ipv4 address to use.
    * @return The network interface used by the address.
    */
    std::string get_dev_from_ipv4(std::string ipv4);

    /**
     * Gets the current size of the rx ring buffer for a given interface.
     * @param interface The device who's ring buffer size to check.
     * @return The current size of the rx ring buffer
     */
    uint32_t get_rx_ring_buffer_size(std::string interface);

    /**
     * Gets the maximum size of the rx ring buffer for a given interface.
     * @param interface The device who's ring buffer size to check.
     * @return The maximum size of the rx ring buffer
     */
    uint32_t get_rx_ring_buffer_max_size(std::string interface);
}
