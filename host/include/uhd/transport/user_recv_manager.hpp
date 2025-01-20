// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <liburing.h>
#include <uhd/utils/log.hpp>

#include <immintrin.h>

#include <uhd/transport/async_recv_manager.hpp>

namespace uhd { namespace transport {

// Manages asynchronous receives using normal network calls such as recvmmsg
class user_recv_manager : public async_recv_manager {

public:

    /**
     * Constructor for user_recv_manager.
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    user_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    ~user_recv_manager();

    /**
     * Calls constructor for user_recv_manager and ensure user_recv_manager is properly aligned.
     * You must call unmake when done
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    static user_recv_manager* make( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    /**
     * Destructs and frees an user_recv_manager
     * @param user_recv_manager* The instance to destruct and free
     */
    static void unmake( user_recv_manager* recv_manager );

    /**
     * Lets liburing know that packets have been consumed
     * @param ch The channel whose packets to mark as clear
     * @param n The number of packets to mark as clear
    */
    inline __attribute__((always_inline)) void clear_packets(const size_t ch, const unsigned n) {
        // TODO: implement
    }


private:

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(user_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
