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

// Architecture:
// For each channel:
// In one thread that recvmmsg a non blocking recv to fill a call buffer
// Once the call buffer has any  packets in it mark how many packets are in it and move on to the next call buffer

class user_recv_manager : public async_recv_manager {

private:

    // Number of call buffers per ch
    // Must be a power of 2 and a constexpr, for some reason having it non constexpr will result in random lag spikes (but only on some runs)
    static constexpr size_t NUM_CALL_BUFFERS = 1024;

    static_assert(PACKET_BUFFER_SIZE % NUM_CALL_BUFFERS == 0, "The packet buffer must be able to contain a whole number of call buffers");

    // Number of packets per call buffer
    static constexpr size_t CALL_BUFFER_SIZE = PACKET_BUFFER_SIZE / NUM_CALL_BUFFERS;

    // Buffer containing mmsghdrs
    // Format: (((mmsghdr * CALL_BUFFER_SIZE) + padding to cache line) * NUM_CALL_BUFFERS) * _num_ch
    // ((mmsghdr * CALL_BUFFER_SIZE) + padding to cache line) == MMGHDR_CALL_BUFFER_SIZE
    static constexpr size_t MMSGHDR_CALL_BUFFER_SIZE = std::ceil(sizeof(mmsghdr) * CALL_BUFFER_SIZE / (double) CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    static constexpr size_t MMSGHDR_CH_BUFFER_SIZE = MMSGHDR_CALL_BUFFER_SIZE * NUM_CALL_BUFFERS;
    inline __attribute__((always_inline)) size_t mmghdr_buffer_size() {
        return MMSGHDR_CH_BUFFER_SIZE * _num_ch;
    }
    uint8_t* const _mmsghdr_buffer;

    // Gets a pointer to specific mmsghdr
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: call buffer
    // p: packet within call buffer
    inline __attribute__((always_inline)) mmsghdr* access_mmsghdr(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return (mmsghdr*) (_mmsghdr_buffer + ((ch + ch_offset) * MMSGHDR_CH_BUFFER_SIZE) + (b * MMSGHDR_CALL_BUFFER_SIZE) + (p * sizeof(mmsghdr)));
    }

    // Buffer containing iovecs
    // Format: (((iovec * CALL_BUFFER_SIZE) + padding to cache line) * NUM_CALL_BUFFERS) * _num_ch
    // ((IOVEC * CALL_BUFFER_SIZE) + padding to cache line) == IOVEC_CALL_BUFFER_SIZE
    // TODO: confirm only 1 element in each iovec
    static constexpr size_t IOVEC_CALL_BUFFER_SIZE = std::ceil(sizeof(mmsghdr) * CALL_BUFFER_SIZE / (double) CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    static constexpr size_t IOVEC_CH_BUFFER_SIZE = IOVEC_CALL_BUFFER_SIZE * NUM_CALL_BUFFERS;
    inline __attribute__((always_inline)) size_t iovec_buffer_size() {
        return IOVEC_CH_BUFFER_SIZE * _num_ch;
    }
    uint8_t* const _iovec_buffer;

    // Gets a pointer to specific mmsghdr
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: call buffer
    // p: packet within call buffer
    inline __attribute__((always_inline)) iovec* access_iovec(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return (iovec*) (_iovec_buffer + ((ch + ch_offset) * IOVEC_CH_BUFFER_SIZE) + (b * IOVEC_CALL_BUFFER_SIZE) + (p * sizeof(iovec)));
    }

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
     * Gets information needed to process the next packet.
     * The caller is responsible for ensuring correct fencing
     * @param ch
     * @return If a packet is ready it returns a struct containing the packet length and pointers to the Vita header and samples. If the packet is not ready the struct will contain 0 for the length and nullptr for the Vita header and samples
     */
    void get_next_async_packet_info(const size_t ch, async_packet_info* info) override;

    /**
     * Lets liburing know that packets have been consumed
     * @param ch The channel whose packets to mark as clear
     * @param n The number of packets to mark as clear
    */
    void clear_packets(const size_t ch, const unsigned n) override;


private:

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(user_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
