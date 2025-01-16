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

    /**
     * Converts from a location in the call buffer (such as the mmsghdr buffer) into a location in a buffer not splot into call buffers (such as the main data buffer)
     */
    static inline __attribute__((always_inline)) size_t call_to_consolidated(size_t b, size_t p) {
        return CALL_BUFFER_SIZE * b + p;
    }

    // Stores have many call buffers have been written to for each channel
    // Format: count, padding to next cache line, repeat for every channel
    uint8_t* const _call_buffer_heads;
    inline __attribute__((always_inline)) uint64_t* access_call_buffer_head(size_t ch, size_t ch_offset = 0) {
        return (uint64_t*) (_call_buffer_heads + ((ch + ch_offset) * CACHE_LINE_SIZE));
    }

    // Stores have many call buffers have been marked as clear by the consumer thread
    // Format: count, padding to next cache line, repeat for every channel
    uint8_t* const _call_buffer_tails;
    inline __attribute__((always_inline)) uint64_t* access_call_buffer_tail(size_t ch, size_t ch_offset = 0) {
        return (uint64_t*) (_call_buffer_tails + ((ch + ch_offset) * CACHE_LINE_SIZE));
    }

    // Stores the number of packets in a call buffer
    // Not cleared after number of packets consumed, use the value of access_call_buffer_head to figure out if this is ready
    // Format: (number of packets in a call buffer, padding to next cache line) repeated for every call buffer, repeat for every channel
    uint8_t* const _packets_in_call_buffer;
    inline __attribute__((always_inline)) uint64_t* access_packets_in_call_buffer(size_t ch, size_t ch_offset, size_t b) {
        return (uint64_t*) (_packets_in_call_buffer + ((((ch + ch_offset) * NUM_CALL_BUFFERS) + b) * CACHE_LINE_SIZE));
    }

    std::vector<std::thread> recv_loops;

    // Flag to to the recv loop when to exit
    // Use fences when setting this to ensure it is synced across threads
    uint8_t stop_flag = 0;

    // Number of packets consumed in the current call buffer
    uint8_t* const _num_packets_consumed_current_buffer;
    inline __attribute__((always_inline)) uint64_t* access_num_packets_consumed_current_buffer(size_t ch, size_t ch_offset = 0) {
        return (uint64_t*) (_num_packets_consumed_current_buffer + ((ch + ch_offset) * CACHE_LINE_SIZE));
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

    inline __attribute__((always_inline)) void advance_packet(const size_t ch) override {

        // Record that this packet has been consumed
        uint64_t* num_packets_consumed = access_num_packets_consumed_current_buffer(ch);
        (*num_packets_consumed)++;

        size_t b = (*access_call_buffer_tail(ch)) & (NUM_CALL_BUFFERS - 1);
        uint64_t* packet_in_call_buffer_ptr = access_packets_in_call_buffer(ch, 0, b);

        // Move to the next buffer if a packets have been consumed
        if(*num_packets_consumed >= *packet_in_call_buffer_ptr) {
            *packet_in_call_buffer_ptr = 0;
            (*access_call_buffer_tail(ch))++;
            *num_packets_consumed = 0;

            // Fence to ensure the writes to the call buffer tail get passed to other threads
            _mm_sfence();
        }
    }

private:

    /**
     * Initializes mmsghdrs and iovecs
     */
    void init_mmsghdr_iovecs();

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(user_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
