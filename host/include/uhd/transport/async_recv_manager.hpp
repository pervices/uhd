// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <uhd/utils/log.hpp>

#include <immintrin.h>

namespace uhd { namespace transport {

struct async_packet_info {
    int64_t length;
    uint8_t* vita_header;
    uint8_t* samples;
};

// Creates and manages receive threads
// Threads continuously receive data and store it in a buffer for latter use
// provider thread(s) refers to the thread(s) receiving data and storing it in the buffer
// consumer thread refers to the thread calling UHD's recv function
class async_recv_manager {

protected:

    static constexpr size_t MAX_CHANNELS = 16;

    // 1/this = hint for the maximum portion of the cores to use for recv threads
    static constexpr size_t MAX_RESOURCE_FRACTION = 3;

    // Number of packets that can be stored in the buffer
    // Hard limit: 2^15 from io_uring. Use the same value for user_recv as well to maximize code similarity
    // Must be a power of 2 so the mask workers correctly
    static constexpr size_t PACKET_BUFFER_SIZE = 32768;

    // Mask used to roll over number of packets
    static constexpr size_t PACKET_BUFFER_MASK = PACKET_BUFFER_SIZE - 1;

    // Optimal alignment for using SIMD instructions to copy/convert samples
    // Set for 512 for future AVX512 copying
    static constexpr size_t SIMD_ALIGNMENT = 512;

    static constexpr size_t PAGE_SIZE = 4096;
    static constexpr size_t HUGE_PAGE_SIZE = 2048 * 1024;

    // Number of channls managed by this streamer
    const uint_fast32_t _num_ch;

    static constexpr size_t CACHE_LINE_SIZE = 64;

    // Size of uint_fast8_t + padding so it takes a whole number of cache lines
    static constexpr size_t PADDED_UINT8_T_SIZE = CACHE_LINE_SIZE;

    // Size of int_fast64_t + padding so it takes a whole number of cache lines
    static constexpr size_t PADDED_INT64_T_SIZE = CACHE_LINE_SIZE;

    const std::vector<int> _recv_sockets;

    // Vita header size
    const uint_fast32_t _header_size;

    // Size of the sample portion of Vita packets
    const uint_fast32_t _packet_data_size;

    // The offset between the start of a packet's portion of _all_ch_packet_buffers and where the Vita header starts
    // It must be set so that the start of samples (which occur immediately after the Vita header) are SIMD_ALIGNMENT aligned
    const size_t _vita_header_offset;

    // Size of the buffer containing a single packet
    const size_t _padded_individual_packet_size;

    // Buffer containing packet length, Vita header, and samples
    // Format: (packet length, padding, vita header | SIMD_ALIGNMENT boundary |, samples, padding to next SIMD_ALIGNMENT) * PACKETS_PER_BUFFER) repeat for PACKET_BUFFER_SIZE, repeat for _num_ch
    uint8_t* const _all_ch_packet_buffers;

    // Gets a pointer to the start of the buffer containing info for the packet (not the start of the packet)
    inline __attribute__((always_inline)) uint8_t* access_packet_buffer(size_t ch, size_t ch_offset, size_t p) {
        return _all_ch_packet_buffers + ((((ch + ch_offset) * PACKET_BUFFER_SIZE) + p ) * _padded_individual_packet_size);
    }

    // Gets a pointer to the Vita header for a packet (which is also the start of the packet
    inline __attribute__((always_inline)) uint8_t* access_packet_vita_header(size_t ch, size_t ch_offset, size_t p) {
        return access_packet_buffer(ch, ch_offset, p) + _vita_header_offset;
    }

    // Gets a pointer to the length of a packet
    inline __attribute__((always_inline)) int64_t* access_packet_length(size_t ch, size_t ch_offset, size_t p) {
        return (int64_t*) (access_packet_buffer(ch, ch_offset, p));
    }

    // Gets a pointer to the start of a packet's samples
    inline __attribute__((always_inline)) uint8_t* access_packet_samples(size_t ch, size_t ch_offset, size_t p) {
        return access_packet_buffer(ch, ch_offset, p) + /* Vita header ends and samples begin at the first page boundary */ SIMD_ALIGNMENT;
    }

    // Flag to indicate if io_uring needs to be armed
    bool io_uring_unarmed = true;

    // Flag to determine if a warning tell the user the gaps between recv calls is to long has been set
    bool slow_consumer_warning_printed = false;

// The constructor is protected since this class should never be instantiated on it's own, and should be created through subclasses
protected:

    /**
     * Constructor for async_recv_manager.
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     * @param manager_variant which manager implementation is being used
     */
    async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    ~async_recv_manager();

public:
    /**
     * Gets information needed to process the next packet.
     * The caller is responsible for ensuring correct fencing
     * @param ch
     * @return If a packet is ready it returns a struct containing the packet length and pointers to the Vita header and samples. If the packet is not ready the struct will contain 0 for the length and nullptr for the Vita header and samples
     */
    virtual void get_next_async_packet_info(const size_t ch, async_packet_info* info) = 0;

    /**
     * Advances the the next packet to be read by the consumer thread
     * @param ch
     */
    virtual void advance_packet(const size_t ch) = 0;

/**
     * Determines which manager to use and creates an instance of it
     * You must call auto_unmake when done
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    static async_recv_manager* auto_make( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    /**
     * Destructs and frees an io_uring_recv_manager
     * @param recv_manager* The instance to destruct and free
     */
    static void auto_unmake( async_recv_manager* recv_manager );

protected:

    /**
     * Attempts to allocate a page aligned buffer using mmap and huge pages.
     * If that fails to allocate using huge pages it will warn the user and fall back to allocating without huge pages.
     * @param size The size of the buffer to allocate in bytes
     * @return A pointer to the buffer that was allocated
     */
    static void* allocate_hugetlb_buffer_with_fallback(size_t size);

    /**
     * Attempts to allocate a page aligned buffer using mmap.
     * This also includes error detection to verify the buffer was allocated.
     * @param size The size of the buffer to allocate in bytes
     * @return A pointer to the bffer that was allocated
     */
    static void* allocate_buffer(size_t size);

};
}}
