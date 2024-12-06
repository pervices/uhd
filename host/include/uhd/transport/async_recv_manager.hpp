// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <liburing.h>

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

private:

    // (1 / this) is the maximum portion of CPU cores that can be used by this program
    static constexpr int_fast32_t MAX_RESOURCE_FRACTION = 3;

    static constexpr size_t MAX_CHANNELS = 16;

    // Number of buffers per ch
    // Must be a power of 2 and a constexpr, for some reason having it non constexpr will result in random lag spikes (but only on some runs)
    static constexpr size_t NUM_BUFFERS = 1024;

    static constexpr size_t BUFFER_MASK = NUM_BUFFERS - 1;

    static constexpr size_t PAGE_SIZE = 4096;

    const uint_fast32_t _num_ch;

    static constexpr size_t CACHE_LINE_SIZE = 64;

    // Size of uint_fast8_t + padding so it takes a whole number of cache lines
    const uint_fast32_t padded_uint_fast8_t_size;

    // Size of int_fast64_t + padding so it takes a whole number of cache lines
    static constexpr size_t PADDED_INT64_T_SIZE = CACHE_LINE_SIZE;

    // Vita header size
    const uint_fast32_t _header_size;

    // Size of the sample portion of Vita packets
    const uint_fast32_t _packet_data_size;

    // Number of packets per buffer

    static constexpr uint32_t PACKETS_PER_BUFFER = 32;

    // Number of entries in each uring
    // Should be a power of 2 to avoid confusion since most kernels round this up to the next power of 2
    static constexpr uint32_t NUM_URING_ENTRIES = PACKETS_PER_BUFFER * NUM_BUFFERS;
    // TODO: add assert is a power of 2

    // Amount of padding before the start of the packet
    // Padding should be such that the data portion starts aligned
    // TODO: optimize target alignment. Currently it is page aligned, it can probably be adjusted to be
    const size_t _packet_pre_pad;

    // Size of the buffer containing a single packet
    const size_t _padded_individual_packet_size;

    // Buffer containing packets + packet length
    // TODO: figure out if page alignment is necessary, or only 512 bytes (for potential future AVX512 use)
    // Format: (length, padding, vita header | page boundary |, samples, padding to next page) * PACKETS_PER_BUFFER * _num_ch
    uint8_t* const _all_ch_packet_buffers;

    // Gets a pointer to the start of a packet (which is also the location of the Vita header)
    inline __attribute__((always_inline)) uint8_t* access_packet(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return _all_ch_packet_buffers + (((b * PACKETS_PER_BUFFER) + ((ch + ch_offset) * NUM_BUFFERS * PACKETS_PER_BUFFER) + p ) * _padded_individual_packet_size) + _packet_pre_pad;
    }

    // Gets a pointer to the length of a packet
    inline __attribute__((always_inline)) int64_t* access_packet_length(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return (int64_t*) (_all_ch_packet_buffers + (((b * PACKETS_PER_BUFFER) + ((ch + ch_offset) * NUM_BUFFERS * PACKETS_PER_BUFFER) + p ) * _padded_individual_packet_size));
    }

    // Gets a pointer to the start of a packet's samples
    inline __attribute__((always_inline)) uint8_t* access_packet_samples(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return _all_ch_packet_buffers + (((b * PACKETS_PER_BUFFER) + ((ch + ch_offset) * NUM_BUFFERS * PACKETS_PER_BUFFER) + p ) * _padded_individual_packet_size) + /* Vita header ends and samples begin at the first page boundary */ PAGE_SIZE;
    }

    // Number of packets stored in a buffer
    const size_t _packets_stored_buffer_size;
    uint8_t* const _packets_stored_buffer;

    // TODO: reduce padding
    static constexpr size_t _padded_io_uring_control_struct_size = PAGE_SIZE;
    static_assert(_padded_io_uring_control_struct_size > sizeof(struct io_uring), "Padded io_uring size smaller than normal io_uring size");
    // Buffer used for control structs used by io_uring
    // Format: io_uring, io_uring_buf_ring, padding to _padded__uring_size, repeat for each channel
    // TODO: see if adding cache line or page size padding between io_uring and io_uring_buf_ring helps
    uint8_t* const _io_uring_control_structs;

    // Access the uring for a given channel. (The ring buffer containing submission and completion queues)
    inline __attribute__((always_inline)) io_uring* access_io_urings(size_t ch, size_t ch_offset) {
        return (io_uring*) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size));
    }

    // Access buf ring for a given channel. (The ring buffer contain the buffers to store received data in)
    //
    inline __attribute__((always_inline)) io_uring_buf_ring** access_io_uring_buf_rings(size_t ch, size_t ch_offset) {
        return (io_uring_buf_ring**) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size) + sizeof(io_uring));
    }

    // Buffer to store flags to indicate sockets have been flushed
    // Not an array of uint8_t, this is done to make manual memory operations easier
    uint8_t* const flush_complete;

    // Access flags to indicate that the sockets have been purged of old data
    // channel
    inline __attribute__((always_inline)) uint_fast8_t* access_flush_complete(size_t ch, size_t ch_offset) {
        return (uint_fast8_t*) (flush_complete + ((ch + ch_offset) * padded_uint_fast8_t_size));
    }

    // Gets a pointer to the part of num_packets_stored corresponding the channel and buffer
    // Use _mm_sfence after to ensure data written to this is complete
    // Theoretically the compiler could optimize out writes to this without atomic or valatile
    // Practically/experimentally it does not optimize the writes out
    inline __attribute__((always_inline)) int_fast64_t* access_num_packets_stored(size_t ch, size_t ch_offset, size_t b) {
       return (int_fast64_t*) (_packets_stored_buffer + ((ch + ch_offset) * _packets_stored_buffer_size) + (PAGE_SIZE * b));
    }

    // The buffer currently being used by the consumer thread
    size_t* active_consumer_buffer;

    // Number of packets consumed in the active consumer buffer
    // Accessed only by the consumer thread
    // channel
    int_fast64_t* num_packets_consumed;

    // Buffer containing the threads the receive data
    size_t num_recv_loops;
    std::thread* recv_loops;

    // Flag used to tell receive threads when to stop
    uint_fast8_t stop_flag = false;

public:

    /**
     * Constructor for async_recv_manager.
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet, const size_t device_total_rx_channels );

    ~async_recv_manager();

    /**
     * Gets information needed to process the next packet
     * @param ch
     * @return If a packet is ready it returns a struct containing the packet length and pointers to the Vita header and samples. If the packet is not ready the struct will contain 0 for the length and nullptr for the Vita header and samples
     */
    inline __attribute__((always_inline)) void get_next_async_packet_info(const size_t ch, async_packet_info* info) {
        size_t b = active_consumer_buffer[ch];
        if(num_packets_consumed[ch] < *access_num_packets_stored(ch, 0, b)) {
            info->length = *access_packet_length(ch, 0, b, num_packets_consumed[ch]);
            info->vita_header = access_packet(ch, 0, b, num_packets_consumed[ch]);
            info->samples = access_packet_samples(ch, 0, b, num_packets_consumed[ch]);
        // Next packet isn't ready
        } else {
            info->length = 0;
            info->vita_header = nullptr;
            info->samples = nullptr;
        }
    }

    /**
     * Advances the the next packet to be read by the consumer thread
     * @param ch
     */
    inline __attribute__((always_inline)) void advance_packet(const size_t ch) {
        num_packets_consumed[ch]++;
        // Move to the next buffer once the buffer is consumed
        if(num_packets_consumed[ch] >= PACKETS_PER_BUFFER) {
            // Clears the number of packets in the buffer
            // TODO: change to system that doesn't require communication from this thread back to the previous one
            *access_num_packets_stored(ch, 0, active_consumer_buffer[ch]) = 0;

            // Moves to the next buffer
            // & is to roll over the the first buffer once the limit is reached
            active_consumer_buffer[ch] = (active_consumer_buffer[ch] + 1) & BUFFER_MASK;

            // Resets count for number of samples consumed in the active buffer
            num_packets_consumed[ch] = 0;
        }
    }


private:

    /**
     * Helper function to initialize the uring for a channel.
     * It is called during the constructor to make undertanding liburing initialization easier
     * @param ch The channel who's uring to initialize
     */
    void uring_init(size_t ch);

    /**
     * Arms recv_multishot
     * Must be called after flushing the sockets and after any time the completion queue gets full
     */
    void arm_recv_multishot(size_t ch, int fd);

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(async_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
