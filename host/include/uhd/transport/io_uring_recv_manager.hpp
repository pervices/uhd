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

// Manages asynchronous receives using io_uring
class io_uring_recv_manager : public async_recv_manager {

private:
    // Number of completion events to cache
    static constexpr uint32_t COMPLETION_EVENT_CACHE_SIZE = PACKET_BUFFER_SIZE/8;

    // Number of entries in each uring
    // Should be a power of 2 to avoid confusion since most kernels round this up to the next power of 2
    // Hard limit: 2 ^ 15
    static constexpr uint32_t NUM_SQ_URING_ENTRIES = 32768;
    static constexpr uint32_t NUM_CQ_URING_ENTRIES = PACKET_BUFFER_SIZE;
    // These constants must be a power of 2. Documentation says most kernels will automatically rounds up to the next power of 2, experimentally not having them as a power of 2 causes some liburing functions to fail
    static_assert((NUM_SQ_URING_ENTRIES>0 && ((NUM_SQ_URING_ENTRIES & (NUM_SQ_URING_ENTRIES-1)) == 0)), "NUM_SQ_URING_ENTRIES must be a power of 2");
    static_assert((NUM_CQ_URING_ENTRIES>0 && ((NUM_CQ_URING_ENTRIES & (NUM_CQ_URING_ENTRIES-1)) == 0)), "NUM_CQ_URING_ENTRIES must be a power of 2");
    // Verify these constants do not exceed 2^15 (hard limit from liburing)
    static_assert(NUM_SQ_URING_ENTRIES <= 32768, "NUM_SQ_URING_ENTRIES has a hard limit of 32768 imposed by liburing");
    static_assert(NUM_CQ_URING_ENTRIES <= 32768, "NUM_CQ_URING_ENTRIES has a hard limit of 32768 imposed by liburing");

    static constexpr size_t _padded_io_uring_control_struct_size = CACHE_LINE_SIZE * 4;
    static_assert((_padded_io_uring_control_struct_size > (sizeof(struct io_uring) + sizeof(io_uring_buf_ring*))), "Padded io_uring + io_uring_buf_ring* size smaller than their normal size");
    // Buffer used for control structs used by io_uring
    // Format: io_uring, io_uring_buf_ring*, padding to next cache line, repeat for each channel
    uint8_t* const _io_uring_control_structs;

    // Access the uring for a given channel. (The ring buffer containing submission and completion queues)
    inline __attribute__((always_inline)) io_uring* access_io_urings(size_t ch, size_t ch_offset = 0) {
        return (io_uring*) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size));
    }

    // Access buf ring for a given channel. (The ring buffer contain the buffers to store received data in)
    inline __attribute__((always_inline)) io_uring_buf_ring** access_io_uring_buf_rings(size_t ch, size_t ch_offset) {
        return (io_uring_buf_ring**) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size) + sizeof(struct io_uring));
    }

    // Stores the bgid used by io_uring_setup_buf_ring and io_uring_sqe_set_flags(..., IOSQE_BUFFER_SELECT);
    // Each channel needs a unique bgid
    int64_t _bgid_storage[MAX_CHANNELS];

    // Number of packets consumed by the recv_packet_handler
    uint64_t _num_packets_consumed[MAX_CHANNELS];

public:

    // TODO: add runtime check to make sure the kernel supports the required features (6.0 or later)
    /**
     * Constructor for io_uring_recv_manager.
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    io_uring_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    ~io_uring_recv_manager();

    /**
     * Calls constructor for io_uring_recv_manager and ensure io_uring_recv_manager is properly aligned.
     * You must call unmake when done
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    static io_uring_recv_manager* make( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    /**
     * Destructs and frees an io_uring_recv_manager
     * @param io_uring_recv_manager* The instance to destruct and free
     */
    static void unmake( io_uring_recv_manager* recv_manager );

    void get_next_async_packet_info(const size_t ch, async_packet_info* info) override;

    inline __attribute__((always_inline)) void advance_packet(const size_t ch) override {
        // Increment where in the packet buffer we are
        _num_packets_consumed[ch]++;

        // Increment where in the cache we are
        cached_cqe_consumed[ch]++;
    }

private:

    /**
     * Gets the next completion event (cqe)
     *
     * This function call liburing's io_uring_peek_batch_cqe and invisibly caches the result for future call
     *
     * @return On success 0 is returned and cqe_ptr is set to the next event in the cache. Returns -EAGAIN if no events are ready
     */
    inline __attribute__((always_inline)) int peek_next_cqe(size_t ch, struct io_uring_cqe **cqe_ptr)
    {
        // If there are still unused completion events in the cache, grab those
        if(_total_cached_cqe[ch] > cached_cqe_consumed[ch]) {
            *cqe_ptr = completion_cache[ch][cached_cqe_consumed[ch]];
            return 0;
        }

        // Marks all events in the cache as completed
        io_uring* ring = access_io_urings(ch);
        if(_total_cached_cqe[ch] > 0) {
            io_uring_buf_ring_cq_advance(ring, *access_io_uring_buf_rings(ch, 0), _total_cached_cqe[ch]);
        }

        // Get new completion events
        int r = io_uring_peek_batch_cqe(ring, completion_cache[ch], COMPLETION_EVENT_CACHE_SIZE);

        // If events ready
        // Not actually likely, just marked as such since it is more important
        if(r > 0) [[likely]] {
            // Reset the number of cached events consumed
            cached_cqe_consumed[ch] = 0;
            // Update the number of events in the cache
            _total_cached_cqe[ch] = r;
            // Provide the first event in the cache to the requester
            *cqe_ptr = completion_cache[ch][0];
            return 0;
        } else {
            // If r == 0 or r == -EAGAIN, all other results are impossible
            // Set cqe_ptr to nullptr to avoid non deterministic behaviour if the return value is checked wrong
            *cqe_ptr = nullptr;
            // Marks the cache as empty and that no samples from it have been consumed since io_uring_peek_batch_cqe cleared it
            _total_cached_cqe[ch] = 0;
            cached_cqe_consumed[ch] = 0;
            // No events are ready, inform the user via returning -EAGAIN
            return -EAGAIN;
        }
    }

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
     * Number of completion events in the cache
     */
    int _total_cached_cqe[MAX_CHANNELS];
    /**
     * Number of completion events in the cache that are full
     */
    int cached_cqe_consumed[MAX_CHANNELS];

    /**
     * Cache completion events to minimize the number of calls of call for getting/clearing completion events
     * Make sure this is last in the class since it is massive to aviod spreading out all the other variables in the class
     */
    io_uring_cqe* completion_cache[MAX_CHANNELS][COMPLETION_EVENT_CACHE_SIZE];
};
}}
