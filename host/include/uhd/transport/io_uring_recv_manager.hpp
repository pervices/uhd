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
    // Number of packets to receive before marking events as completed/marking buffers as clear
    // Packets are marked as clear in batching to avoid contention between threads
    static constexpr uint32_t PACKETS_UPDATE_INCREMENT = PACKET_BUFFER_SIZE/2;

    // Number of entries in each uring
    // Should be a power of 2 to avoid confusion since most kernels round this up to the next power of 2
    // Hard limit: 2 ^ 15
    static constexpr uint32_t NUM_SQ_URING_ENTRIES = 1;
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

    // Number of packets io_uring has been told have been consumed
    int64_t _packets_advanced[MAX_CHANNELS];

    // Number of packets consumed by the recv_packet_handler
    int64_t _num_packets_consumed[MAX_CHANNELS];

public:

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

    /**
     * A modified version of io_uring_peek_cqe that peeks at a pseudo head instead of the actual head of the queue.
     * This function exists because we want to minimize updates to variables used by liburing (such the location of the head) and still need to be able to access elements not at the official head.
     */
    inline __attribute__((always_inline)) int custom_io_uring_peek_cqe(size_t ch, struct io_uring *ring, struct io_uring_cqe **cqe_ptr)
    {
        constexpr unsigned mask = NUM_CQ_URING_ENTRIES - 1;

        unsigned tail = io_uring_smp_load_acquire(ring->cq.ktail);
        // pseudo_head = real_head + offset
        unsigned head = *ring->cq.khead + get_packets_advancable(ch);

        unsigned available = tail - head;
        // There is an event in the completion queue that is ready to be used
        if (available) {
            // Tell the user the location of the event
            *cqe_ptr = &ring->cq.cqes[head & mask];
            // Return0 to indicate success
            return 0;
        } else {
            *cqe_ptr = nullptr;
            unsigned advancable = get_packets_advancable(ch);
            // Since we are caught up, take the opportunity to mark packets as clear
            // Putting it inside if might give better performance what advancing with 0
            if(advancable) {
                clear_packets(ch, get_packets_advancable(ch));
            }
            return -EAGAIN;
        }
    }

    inline __attribute__((always_inline)) void advance_packet(const size_t ch) override {

        _num_packets_consumed[ch]++;

        unsigned packets_advancable = get_packets_advancable(ch);
        // Mark packets are clear in batches to improve performance
        if(packets_advancable > PACKETS_UPDATE_INCREMENT) {
            clear_packets(ch, packets_advancable);
        }
    }

private:

    /**
     * Gets information needed to process the next packet.
     * The caller is responsible for ensuring correct fencing
     * @param ch
     * @return If a packet is ready it returns a struct containing the packet length and pointers to the Vita header and samples. If the packet is not ready the struct will contain 0 for the length and nullptr for the Vita header and samples
     */
    // TODO: make non inline
    inline __attribute__((always_inline)) void get_next_async_packet_info(const size_t ch, async_packet_info* info) override {

        struct io_uring* ring = access_io_urings(ch, 0);
        struct io_uring_cqe *cqe_ptr;

        // Checks if a packet is ready
        int r = custom_io_uring_peek_cqe(ch, ring, &cqe_ptr);

        // The next packet is not ready
        if(r == -EAGAIN) {
            info->length = 0;
            info->vita_header = nullptr;
            info->samples = nullptr;
            return;
        }

        if(cqe_ptr->res > 0) [[likely]] {
            // IORING_CQE_F_MORE indicates multishot will continue sending messages
            // If IORING_CQE_F_MORE is not present multishot has stopped and must be restarted
            if(! (cqe_ptr->flags & IORING_CQE_F_MORE)) [[unlikely]] {
                // Issues new multishot request
                arm_recv_multishot(ch, _recv_sockets[ch]);
            }

            info->length = cqe_ptr->res;
            info->vita_header = access_packet_vita_header(ch, 0, _num_packets_consumed[ch] & PACKET_BUFFER_MASK);
            info->samples = access_packet_samples(ch, 0, _num_packets_consumed[ch] & PACKET_BUFFER_MASK);

        // All buffers are used (should be unreachable)
        } else if (-cqe_ptr->res == ENOBUFS) {
            // Clear this request
            // This function is responsible for marking failed recvs are complete, advance_packet is responsible for marking successful events as complete
            io_uring_cq_advance(ring, 1);

            if(!slow_consumer_warning_printed) {
                UHD_LOG_WARNING("ASYNC_RECV_MANAGER", "Sample consumer thread to slow. Try reducing time between recv calls");
                slow_consumer_warning_printed = true;
            }
            info->length = 0;
            info->vita_header = nullptr;
            info->samples = nullptr;
        } else {
            throw std::runtime_error("recv failed with: " + std::string(strerror(-cqe_ptr->res)));
        }
    }

    inline __attribute__((always_inline)) unsigned get_packets_advancable(size_t ch) {
        return _num_packets_consumed[ch] - _packets_advanced[ch];
    }

    /**
     * Lets liburing know that packets have been consumed
     * @param ch The channel whose packets to mark as clear
     * @param n The number of packets to mark as clear
    */
    inline __attribute__((always_inline)) void clear_packets(const size_t ch, const unsigned n) {
        io_uring_buf_ring_cq_advance(access_io_urings(ch), *access_io_uring_buf_rings(ch, 0), n);
        _packets_advanced[ch] += n;
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

};
}}
