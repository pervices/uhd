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

    // Number of packets that can be stored in the buffer
    // Hard limit: 2^15
    // Should be a power of 2
    static constexpr size_t PACKET_BUFFER_SIZE = 32768;

    // Mask used to roll over number of packets
    static constexpr size_t PACKET_BUFFER_MASK = PACKET_BUFFER_SIZE - 1;

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

    // Number of packets to receive before updating counts used by other threads
    // TODO: implement batching io_uring_buf_ring_cq_advance calls
    static constexpr uint32_t PACKETS_UPDATE_INCREMENT = 32768/2;

    // Number of entries in each uring
    // Should be a power of 2 to avoid confusion since most kernels round this up to the next power of 2
    // Hard limit: 2 ^ 15
    static constexpr uint32_t NUM_SQ_URING_ENTRIES = 1;
    static constexpr uint32_t NUM_CQ_URING_ENTRIES = 32768;
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

    // Gets a pointer to the start of the buffer containing info for the packet (not the start of the packet
    inline __attribute__((always_inline)) uint8_t* access_packet_buffer(size_t ch, size_t ch_offset, size_t p) {
        return _all_ch_packet_buffers + ((((ch + ch_offset) * PACKET_BUFFER_SIZE) + p ) * _padded_individual_packet_size);
    }

    // Gets a pointer to the Vita header for a packet (which is also the start of the packet
    inline __attribute__((always_inline)) uint8_t* access_packet_vita_header(size_t ch, size_t ch_offset, size_t p) {
        return access_packet_buffer(ch, ch_offset, p) + _packet_pre_pad;
    }

    // Gets a pointer to the length of a packet
    inline __attribute__((always_inline)) int64_t* access_packet_length(size_t ch, size_t ch_offset, size_t p) {
        return (int64_t*) (access_packet_buffer(ch, ch_offset, p));
    }

    // Gets a pointer to the start of a packet's samples
    inline __attribute__((always_inline)) uint8_t* access_packet_samples(size_t ch, size_t ch_offset, size_t p) {
        return access_packet_buffer(ch, ch_offset, p) + /* Vita header ends and samples begin at the first page boundary */ PAGE_SIZE;
    }

    // TODO: reduce padding
    static constexpr size_t _padded_io_uring_control_struct_size = PAGE_SIZE;
    static_assert(_padded_io_uring_control_struct_size > sizeof(struct io_uring), "Padded io_uring size smaller than normal io_uring size");
    // Buffer used for control structs used by io_uring
    // Format: io_uring, io_uring_buf_ring, padding to _padded__uring_size, repeat for each channel
    // TODO: see if adding cache line or page size padding between io_uring and io_uring_buf_ring helps
    uint8_t* const _io_uring_control_structs;

    // Access the uring for a given channel. (The ring buffer containing submission and completion queues)
    inline __attribute__((always_inline)) io_uring* access_io_urings(size_t ch, size_t ch_offset = 0) {
        return (io_uring*) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size));
    }

    // Access buf ring for a given channel. (The ring buffer contain the buffers to store received data in)
    // Might prevent seg faults
    static_assert(PAGE_SIZE/2 + sizeof(io_uring_buf_ring**) > sizeof(struct io_uring), "io_uring_buf_ring** starts to early");
    inline __attribute__((always_inline)) io_uring_buf_ring** access_io_uring_buf_rings(size_t ch, size_t ch_offset) {
        return (io_uring_buf_ring**) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size) + PAGE_SIZE/2);
    }

    // Number of packets consumed in the active consumer buffer
    // Accessed only by the consumer thread
    int64_t _num_packets_consumed[MAX_CHANNELS];

    // TODO: make this channel specific and put it next to _num_packets_consumed
    int64_t _packets_advanced[MAX_CHANNELS];

    // Stores the bgid used by io_uring_setup_buf_ring and io_uring_sqe_set_flags(..., IOSQE_BUFFER_SELECT);
    // Each channel needs a unique bgid
    int64_t _bgid_storage[MAX_CHANNELS];

public:

    /**
     * Constructor for async_recv_manager.
     * @param total_rx_channels Number of rx channels on the device. Used for calculating how many threads and RAM to use
     * @param recv_sockets Vector containing the file descriptor for all sockets
     * @param header_size Size of the Vita header in bytes
     * @param max_sample_bytes_per_packet Maximum size of the sample data in bytes
     */
    async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet );

    ~async_recv_manager();

    bool slow_consumer_warning_printed = false;

    inline __attribute__((always_inline)) unsigned get_packets_advancable(size_t ch) {
        return _num_packets_consumed[ch] - _packets_advanced[ch];
    }

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
            // Set cqe_ptr to nullptr to avoid mistakes from it being stale
            *cqe_ptr = nullptr;
            // Error code to indicate no data ready for a non blocking request
            return -EAGAIN;
        }
    }

    /**
     * Gets information needed to process the next packet.
     * The caller is responsible for ensuring correct fencing
     * @param ch
     * @return If a packet is ready it returns a struct containing the packet length and pointers to the Vita header and samples. If the packet is not ready the struct will contain 0 for the length and nullptr for the Vita header and samples
     */
    inline __attribute__((always_inline)) void get_next_async_packet_info(const size_t ch, async_packet_info* info) {

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

    /**
     * Advances the the next packet to be read by the consumer thread
     * @param ch
     */
    inline __attribute__((always_inline)) void advance_packet(const size_t ch) {

        _num_packets_consumed[ch]++;

        unsigned packets_advancable = get_packets_advancable(ch);
        // Mark packets are clear in batches to improve performance
        if(packets_advancable > PACKETS_UPDATE_INCREMENT) {
            clear_packets(ch, packets_advancable);
        }
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
     * Attempts to allocate a buffer using mmap and huge pages.
     * If that fails to allocate using huge pages it will warn the user and fall back to allocating without huge pages.
     * @param size The size of the buffer to allocate in bytes
     * @return A pointer to the buffer that was allocated
     */
    static void* allocate_hugetlb_buffer_with_fallback(size_t size);

    /**
     * Attempts to allocate a buffer aligned to a page using mmap.
     * This also includes error detection to verify the buffer was allocated.
     * @param size The size of the buffer to allocate in bytes
     * @return A pointer to the bffer that was allocated
     */
    static void* allocate_buffer(size_t size);

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(async_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
