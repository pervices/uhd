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
    static constexpr uint32_t PACKETS_UPDATE_INCREMENT = 1024;

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

    // Stores the number of packets received on each channel
    // TODO: figure out if it can be padded to a cache line (or possibly removing padding entirely depending on future architecture decisions)
    static constexpr uint64_t PACKETS_RECEIVED_COUNTER_SIZE = PAGE_SIZE;
    uint8_t* const _packets_received_counters;

    // Gets a pointer to the part of num_packets_stored corresponding the channel and buffer.
    // The caller is resonsible for using fencing to ensure correct ordering.
    inline __attribute__((always_inline)) int64_t* access_packets_received_counter(size_t ch, size_t ch_offset) {
       return (int64_t*) (_packets_received_counters + ((ch + ch_offset) * PACKETS_RECEIVED_COUNTER_SIZE));
    }

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
    // Might prevent seg faults
    static_assert(PAGE_SIZE/2 + sizeof(io_uring_buf_ring**) > sizeof(struct io_uring), "io_uring_buf_ring** starts to early");
    inline __attribute__((always_inline)) io_uring_buf_ring** access_io_uring_buf_rings(size_t ch, size_t ch_offset) {
        return (io_uring_buf_ring**) (_io_uring_control_structs + ((ch + ch_offset) * _padded_io_uring_control_struct_size) + PAGE_SIZE/2);
    }

    // Buffer to store flags to indicate sockets have been flushed
    // Not an array of uint8_t, this is done to make manual memory operations easier
    uint8_t* const flush_complete;

    // Access flags to indicate that the sockets have been purged of old data
    // channel
    inline __attribute__((always_inline)) uint_fast8_t* access_flush_complete(size_t ch, size_t ch_offset) {
        return (uint_fast8_t*) (flush_complete + ((ch + ch_offset) * PADDED_UINT8_T_SIZE));
    }

    // The buffer currently being used by the consumer thread
    size_t* active_consumer_buffer;

    // Number of packets consumed in the active consumer buffer
    // Accessed only by the consumer thread
    // channel
    int_fast64_t* num_packets_consumed;

    // Buffer containing the threads the receive data
    std::vector<std::thread> recv_loops;

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

    bool slow_consumer_warning_printed = false;
    // TODO: make this channel specific
    bool multishot_armed = false;

    int64_t num_packets_received = 0;

    /**
     * Gets information needed to process the next packet.
     * The caller is responsible for ensuring correct fencing
     * @param ch
     * @return If a packet is ready it returns a struct containing the packet length and pointers to the Vita header and samples. If the packet is not ready the struct will contain 0 for the length and nullptr for the Vita header and samples
     */
    inline __attribute__((always_inline)) void get_next_async_packet_info(const size_t ch, async_packet_info* info) {

        // TODO: see if re-arming required after ENOBUFS
        if(!multishot_armed) {
            arm_recv_multishot(ch, _recv_sockets[ch]);
            multishot_armed = true;
        }

        struct io_uring* ring = access_io_urings(ch, 0);
        struct io_uring_cqe *cqe_ptr;

        // Checks if a packet is ready
        int r = io_uring_peek_cqe(ring, &cqe_ptr);

        // The next packet is not ready
        if(r == -EAGAIN) {
            info->length = 0;
            info->vita_header = nullptr;
            info->samples = nullptr;
            return;
        }

        // If IORING_CQE_F_MORE multishot will continue sending messages
        if(! (cqe_ptr->flags & IORING_CQE_F_MORE)) {
            printf("Multishot stopped\n");
            printf("cqe_ptr->user_data: %llu\n", cqe_ptr->user_data);
            printf("cqe_ptr->res: %u\n", cqe_ptr->res);
            printf("cqe_ptr->flags: %u\n", cqe_ptr->flags);
        }

        if(cqe_ptr->res > 0) [[likely]] {
            num_packets_received++;
            info->length = cqe_ptr->res;
            info->vita_header = access_packet_vita_header(ch, 0, num_packets_consumed[ch] & PACKET_BUFFER_MASK);
            info->samples = access_packet_samples(ch, 0, num_packets_consumed[ch] & PACKET_BUFFER_MASK);
            io_uring_cq_advance(ring, 1);

        // All buffers are used (should be unreachable)
        } else if (-cqe_ptr->res == ENOBUFS) {
            // Clear this request
            // This function is responsible for marking failed recvs are complete, advance_packet is responsible for marking successful events as complete
            io_uring_cq_advance(ring, 1);

            if(!slow_consumer_warning_printed) {
                printf("num_packets_consumed[ch]: %li\n", num_packets_consumed[ch]);
                printf("num_packets_received: %li\n", num_packets_received);
                UHD_LOG_WARNING("ASYNC_RECV_MANAGER", "Sample consumer thread to slow. Try reducing time between recv calls");
                slow_consumer_warning_printed = true;
            }
            info->length = 0;
            info->vita_header = nullptr;
            info->samples = nullptr;
        } else {
            printf("E500\n");
            throw std::runtime_error("recv failed with: " + std::string(strerror(-cqe_ptr->res)));
        }
    }

    // TODO: make this channel specific and put it next to num_packets_consumed
    int64_t packets_advanced = 0;
    /**
     * Advances the the next packet to be read by the consumer thread
     * @param ch
     */
    inline __attribute__((always_inline)) void advance_packet(const size_t ch) {
        num_packets_consumed[ch]++;

        struct io_uring* ring = access_io_urings(ch, 0);
        // io_uring_cq_advance(ring, 1);

        // int64_t packets_advancable = num_packets_consumed[ch] - packets_advanced;
        // // TODO: see if batching helps performance
        // // Batching hurt performance
        // if(packets_advancable < PACKETS_UPDATE_INCREMENT) {
        //     // io_uring_buf_ring_advance(*access_io_uring_buf_rings(ch, 0), packets_advancable);
        //     packets_advancable = 0;
        // }
        // io_uring_buf_ring_cq_advance(ring, *access_io_uring_buf_rings(ch, 0), 1);
        io_uring_buf_ring_advance(*access_io_uring_buf_rings(ch, 0), 1);
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
