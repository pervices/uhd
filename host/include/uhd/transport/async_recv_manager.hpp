// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <atomic>
#include <thread>

namespace uhd { namespace transport {

// Creates and manages receive threads
// Threads continuously receive data and store it in a buffer for latter use
// provider thread(s) refers to the thread(s) receiving data and storing it in the buffer
// consumer thread refers to the thread calling UHD's recv function
class async_recv_manager {

private:

    // Number of buffers to be created per channel cycle through, must be a power of 2
    // Enough for 0.1s of data with 1 packet per buffer with packets containing 2220 samples
    // TODO: reduce number of buffer depending on system RAM
    static constexpr int_fast64_t NUM_BUFFERS = 65536;

    // (1 / this) is the maximum portion of CPU cores and RAM that can be used by this program
    static constexpr int_fast32_t MAX_RESOURCE_FRACTION = 3;

    // Maximum packets per recvmmsg with multiple channels per thread, ignored when only 1 channel per thread
    static constexpr uint32_t MAX_PACKETS_PER_RECVMMSG = 20;

    // Pointer to buffers where packet data is stored
    // channel, buffer_num
    std::vector<std::vector<uint8_t*>> packet_buffer_ptrs;

    // Pointer to buffers which contain mmsghdr for each buffer, followed by the iovec containing data
    // channel, buffer_num, packet
    // Each buffer should be on it's own memory page
    std::vector<std::vector<struct mmsghdr*>> mmsghdrs;

    // Flag to indicate that hte sockets have been purged of old data
    // channel
    std::vector<std::atomic<uint_fast8_t>> flush_complete;

    // Number of packets stored in each buffer
    // Accessed by both provider and consumer threads
    // channel, buffer_num
    std::vector<std::vector<std::atomic<int_fast64_t>>*> num_packets_stored;

    // Number of packets the consumer thread knows are available in the current buffer
    // channel
    std::vector<int_fast64_t> known_num_packets_stored;

    // The buffer currently being used by the consumer thread
    std::vector<size_t> active_consumer_buffer;

    // Number of packets consumed in the active consumer buffer
    // Accessed only by the consumer thread
    // channel, buffer_num
    std::vector<int_fast64_t> num_packets_consumed;

    // Size of the packets to be received not including the trailer
    const int_fast64_t packet_size;

    // Size of the buffer used to store packets
    int_fast64_t packets_per_buffer;

    // Vector containing the threads the receive data
    std::vector<std::thread> recv_loops;

    // Flag used to tell receive threads when to stop
    std::atomic<uint_fast8_t> stop_flag;

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

    /**
     * Gets the location of the next packet.
     * @param ch
     * @return returns a pointer to the next packet if one is available, returns nullptr if the next packet isn't ready yet
     */
    uint8_t* get_next_packet(const size_t ch);

    /**
     * Gets the msg_len of the mmsghdr for the next packet.
     * Does not check to make sure the next packet is ready, make sure it is ready by using get_next_packet first
     * @param ch
     * @return returns msg_len of the mmsghdr corresponding to the next packet
     */
    uint32_t get_next_packet_length(const size_t ch);

    /**
     * Advances the the next packet to be read by the consumer thread
     * @param ch
     */
    void advance_packet(const size_t ch);


private:

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(async_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
