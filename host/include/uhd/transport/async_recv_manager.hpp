// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <atomic>
#include <thread>
#include <cmath>
#include <iostream>

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

    const size_t _num_ch;

    size_t cache_line_size;

    // Real size of each data buffer (include's some extra padding to contain a while number of pages)
    int_fast64_t packet_buffer_size;

    // Pointer to buffers where packet data is stored
    // channel, buffer_num
    uint8_t* packet_buffer;
    // Gets a pointer a packet buffer
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: buffer
    inline uint8_t* access_packet_buffer(size_t ch, size_t ch_offset, size_t b) {
        return packet_buffer + ((ch + ch_offset) * NUM_BUFFERS * packet_buffer_size) + (b * packet_buffer_size);
    }

    // Size of the packets to be received not including the trailer
    const int_fast64_t packet_size;

    // Size of the buffer used to store packets
    int_fast64_t packets_per_buffer;

    // Size of the buffer to contain all mmsghdrs and io_vecs
    size_t mmmsghdr_iovec_buffer_size;

    // Pointer to a buffer which contains mmsghdr and iovec buffers
    // Each buffer contains packets_per_buffer mmsghdrs followed by packets_per_buffer iovecs
    // channel, buffer_num, packet
    // Each buffer should be on it's own memory page
    uint8_t* mmsghdr_iovecs;

    // Gets a pointer to specific mmsghdr buffer
    inline mmsghdr* access_mmsghdr_buffer(size_t ch, size_t ch_offset, size_t b) {
        return (mmsghdr*) (mmsghdr_iovecs + ((ch + ch_offset) * NUM_BUFFERS * mmmsghdr_iovec_buffer_size) + (b * mmmsghdr_iovec_buffer_size));
    }

    // Gets a pointer to specific mmsghdr
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: buffer
    // p: packet number
    inline mmsghdr* access_mmsghdr(size_t ch, size_t ch_offset,size_t b, size_t p) {
        return (mmsghdr*) (mmsghdr_iovecs + ((ch + ch_offset) * NUM_BUFFERS * mmmsghdr_iovec_buffer_size) + (b * mmmsghdr_iovec_buffer_size) + (p * sizeof(mmsghdr)));
    }

    // Size of std::atomic<uint_fast8_t> + padding so it takes a whole number of cache lines
    const size_t padded_atomic_fast_u8_size;

    // Buffer to store flags to indicate sockets have been flushed
    // Not an array of uint8_t, this is done to make manual memory operations easier
    uint8_t* flush_complete;

    //Access flags to indicate that the sockets have been purged of old data
    // channel
    inline std::atomic<uint_fast8_t>* access_flush_complete(size_t ch, size_t ch_offset) {
        return (std::atomic<uint_fast8_t>*) (flush_complete + ((ch + ch_offset) * padded_atomic_fast_u8_size));
    }

    // Size of std::atomic<int_fast64_t> + padding so it takes a whole number of cache lines
    const size_t padded_atomic_fast_64_size;

    // Number of packets stored in each buffer
    // Accessed by both provider and consumer threads
    // Use access_num_packets_stored to access
    // Actually a buffer that stores std::atomic<int_fast64_t>, using std::atomic<int_fast64_t>* messes with the math when trying to specify where elements in the buffer go to because of padding
    uint8_t* num_packets_stored;

    // Gets a pointer to the part of num_packets_stored corresponding the channel and buffer
    inline std::atomic<int_fast64_t>* access_num_packets_stored(size_t ch, size_t ch_offset, size_t b) {
        return (std::atomic<int_fast64_t>*) (num_packets_stored + ((ch + ch_offset) * _num_ch * NUM_BUFFERS * padded_atomic_fast_64_size) + (b * padded_atomic_fast_64_size));
    }

    // Flag used to tell receive threads when to stop
    // Pointer instead of value so it can be on it's own cache line
    std::atomic<uint_fast8_t>* stop_flag;

    // The buffer currently being used by the consumer thread
    size_t* active_consumer_buffer;

    // Number of packets consumed in the active consumer buffer
    // Accessed only by the consumer thread
    // channel
    int_fast64_t* num_packets_consumed;

    // Buffer containing the threads the receive data
    size_t num_recv_loops;
    std::thread* recv_loops;

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
