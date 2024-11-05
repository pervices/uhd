// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <immintrin.h>

namespace uhd { namespace transport {

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
    // 16 is optimal for single channel, higher values are optimal for multiple
    // TODO: improve single channel performance with a higher number of buffers
    static constexpr size_t NUM_BUFFERS = 1024;

    // Mask used to roll over the buffers
    static constexpr size_t BUFFER_MASK = NUM_BUFFERS - 1;

    const uint64_t _num_ch;

    const uint64_t cache_line_size;

    const uint64_t page_size;

    // Size of uint_fast8_t + padding so it takes a whole number of cache lines
    const uint64_t padded_uint_fast8_t_size;

    // Size of int_fast64_t + padding so it takes a whole number of cache lines
    const uint64_t padded_int_fast64_t_size;


    // Vita header size
    const uint64_t _header_size;

    // TODO: see if padding the vita header is actually usefull
    // Size of Vita header + padding to be on it's own cache line
    const uint64_t _padded_header_size;

    // Size of the sample portion of Vita packets
    const uint64_t _packet_data_size;

    // Size of the buffer used to store packets
    const uint64_t packets_per_buffer;

    // Size of the buffer to contain: packets in the buffer (padded to cache line), number of times a buffer was written to (padded to cache line), all: mmsghdrs, io_vecs (length 2: header, data), padded to a whole number of pages
    const uint64_t _num_packets_stored_times_written_mmmsghdr_iovec_subbuffer_size;

    // Size of the buffer to contain: all vita headers padded to a whole number of pages
    const uint64_t _vitahdr_subbuffer_size;

    // Real size of each packet sample buffer (include's some extra padding to contain a while number of pages)
    const size_t _data_subbuffer_size;

    // Size of the combined buffer
    // Order: mmsghdrs, iovecs, Vita headers, padding out to the next memory page, samples
    const size_t _combined_buffer_size;

    // Format: NUM_BUFFERS * (number of packets stored counter, padding to next cache line, number of times this part of the ring buffer has beeing written to, padding to next cache line, mmsghdrs for the buffer, iovecs for the buffer, padding to next memory page, vita headers for the buffer, padding to next memory page, samples for the buffer)
    uint8_t* const _combined_buffer;

    // Get's a specific channel's combined buffers
    inline __attribute__((always_inline)) uint8_t* access_ch_combined_buffers(size_t ch, size_t ch_offset) {
        return _combined_buffer + ((ch + ch_offset) * NUM_BUFFERS * _combined_buffer_size);
    }

    // Get's a specific combined buffer belonging to a specific channel
    inline __attribute__((always_inline)) uint8_t* access_ch_combined_buffer(size_t ch, size_t ch_offset, size_t b) {
        return access_ch_combined_buffers(ch, ch_offset) + (b * _combined_buffer_size);
    }

    // Pointer to the start of where packet samples are stored in the buffer
    // Gets a pointer a packet buffer
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: buffer
    inline __attribute__((always_inline)) uint8_t* access_packet_data_buffer(size_t ch, size_t ch_offset, size_t b) {
        return access_ch_combined_buffer(ch, ch_offset, b) + _num_packets_stored_times_written_mmmsghdr_iovec_subbuffer_size + _vitahdr_subbuffer_size;
    }

    // Pointer to buffers where a specific packet's samples are stored
    // Gets a pointer a packet buffer
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: buffer
    inline __attribute__((always_inline)) uint8_t* access_packet_data(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return access_packet_data_buffer(ch, ch_offset, b) + (p * _packet_data_size);
    }

    // Gets a pointer to specific mmsghdr buffer
    inline __attribute__((always_inline)) uint8_t* access_mmsghdr_buffer(size_t ch, size_t ch_offset, size_t b) {
        return access_ch_combined_buffer(ch, ch_offset, b) + /* Packets in bufffer count */ padded_int_fast64_t_size + /*  Number of times the buffer has been written to count*/ padded_int_fast64_t_size;
    }

    // Gets a pointer to specific mmsghdr
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: buffer
    // p: packet number
    inline __attribute__((always_inline)) mmsghdr* access_mmsghdr(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return (mmsghdr*) (access_mmsghdr_buffer(ch, ch_offset, b) + (p * sizeof(mmsghdr)));
    }

    // Gets a pointer to iovecs for a buffer
    // ch: channel
    // ch_offset: channel offset (the first channel of the thread)
    // b: buffer
    // p: packet number
    inline __attribute__((always_inline)) iovec* access_iovec_buffer(size_t ch, size_t ch_offset, size_t b) {
        return (iovec*) (access_ch_combined_buffer(ch, ch_offset, b) + /* Packets in bufffer count */ padded_int_fast64_t_size + /*  Number of times the buffer has been written to count*/ padded_int_fast64_t_size + (packets_per_buffer * sizeof(mmsghdr)));
    }

    inline __attribute__((always_inline)) uint8_t* access_vita_hdr(size_t ch, size_t ch_offset, size_t b, size_t p) {
        return access_ch_combined_buffer(ch, ch_offset, b) + _num_packets_stored_times_written_mmmsghdr_iovec_subbuffer_size + (p * _padded_header_size);
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
        return (int_fast64_t*) access_ch_combined_buffer(ch, ch_offset, b);
    }

    // Gets a pointer to a int_fast64_t that stores the number of times a channel has had buffers been written to
    inline __attribute__((always_inline)) int_fast64_t* access_buffer_writes_count(size_t ch, size_t ch_offset, size_t b) {
        return (int_fast64_t*) (access_ch_combined_buffer(ch, ch_offset, b) + /* Packets in bufffer count */ padded_int_fast64_t_size);
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
    uint64_t stop_flag = false;

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
     * Gets the location of the next packet's vita header
     * @param ch
     * @return returns a pointer to the next packet's vita header, returns nullptr if the next packet isn't ready yet
     */
    inline __attribute__((always_inline)) uint8_t* get_next_packet_vita_header(const size_t ch) {
        size_t b = active_consumer_buffer[ch];
        return access_vita_hdr(ch, 0, b, num_packets_consumed[ch]);
    }

    /**
     * Gets the location of the next packet's samples
     * @param ch
     * @return returns a pointer to the next packet if one is available, make sure it is ready by called get_next_packet_vita_header first
     */
    inline __attribute__((always_inline)) uint8_t* get_next_packet_samples(const size_t ch) {
        size_t b = active_consumer_buffer[ch];
        return access_packet_data(ch, 0, b, num_packets_consumed[ch]);
    }

    /**
     * Gets the msg_len of the mmsghdr for the next packet.
     * Does not check to make sure the next packet is ready, make sure it is ready by using get_next_packet get_next_packet_vita_header
     * @param ch
     * @return returns msg_len of the mmsghdr corresponding to the next packet
     */
    inline __attribute__((always_inline)) uint64_t get_next_packet_length(const size_t ch) {
        size_t b = active_consumer_buffer[ch];
        return access_mmsghdr(ch, 0, b, num_packets_consumed[ch])->msg_len;
    }

    /**
     * Checks if the next packet is the first one of the buffer.
     * @param ch
     * @return True when it is the first packet of the buffer, false otherwise. Will return true even if the packet isn't ready ready
     */
    inline __attribute__((always_inline)) uint_fast8_t is_first_packet_of_buffer(const size_t ch) {
        return !num_packets_consumed[ch];
    }

    /**
     * Gets a number used to track the number of writes to the buffer and whether a write is currently in progress.
     * This function is responsible for adding the fences to ensure correct access
     * @param ch
     * @return Returns the number of complete times the currently active consumer buffer has been written to times 2. Also adds +1 if a write is currently in progress.
     */
    inline __attribute__((always_inline)) int_fast64_t get_buffer_write_count(const size_t ch) {
        // Fence to ensure that any loads from the provider thread are complete before buffer_write_count is obtained
        _mm_lfence();
        size_t b = active_consumer_buffer[ch];
        int_fast64_t buffer_write_count = *access_buffer_writes_count(ch, 0, b);
        // Fence to ensure buffer_write_count is obtained before any future loads from the provider thread occur
        _mm_lfence();
        return buffer_write_count;
    }

    /**
     * Reset the location of the consume head to the start of the buffer.
     * It is used to go to the start of a buffer when the buffer gets overwritten mid read
     * @param ch
     */
    inline __attribute__((always_inline)) void reset_buffer_read_head(const size_t ch) {
        num_packets_consumed[ch] = 0;
    }

    /**
     * Advances the the next packet to be read by the consumer thread
     * @param ch
     */
    inline __attribute__((always_inline)) void advance_packet(const size_t ch) {
        size_t b = active_consumer_buffer[ch];
        num_packets_consumed[ch]++;
        // Move to the next buffer once all packets in this buffer are consumed
        int_fast64_t* num_packets_stored_addr = access_num_packets_stored(ch, 0, b);
        if(num_packets_consumed[ch] >= *num_packets_stored_addr) {

            // Moves to the next buffer
            // & is to roll over the the first buffer once the limit is reached
            active_consumer_buffer[ch] = (active_consumer_buffer[ch] + 1) & (NUM_BUFFERS -1);

            // Resets count for number of samples consumed in the active buffer
            num_packets_consumed[ch] = 0;
        }
    }


private:

    /**
     * Function that continuously receives data and stores it in the buffer
     * @param sockets the sockets that this thread receives on. Must be a continuous subset corresponding to the storage buffers
     * @param ch_offset offset in the storages buffers that corresponds to the start of the channel
     */
    static void recv_loop(async_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset);

};
}}
