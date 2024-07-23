// Copyright 2024 Per Vices Corporation

#include <uhd/transport/async_recv_manager.hpp>

#include <iostream>
#include <unistd.h>
#include <sys/sysinfo.h>
#include <uhd/exception.hpp>
#include <cmath>
#include <string.h>
#include <uhd/utils/log.hpp>
#include <uhd/utils/thread.hpp>
#include <sys/mman.h>
#include <uhdlib/utils/system_time.hpp>

namespace uhd { namespace transport {

async_recv_manager::async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet )
:
_num_ch(recv_sockets.size()),
cache_line_size(sysconf(_SC_LEVEL1_DCACHE_LINESIZE)),
packet_size(header_size + max_sample_bytes_per_packet),
padded_atomic_fast_u8_size(ceil( (size_t)sizeof(std::atomic<uint_fast8_t>) / (double)cache_line_size) * cache_line_size),
padded_atomic_fast_64_size(ceil( (size_t)sizeof(std::atomic<int_fast64_t>) / (double)cache_line_size) * cache_line_size)
{
    if(cache_line_size == 0) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unable to get cache line size, assuming it is 64";
        cache_line_size = 64;
    }
    const size_t page_size = getpagesize();

    // Create buffers used to store control data for the consumer thread
    size_t active_consumer_buffer_size = _num_ch * sizeof(size_t);
    active_consumer_buffer_size = (size_t) ceil(active_consumer_buffer_size / (double)cache_line_size) * cache_line_size;
    active_consumer_buffer = (size_t* )aligned_alloc(cache_line_size, active_consumer_buffer_size);
    size_t num_packets_consumed_size = _num_ch * sizeof(int_fast64_t);
    num_packets_consumed_size = (size_t) ceil(num_packets_consumed_size / (double)cache_line_size) * cache_line_size;
    num_packets_consumed = (int_fast64_t*) aligned_alloc(cache_line_size, num_packets_consumed_size);

    // Create buffer for flush complete flag in seperate cache lines
    flush_complete = (uint8_t*) aligned_alloc(cache_line_size, _num_ch * padded_atomic_fast_u8_size);
    // Create buffer to store count of number of packets stored
    num_packets_stored = (uint8_t*) aligned_alloc(cache_line_size, _num_ch * NUM_BUFFERS * padded_atomic_fast_64_size);

    // Create stop flag and ensure it is not on a cache line shared with anything else
    stop_flag = (std::atomic<uint_fast8_t>*) aligned_alloc(cache_line_size, padded_atomic_fast_u8_size);
    new(stop_flag) std::atomic<uint_fast8_t>(0);

    for(size_t ch = 0; ch < _num_ch; ch++) {
        active_consumer_buffer[ch] = 0;
        num_packets_consumed[ch] = 0;

        // Placement new
        new(access_flush_complete(ch, 0)) std::atomic<uint_fast8_t>(0);
        // Create buffers for the sample count for each channel's buffers. Elements in the buffer are aligned to the cache line and padded to avoid false sharing
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // Placement new
            new(access_num_packets_stored(ch, 0, b)) std::atomic<int_fast64_t>(0);
        }
    }
    *stop_flag = false;

    // Have 1 page worth of packet mmsghdrs and iovec per buffer
    // NOTE: Achieving 1 mmsghdr and 1 iovec per buffer asummes iovec has a 1 element
    packets_per_buffer = page_size / (sizeof(mmsghdr) + sizeof(iovec));
    // The actual size of the buffers in bytes
    packet_buffer_size = packets_per_buffer * packet_size;

    // Add padding to ensure each buffer starts at a new memory page
    packet_buffer_size = (size_t) ceil(packet_buffer_size / (double)page_size) * page_size;

    // Allocates buffer to store all data, buffer start and each buffer are aligned to a memory page for performance
    size_t combined_packet_buffer_size = _num_ch * NUM_BUFFERS * packet_buffer_size;
    packet_buffer = (uint8_t*) aligned_alloc(page_size, combined_packet_buffer_size);
    if(packet_buffer == nullptr) {
        throw uhd::environment_error( "out of memory when attempting to allocate:" + std::to_string(combined_packet_buffer_size) + " bytes");
    }
    memset(packet_buffer, 0, combined_packet_buffer_size);

    // Each mmmsghdr_iovec_buffer_size buffer contains both mmsghdrs and their corresponding iovec
    mmmsghdr_iovec_buffer_size = (sizeof(mmsghdr) + sizeof(iovec)) * packets_per_buffer;
    // Each buffer needs to be page aligned to that all elements sent to mmsg at a time are on the same page
    mmmsghdr_iovec_buffer_size = (size_t) ceil(mmmsghdr_iovec_buffer_size / (double)page_size) * page_size;

    size_t combined_mmsghdr_size =  _num_ch * NUM_BUFFERS * mmmsghdr_iovec_buffer_size;

    mmsghdr_iovecs = (uint8_t*) aligned_alloc(page_size, combined_mmsghdr_size);
    if(mmsghdr_iovecs == nullptr) {
        throw uhd::environment_error( "out of memory when attempting to allocate:" + std::to_string(combined_mmsghdr_size) + " bytes");
    }
    memset(mmsghdr_iovecs, 0, combined_mmsghdr_size);

    int64_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    int64_t ch_per_thread = (int64_t) ceil( ( MAX_RESOURCE_FRACTION * total_rx_channels) / (double)num_cores );

    num_recv_loops = (size_t) ceil( _num_ch / (double) ch_per_thread );

    size_t recv_loops_size = (size_t) ceil((sizeof(std::thread) * num_recv_loops) / (double)cache_line_size) * cache_line_size;
    recv_loops = (std::thread*) aligned_alloc(cache_line_size, recv_loops_size);

    std::cout << "Starting threads\n";

    // Creates thread to receive data
    size_t ch_offset = 0;
    for(size_t n = 0; n < num_recv_loops; n++) {
        std::vector<int> thread_sockets(recv_sockets.begin() + ch_offset, recv_sockets.begin() + std::min(ch_offset + ch_per_thread, recv_sockets.size()));

        // Placement new
        new(&recv_loops[n]) std::thread(recv_loop, this, thread_sockets, ch_offset);

        ch_offset+=ch_per_thread;
    }

    uhd::time_spec_t start = uhd::get_system_time();
    for(size_t n = 0; n < _num_ch; n++) {
        while(!access_flush_complete(n, 0)->load()) {
            if(start + 30.0 < uhd::get_system_time()) {
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "A timeout occured while flushing sockets. It is likely that the device is already streaming";
                throw std::runtime_error("Timeout while flushing buffers");
            }
        }
    }
    std::cout << "flush complete\n";
}

async_recv_manager::~async_recv_manager()
{
    // Manual destructor calls are required when using placement new
    *stop_flag = true;
    for(size_t n = 0; n < num_recv_loops; n++) {
        recv_loops[n].join();
        recv_loops[n].~thread();
    }

    // Frees packets and mmsghdr buffers
    free(packet_buffer);
    free(mmsghdr_iovecs);
    for(size_t ch = 0; ch < _num_ch; ch++) {
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // Unlike normal, when using pacement new the destructor must be manually called
            access_num_packets_stored(ch, 0, b)->~atomic<int_fast64_t>();
        }
        access_flush_complete(ch, 0)->~atomic<uint_fast8_t>();
    }
    stop_flag->~atomic<uint_fast8_t>();
    free(stop_flag);
    free(flush_complete);
    free(num_packets_stored);
    free(active_consumer_buffer);
    free(num_packets_consumed);
    free(recv_loops);
}

void async_recv_manager::recv_loop(async_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset) {
    // Enables use of a realtime schedueler which will prevent this program from being interrupted and causes it to be bound to a core, but will result in it's core being fully utilized
    bool priority_set = uhd::set_thread_priority_safe();

    size_t num_ch = sockets.size();

    // Set the socket's affinity, improves speed and reliability
    // Skip setting if setting priority (which also sets affinity failed)
    if(priority_set) {
        unsigned int cpu;
        int r = getcpu(&cpu, nullptr);
        if(!r) {
            for(size_t ch = 0; ch < num_ch; ch++) {
                r = setsockopt(sockets[ch], SOL_SOCKET, SO_INCOMING_CPU, &cpu, sizeof(cpu));
                if(r) {
                    UHD_LOGGER_WARNING("ASYNC_RECV_MANAGER") << "Unable to set socket affinity. Error code: " + std::string(strerror(errno));
                }
            }
        } else {
            UHD_LOGGER_WARNING("ASYNC_RECV_MANAGER") << "getcpu failed, unable to set receive socket affinity to current core. Performance may be impacted. Error code: " + std::string(strerror(errno));
        }
    }

    // Configure iovecs
    for(size_t ch = 0; ch < num_ch; ch++) {
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // iovecs are stored in the same buffer as mmsghdrs, after all the msghdrs
            struct iovec* iovecs =(iovec*) self->access_mmsghdr(ch, ch_offset, b, self->packets_per_buffer);
            for(int_fast64_t p = 0; p < self->packets_per_buffer; p++) {
                // Points iovecs to the corresponding point in the buffers
                iovecs[p].iov_base = self->access_packet_buffer(ch, ch_offset, b) + (p * self->packet_size);
                iovecs[p].iov_len = self->packet_size;

                // Points mmsghdrs to the corresponding io_vec
                // Since there is only one location data should be written to per packet just take the address of the location to write to
                self->access_mmsghdr(ch, ch_offset, b, p)->msg_hdr.msg_iov = &iovecs[p];
                self->access_mmsghdr(ch, ch_offset, b, p)->msg_hdr.msg_iovlen = 1;
            }
        }
    }

    // Create a copy of sockets that is cache line aligned/padded to ensure no false sharing
    size_t aligned_sockets_size = (size_t) ceil(num_ch * sizeof(int)/ (double) self->cache_line_size) * self->cache_line_size;
    int* aligned_sockets = (int*) aligned_alloc(self->cache_line_size, aligned_sockets_size);
    for(size_t ch = 0; ch < num_ch; ch++) {
        aligned_sockets[ch] = sockets[ch];
    }

    // Tracks which buffer is currently being written to by each channel
    // Access: b[ch]
    size_t b_size = (size_t) ceil(num_ch * sizeof(size_t)/ (double) self->cache_line_size) * self->cache_line_size;
    size_t* b = (size_t*) aligned_alloc(self->cache_line_size, b_size);

    // Memory order to use when setting flush_complete
    // Used to avoid non relaxed writes and branches
    size_t order_size = (size_t) ceil(num_ch * sizeof(std::memory_order)/ (double) self->cache_line_size) * self->cache_line_size;
    std::memory_order* flush_order = (std::memory_order*) aligned_alloc(self->cache_line_size, order_size);

    std::memory_order* b_aquire_load_order = (std::memory_order*) aligned_alloc(self->cache_line_size, order_size);

    // Sets initial values
    for(size_t ch = 0; ch < num_ch; ch++) {
        b[ch] = 0;
        flush_order[ch] = std::memory_order_release;
        b_aquire_load_order[ch] = std::memory_order_consume;
    }

    std::memory_order stop_flag_order = std::memory_order_relaxed;

    uint_fast8_t loop_counts = 0;
    while(!self->stop_flag->load(stop_flag_order)) {
        // Use atomic check for stop flag every 0x3f samples to balance need to update flag and avoiding interprocess communication
        loop_counts++;
        // stop_flag_order = (std::memory_order) ((size_t) (std::memory_order_consume * (loop_counts >> 6)) + (size_t) std::memory_order_relaxed - (size_t) (std::memory_order_relaxed * (loop_counts >> 6)));
        loop_counts &= 0x3f;

        for(size_t ch = 0; ch < num_ch; ch++) {

            // Check if buffer is empty (ready to have data stored)
            if(self->access_num_packets_stored(ch, ch_offset, b[ch])->load(b_aquire_load_order[ch])) {
                // If unable to aquire buffer, switch to atomic checks to see future changes immediately
                b_aquire_load_order[ch] = std::memory_order_consume;
                std::cout << "Main thread slow\n";
                continue;
            }

            // Switch to relaxed memory ordering for check if the buffer is available since synchronization is no longer needed
            b_aquire_load_order[ch] = std::memory_order_relaxed;

            // Receives any packets already in the buffer
            int packets_received = recvmmsg(aligned_sockets[ch], self->access_mmsghdr_buffer(ch, ch_offset, b[ch]), self->packets_per_buffer, MSG_DONTWAIT, 0);

            // If packets received
            if(packets_received >= 0) {
                // Increment the counter for number of packets stored
                // Relaxed load is acceptable here since the only time the other thread writes to this is beore the buffer is aqquired
                // * flush_complete skips recording that packets were received until the sockets have been flushed
                self->access_num_packets_stored(ch, ch_offset, b[ch])->store(packets_received * self->access_flush_complete(ch, ch_offset)->load(std::memory_order_relaxed), std::memory_order_release);

                // Shift to the next buffer if there is any data ready in the buffer, loop back to first buffer
                b[ch] = (b[ch] + self->access_flush_complete(ch, ch_offset)->load(std::memory_order_relaxed)) & (NUM_BUFFERS -1);

            // EAGAIN, EWOULDBLOCK are caused by MSG_DONTWAIT when no packets are available and expected
            // EINTR is caused by the program receiving an interrupt during recvmmsg and is expected (such as the user pressing ctrl c to exit)
            } else if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                // Sets the flag to indicate that the buffers have been cleared once a recvmmsg returns nothing
                // Only sets it if not already set to avoid need for non relaxed atomic access
                self->access_flush_complete(ch, ch_offset)->store(1, flush_order[ch]);
                flush_order[ch] = std::memory_order_relaxed;
                continue;
            } else {
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error during recvmmsg: " + std::string(strerror(errno));
                *(self->stop_flag) = true;
                return;
            }
        }
    }

    free(aligned_sockets);
    free(b);
    free(flush_order);
}

uint8_t* async_recv_manager::get_next_packet(const size_t ch) {
    size_t b = active_consumer_buffer[ch];
    // Check if the next packet is ready
    if(access_num_packets_stored(ch, 0, b)->load(std::memory_order_consume) > num_packets_consumed[ch]) {
        return access_packet_buffer(ch, 0, b) + (packet_size * num_packets_consumed[ch]);
    }
    else {
        return nullptr;
    }
}

uint32_t async_recv_manager::get_next_packet_length(const size_t ch) {
    size_t b = active_consumer_buffer[ch];
    return access_mmsghdr(ch, 0, b, num_packets_consumed[ch])->msg_len;
}

void async_recv_manager::advance_packet(const size_t ch) {
    size_t b = active_consumer_buffer[ch];
    num_packets_consumed[ch]++;
    // Move to the next buffer once all packets in this buffer are consumed
    if(num_packets_consumed[ch] >= access_num_packets_stored(ch, 0, b)->load(std::memory_order_consume)) {
        // Marks this buffer as clear
        access_num_packets_stored(ch, 0, b)->store(0, std::memory_order_release);

        // Moves to the next buffer
        // & is to roll over the the first buffer once the limit is reached
        active_consumer_buffer[ch] = (active_consumer_buffer[ch] + 1) & (NUM_BUFFERS -1);

        // Resets count for number of samples consumed in the active buffer
        num_packets_consumed[ch] = 0;
    }
}

}}
