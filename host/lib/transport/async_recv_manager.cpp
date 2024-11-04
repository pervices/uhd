// Copyright 2024 Per Vices Corporation

#include <uhd/transport/async_recv_manager.hpp>

#include <iostream>
#include <unistd.h>
#include <uhd/exception.hpp>
#include <string.h>
#include <uhd/utils/log.hpp>
#include <uhd/utils/thread.hpp>
#include <uhdlib/utils/system_time.hpp>
#include <algorithm>
#include <sys/mman.h>
#include <sys/syscall.h>

namespace uhd { namespace transport {

async_recv_manager::async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet, const size_t device_total_rx_channels)
:
_num_ch(recv_sockets.size()),
cache_line_size(sysconf(_SC_LEVEL1_DCACHE_LINESIZE)),
page_size(getpagesize()),
padded_uint_fast8_t_size(std::ceil( (uint_fast32_t)sizeof(uint_fast8_t) / (double)cache_line_size ) * cache_line_size),
padded_int_fast64_t_size(std::ceil( (uint_fast32_t)sizeof(int_fast64_t) / (double)cache_line_size ) * cache_line_size),
_header_size(header_size),
_padded_header_size(std::ceil( header_size / (double)cache_line_size ) * cache_line_size),
_packet_data_size(max_sample_bytes_per_packet),
// Have 1 page worth of packet mmsghdrs, iovecs, and Vita headers per buffer + the count for the number of packets in the buffer
// NOTE: Achieving 1 mmsghdr and 1 iovec per buffer asummes iovec has a 2 elements
packets_per_buffer(page_size / (padded_int_fast64_t_size + padded_int_fast64_t_size + sizeof(mmsghdr) + ( 2 * sizeof(iovec) ))),
_num_packets_stored_times_written_mmmsghdr_iovec_subbuffer_size((uint_fast32_t) std::ceil((/* Packets in bufffer count */ padded_int_fast64_t_size + /*  Number of times the buffer has been written to count*/ padded_int_fast64_t_size + sizeof(mmsghdr) + (2 * sizeof(iovec))) * packets_per_buffer / (double)page_size) * page_size),
_vitahdr_subbuffer_size((uint_fast32_t) std::ceil(_padded_header_size * packets_per_buffer / (double)page_size) * page_size),
// Size of each packet buffer + padding to be a whole number of pages
_data_subbuffer_size((size_t) std::ceil((packets_per_buffer * _packet_data_size) / (double)page_size) * page_size),
// padded_int_fast64_t_size is for the count for number of packets stored
_combined_buffer_size(_num_packets_stored_times_written_mmmsghdr_iovec_subbuffer_size + _vitahdr_subbuffer_size + _data_subbuffer_size),
// Allocates buffer to store all mmsghdrs, iovecs, Vita headers, Vita payload
_combined_buffer((uint8_t*) aligned_alloc(page_size, _num_ch * NUM_BUFFERS * _combined_buffer_size)),
// Create buffer for flush complete flag in seperate cache lines
flush_complete((uint8_t*) aligned_alloc(cache_line_size, _num_ch * padded_uint_fast8_t_size))
{
    if(device_total_rx_channels > MAX_CHANNELS) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unsupported number of channels, constants must be updated";
        throw assertion_error("Unsupported number of channels");
    }

    // Check if memory allocation failed
    if(_combined_buffer == nullptr) {
        throw uhd::environment_error( "aligned_alloc failed for internal buffers" );
    }

    // Create buffers used to store control data for the consumer thread
    size_t active_consumer_buffer_size = _num_ch * sizeof(size_t);
    active_consumer_buffer_size = (size_t) ceil(active_consumer_buffer_size / (double)cache_line_size) * cache_line_size;
    active_consumer_buffer = (size_t* )aligned_alloc(cache_line_size, active_consumer_buffer_size);
    size_t num_packets_consumed_size = _num_ch * sizeof(int_fast64_t);
    num_packets_consumed_size = (size_t) ceil(num_packets_consumed_size / (double)cache_line_size) * cache_line_size;
    num_packets_consumed = (int_fast64_t*) aligned_alloc(cache_line_size, num_packets_consumed_size);

    // Initialize control variables to 0
    for(size_t ch = 0; ch < _num_ch; ch++) {
        active_consumer_buffer[ch] = 0;
        num_packets_consumed[ch] = 0;
        *access_flush_complete(ch, 0) = 0;
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // Hint to keep the mmsghdrs/iovecs, vita headers in cache
            // Probably doesn't actually do anything
            madvise(access_mmsghdr_buffer(ch, 0, b), _num_packets_stored_times_written_mmmsghdr_iovec_subbuffer_size, MADV_WILLNEED);
            madvise(access_vita_hdr(ch, 0, b, 0), _vitahdr_subbuffer_size, MADV_WILLNEED);
        }
    }

    // Disbale huge pages, huge pages could help but they are unpredictable would could caused a latency spike
    madvise(_combined_buffer, _num_ch * NUM_BUFFERS * _combined_buffer_size, MADV_NOHUGEPAGE);

    // Set entire buffer to 0 to avoid issues with lazy allocation
    memset(_combined_buffer, 0, _num_ch * NUM_BUFFERS * _combined_buffer_size);

    int64_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    int64_t ch_per_thread = (int64_t) std::ceil( ( MAX_RESOURCE_FRACTION * total_rx_channels) / (double)num_cores );

    num_recv_loops = (size_t) std::ceil( _num_ch / (double) ch_per_thread );

    size_t recv_loops_size = (size_t) std::ceil((sizeof(std::thread) * num_recv_loops) / (double)cache_line_size) * cache_line_size;
    recv_loops = (std::thread*) aligned_alloc(cache_line_size, recv_loops_size);

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
        while(! *access_flush_complete(n, 0)) {
            if(start + 30.0 < uhd::get_system_time()) {
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "A timeout occured while flushing sockets. It is likely that the device is already streaming";
                throw std::runtime_error("Timeout while flushing buffers");
            }
        }
    }
}

async_recv_manager::~async_recv_manager()
{
    // Manual destructor calls are required when using placement new
    stop_flag = true;
    for(size_t n = 0; n < num_recv_loops; n++) {
        recv_loops[n].join();
        recv_loops[n].~thread();
    }

    // Frees packets and mmsghdr buffers
    free(_combined_buffer);
    free(flush_complete);
    free(active_consumer_buffer);
    free(num_packets_consumed);
    free(recv_loops);
}

void async_recv_manager::recv_loop(async_recv_manager* const self, const std::vector<int> sockets_, const size_t ch_offset) {

    // Enables use of a realtime schedueler which will prevent this program from being interrupted and causes it to be bound to a core, but will result in it's core being fully utilized
    bool priority_set = uhd::set_thread_priority_safe();

    const uint_fast32_t num_ch = sockets_.size();

    // Mask used to roll over the buffers
    const uint_fast32_t buffer_mask = self->NUM_BUFFERS - 1;

    // Records the buffer in use by each channel
    uint_fast32_t b[MAX_CHANNELS];
    // Copy of sockets used by this thread on the stack
    int sockets[MAX_CHANNELS];
    // Local copy of flush complete flag so that we never need to read from the shared flag
    uint_fast8_t local_flush_complete[MAX_CHANNELS];
    // Tracks the number of times a buffer has been written to for each channel * 2
    int_fast64_t buffer_writes_count[MAX_CHANNELS];
    for(uint_fast32_t ch = 0; ch < num_ch; ch++) {
        sockets[ch] = sockets_[ch];
        b[ch] = 0;
        local_flush_complete[ch] = 0;
        buffer_writes_count[ch] = 0;
    }

    // Set the socket's affinity, improves speed and reliability
    // Skip setting if setting priority (which also sets affinity failed)
    if(priority_set) {
        unsigned int cpu;
        // Syscall used because getcpu is does not exist on Oracle
        int r = syscall(SYS_getcpu, &cpu, nullptr);
        if(!r) {
            for(uint_fast32_t ch = 0; ch < num_ch; ch++) {
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
    for(uint_fast32_t ch = 0; ch < num_ch; ch++) {
        for(uint_fast32_t b = 0; b < self->NUM_BUFFERS; b++) {
            // iovecs are stored in the same buffer as mmsghdrs, after all the msghdrs
            struct iovec* iovecs = self->access_iovec_buffer(ch, ch_offset, b);
            for(uint_fast32_t p = 0; p < self->packets_per_buffer; p++) {

                uint_fast32_t header_iovec = 2 * p;
                uint_fast32_t data_iovec = 2 * p + 1;

                // Point iovecs to the location to store the vita header
                iovecs[header_iovec].iov_base = (void*) self->access_vita_hdr(ch, ch_offset, b, p);
                iovecs[header_iovec].iov_len = self->_header_size;

                // Points iovecs to the corresponding point in the buffers
                iovecs[data_iovec].iov_base = (void*) self->access_packet_data(ch, ch_offset, b, p);
                iovecs[data_iovec].iov_len = self->_packet_data_size;

                // Points mmsghdrs to the corresponding io_vec
                // Since there is only one location data should be written to per packet just take the address of the location to write to
                self->access_mmsghdr(ch, ch_offset, b, p)->msg_hdr.msg_iov = &iovecs[header_iovec];
                self->access_mmsghdr(ch, ch_offset, b, p)->msg_hdr.msg_iovlen = 2;
            }
        }
    }

    int error_code = 0;

    uint_fast32_t ch = 0;

    // Number of packets to receive on next recvmmsg (will be 0 if the buffer isn't ready yet)
    uint_fast32_t packets_to_recv = self->packets_per_buffer;

    size_t total_packets_received = 0;

    // Several times this loop uses !! to ensure something is a bool (range 0 or 1)
    while(!self->stop_flag) [[likely]] {

        /// Get pointer to count used to detect if provider thread overwrote the packet while the consumer thread was accessing it
        int_fast64_t* buffer_write_count = self->access_buffer_writes_count(ch, ch_offset, b[ch]);

        // Increment the count to an odd number to indicate at writting to the buffer has begun
        // If the count is already odd skip incrementing since that indicates that the write process started but the previous recvmmsg didn't return any packets
        buffer_writes_count[ch]+= !(buffer_writes_count[ch] & 1);
        *buffer_write_count = buffer_writes_count[ch];

        // Fence to ensure buffer_write_count is set to an off number before recvmmsg
        _mm_sfence();

        // Receives any packets already in the buffer
        const int r = recvmmsg(sockets[ch], (mmsghdr*) self->access_mmsghdr_buffer(ch, ch_offset, b[ch]), packets_to_recv, MSG_DONTWAIT, 0);

        // Record if packets are received. Use bool since it will always be 0 or 1 which is useful for later branchless code
        bool packets_received = r > 0;

        // Record if the count for number of buffers. Use bool since it will always be 0 or 1 which is useful for later branchless code
        bool update_counts = packets_received & local_flush_complete[ch];

        total_packets_received += r * update_counts;

        // Set counter for number of packets stored
        *self->access_num_packets_stored(ch, ch_offset, b[ch]) = (r * update_counts);

        // Fence to ensure writes to recvmmsg and num_packets_stored are completed before buffer_write_count is complete
        _mm_sfence();

        // Increment the count from an odd number to an even number to indicate recvmmsg and updating the number of packets has been completed
        buffer_writes_count[ch] += update_counts;
        *buffer_write_count = buffer_writes_count[ch];

        // Shift to the next buffer is any packets received, the & loops back to the first buffer
        b[ch] = (b[ch] + (packets_received & local_flush_complete[ch])) & buffer_mask;

        // Set flush complete (already complete || recvmmsg returned with no packets)
        local_flush_complete[ch] = local_flush_complete[ch] || (r == -1 && (errno == EAGAIN || errno == EWOULDBLOCK));
        *self->access_flush_complete(ch, ch_offset) = local_flush_complete[ch];

        // Move onto the next channel, looping back to the start once reaching the end
        // Achieves results like a for loop while reducing branches
        ch++;
        ch = ch * !(ch >= num_ch);

        // Set error_code to the first unhandled error encountered
        error_code = error_code | ((r == -1 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && !error_code) * errno);
    }

    printf("total_packets_received: %lu\n", total_packets_received);

    if(error_code) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error during recvmmsg: " + std::string(strerror(error_code));
    }
}

}}
