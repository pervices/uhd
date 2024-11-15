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
#include <immintrin.h>

namespace uhd { namespace transport {

async_recv_manager::async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet, const size_t device_total_rx_channels)
:
_num_ch(recv_sockets.size()),
padded_uint_fast8_t_size(std::ceil( (uint_fast32_t)sizeof(uint_fast8_t) / (double)CACHE_LINE_SIZE ) * CACHE_LINE_SIZE),
_header_size(header_size),
_padded_header_size(std::ceil( header_size / (double)CACHE_LINE_SIZE ) * CACHE_LINE_SIZE),
_packet_data_size(max_sample_bytes_per_packet),
_mmmsghdr_iovec_subbuffer_size((uint_fast32_t) std::ceil((sizeof(mmsghdr) + (2 * sizeof(iovec))) * PACKETS_PER_BUFFER / (double)PAGE_SIZE) * PAGE_SIZE),
_vitahdr_subbuffer_size((uint_fast32_t) std::ceil(_padded_header_size * PACKETS_PER_BUFFER / (double)PAGE_SIZE) * PAGE_SIZE),
// Size of each packet buffer + padding to be a whole number of pages
_data_subbuffer_size((size_t) std::ceil((PACKETS_PER_BUFFER * _packet_data_size) / (double)PAGE_SIZE) * PAGE_SIZE),

_combined_buffer_size(std::ceil((_mmmsghdr_iovec_subbuffer_size + _vitahdr_subbuffer_size + _data_subbuffer_size) / (double) PAGE_SIZE) * PAGE_SIZE ),
// Allocates buffer to store all mmsghdrs, iovecs, Vita headers, Vita payload
_combined_buffer((uint8_t*) aligned_alloc(PAGE_SIZE, _num_ch * NUM_BUFFERS * _combined_buffer_size)),

_buffer_write_count_buffer_size((uint_fast32_t) std::ceil(PAGE_SIZE * NUM_BUFFERS / (double) PAGE_SIZE) * PAGE_SIZE),
_buffer_write_count_buffer((uint8_t*) aligned_alloc(PAGE_SIZE, _num_ch * _buffer_write_count_buffer_size)),
_packets_stored_buffer_size((uint_fast32_t) std::ceil(PAGE_SIZE * NUM_BUFFERS / (double) PAGE_SIZE) * PAGE_SIZE),
_packets_stored_buffer((uint8_t*) aligned_alloc(PAGE_SIZE, _num_ch * _packets_stored_buffer_size)),
// Create buffer for flush complete flag in seperate cache lines
flush_complete((uint8_t*) aligned_alloc(CACHE_LINE_SIZE, _num_ch * padded_uint_fast8_t_size))
{
    printf("_num_ch * NUM_BUFFERS * _combined_buffer_size: %lu\n", _num_ch * NUM_BUFFERS * _combined_buffer_size);

    madvise(_combined_buffer, _num_ch * NUM_BUFFERS * _combined_buffer_size, MADV_SEQUENTIAL);

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
    active_consumer_buffer_size = (size_t) ceil(active_consumer_buffer_size / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    active_consumer_buffer = (size_t* )aligned_alloc(CACHE_LINE_SIZE, active_consumer_buffer_size);
    size_t num_packets_consumed_size = _num_ch * sizeof(int_fast64_t);
    num_packets_consumed_size = (size_t) ceil(num_packets_consumed_size / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    num_packets_consumed = (int_fast64_t*) aligned_alloc(CACHE_LINE_SIZE, num_packets_consumed_size);

    // Initialize control variables to 0
    for(size_t ch = 0; ch < _num_ch; ch++) {
        active_consumer_buffer[ch] = 0;
        num_packets_consumed[ch] = 0;
        *access_flush_complete(ch, 0) = 0;
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // madvise(access_ch_combined_buffer(ch, 0, b), _mmmsghdr_iovec_subbuffer_size, MADV_NOHUGEPAGE);
        }
    }

    // MADV_NOHUGEPAGE is used to prevent pages from being merged into a huge page
    // Having certain stuff share pages causes rare latency spikes
    // // mmsghdr and iovec buffer, write count buffer, and packet stored buffer are all important to have on their own page
    // madvise(_buffer_write_count_buffer, _num_ch * _buffer_write_count_buffer_size, MADV_NOHUGEPAGE);
    // madvise(_packets_stored_buffer, _num_ch * _packets_stored_buffer_size, MADV_NOHUGEPAGE);

    // Set entire buffer to 0 to avoid issues with lazy allocation
    memset(_combined_buffer, 0, _num_ch * NUM_BUFFERS * _combined_buffer_size);
    memset(_buffer_write_count_buffer, 0, _num_ch * _buffer_write_count_buffer_size);
    memset(_packets_stored_buffer, 0, _num_ch * _packets_stored_buffer_size);

    int64_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    int64_t ch_per_thread = (int64_t) std::ceil( ( MAX_RESOURCE_FRACTION * total_rx_channels) / (double)num_cores );
    printf("ch_per_thread: %li\n", ch_per_thread);

    num_recv_loops = (size_t) std::ceil( _num_ch / (double) ch_per_thread );

    size_t recv_loops_size = (size_t) std::ceil((sizeof(std::thread) * num_recv_loops) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    recv_loops = (std::thread*) aligned_alloc(CACHE_LINE_SIZE, recv_loops_size);

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
                stop_flag = true;
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

void async_recv_manager::recv_loop(async_recv_manager* const self_, const std::vector<int> sockets_, const size_t ch_offset_) {
    // TODO: add comments for each element
    // Struct contianing all local variables used by the main receive loop
    struct local_variables_s {
        // The manager this receives data for
        async_recv_manager* self;
        // Control variable to cycle through channels
        uint64_t ch;
        // Where in the original list of channels this loop starts from
        uint64_t ch_offset;
        // The number of channels this loop receives for
        uint64_t num_ch;
        // TODO: improve cache locality of these arrays
        // Buffer currently being written to for each channel
        uint64_t b[MAX_CHANNELS];
        int64_t* buffer_write_count;
        int64_t buffer_writes_count[MAX_CHANNELS];
        int sockets[MAX_CHANNELS];
        int r;
        bool are_packets_received;
        int error_code;
    };

    // Union to pad local_variables_s to a full page
    union local_variables_u {
        struct local_variables_s lv;
        uint8_t padding[PAGE_SIZE];
    };


    union local_variables_u local_variables __attribute__ ((aligned (PAGE_SIZE)));
    assert(sizeof(local_variables) == PAGE_SIZE);

    // madvise(&local_variables, sizeof(local_variables), MADV_NOHUGEPAGE);

    madvise(&local_variables, sizeof(local_variables), MADV_WILLNEED);

    local_variables.lv.self = self_;
    local_variables.lv.ch = 0;
    local_variables.lv.ch_offset = ch_offset_;
    local_variables.lv.num_ch = sockets_.size();
    for(size_t init_ch = 0; init_ch < local_variables.lv.num_ch; init_ch++) {
        local_variables.lv.sockets[init_ch] = sockets_[init_ch];
        local_variables.lv.b[init_ch] = 0;
        local_variables.lv.buffer_writes_count[init_ch] = 0;
    }
    local_variables.lv.error_code = 0;



    // Enables use of a realtime schedueler which will prevent this program from being interrupted and causes it to be bound to a core, but will result in it's core being fully utilized
    uhd::set_thread_priority_safe(1, true);

    // // Set the thread and socket's affinity to the current core, improves speed and reliability
    // unsigned int cpu;
    // // Syscall used because getcpu is does not exist on Oracle
    // int r = syscall(SYS_getcpu, &cpu, nullptr);
    // if(!r) {
    //     for(uint_fast32_t ch = 0; ch < local_variables.lv.num_ch; ch++) {
    //         std::vector<size_t> target_cpu(1, cpu);
    //         set_thread_affinity(target_cpu);
    //
    //         r = setsockopt(local_variables.lv.sockets[ch], SOL_SOCKET, SO_INCOMING_CPU, &cpu, sizeof(cpu));
    //         if(r) {
    //             UHD_LOGGER_WARNING("ASYNC_RECV_MANAGER") << "Unable to set socket affinity. Error code: " + std::string(strerror(errno));
    //         }
    //     }
    // } else {
    //     UHD_LOGGER_WARNING("ASYNC_RECV_MANAGER") << "getcpu failed, unable to set receive socket affinity to current core. Performance may be impacted. Error code: " + std::string(strerror(errno));
    // }

    // Configure iovecs
    for(uint_fast32_t ch = 0; ch < local_variables.lv.num_ch; ch++) {
        for(uint_fast32_t b = 0; b < NUM_BUFFERS; b++) {
            // iovecs are stored in the same buffer as mmsghdrs, after all the msghdrs
            struct iovec* iovecs = local_variables.lv.self->access_iovec_buffer(ch, local_variables.lv.ch_offset, b);
            for(uint_fast32_t p = 0; p < PACKETS_PER_BUFFER; p++) {

                uint_fast32_t header_iovec = 2 * p;
                uint_fast32_t data_iovec = 2 * p + 1;

                // Point iovecs to the location to store the vita header
                iovecs[header_iovec].iov_base = (void*) local_variables.lv.self->access_vita_hdr(ch, local_variables.lv.ch_offset, b, p);
                iovecs[header_iovec].iov_len = local_variables.lv.self->_header_size;

                // Points iovecs to the corresponding point in the buffers
                iovecs[data_iovec].iov_base = (void*) local_variables.lv.self->access_packet_data(ch, local_variables.lv.ch_offset, b, p);
                iovecs[data_iovec].iov_len = local_variables.lv.self->_packet_data_size;

                // Points mmsghdrs to the corresponding io_vec
                // Since there is only one location data should be written to per packet just take the address of the location to write to
                local_variables.lv.self->access_mmsghdr(ch, local_variables.lv.ch_offset, b, p)->msg_hdr.msg_iov = &iovecs[header_iovec];
                local_variables.lv.self->access_mmsghdr(ch, local_variables.lv.ch_offset, b, p)->msg_hdr.msg_iovlen = 2;
            }
        }
    }

    // Flush packets
    for(size_t flush_ch = 0; flush_ch < local_variables.lv.num_ch; flush_ch++) {
        int r = -1;
        // Receive packets until none are received
        while(!local_variables.lv.self->stop_flag) {
            r = recvmmsg(local_variables.lv.sockets[flush_ch], (mmsghdr*) local_variables.lv.self->access_mmsghdr_buffer(flush_ch, local_variables.lv.ch_offset, 0), PACKETS_PER_BUFFER, MSG_DONTWAIT, 0);
            // If no packets and received the error code for no packets and using MSG_DONTWAIT, continue to next channel
            if(r == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                *local_variables.lv.self->access_flush_complete(flush_ch, local_variables.lv.ch_offset) = 1;
                break;
            }
        }
    }

    // Several times this loop uses !! to ensure something is a bool (range 0 or 1)
    while(!local_variables.lv.self->stop_flag) [[likely]] {

        /// Get pointer to count used to detect if provider thread overwrote the packet while the consumer thread was accessing it
        local_variables.lv.buffer_write_count = local_variables.lv.self->access_buffer_writes_count(local_variables.lv.ch, local_variables.lv.ch_offset, local_variables.lv.b[local_variables.lv.ch]);

        // Increment the count to an odd number to indicate at writting to the buffer has begun
        // If the count is already odd skip incrementing since that indicates that the write process started but the previous recvmmsg didn't return any packets
        local_variables.lv.buffer_writes_count[local_variables.lv.ch]+= !(local_variables.lv.buffer_writes_count[local_variables.lv.ch] & 1);
        *local_variables.lv.buffer_write_count = local_variables.lv.buffer_writes_count[local_variables.lv.ch];

        // Fence to ensure buffer_write_count is set to an off number before recvmmsg
        // _mm_sfence();

        // Receives any packets already in the buffer
        local_variables.lv.r = recvmmsg(local_variables.lv.sockets[local_variables.lv.ch], (mmsghdr*) local_variables.lv.self->access_mmsghdr_buffer(local_variables.lv.ch, local_variables.lv.ch_offset, local_variables.lv.b[local_variables.lv.ch]), PACKETS_PER_BUFFER, MSG_DONTWAIT, 0);

        // Record if packets are received. Use bool since it will always be 0 or 1 which is useful for later branchless code
        local_variables.lv.are_packets_received = local_variables.lv.r > 0;

        // Set counter for number of packets stored
        *local_variables.lv.self->access_num_packets_stored(local_variables.lv.ch, local_variables.lv.ch_offset, local_variables.lv.b[local_variables.lv.ch]) = (local_variables.lv.r * local_variables.lv.are_packets_received);

        // Fence to ensure writes to recvmmsg and num_packets_stored are completed before buffer_write_count is complete
        // _mm_sfence();

        // Increment the count from an odd number to an even number to indicate recvmmsg and updating the number of packets has been completed
        local_variables.lv.buffer_writes_count[local_variables.lv.ch] += local_variables.lv.are_packets_received;
        *local_variables.lv.buffer_write_count = local_variables.lv.buffer_writes_count[local_variables.lv.ch];

        // Shift to the next buffer is any packets received, the & loops back to the first buffer
        local_variables.lv.b[local_variables.lv.ch] = (local_variables.lv.b[local_variables.lv.ch] + local_variables.lv.are_packets_received) & BUFFER_MASK;

        // Move onto the next channel, looping back to the start once reaching the end
        // Achieves results like a for loop while reducing branches
        local_variables.lv.ch++;
        local_variables.lv.ch = local_variables.lv.ch * !(local_variables.lv.ch >= local_variables.lv.num_ch);

        // DEBUG
        // _mm_sfence();

        // Set error_code to the first unhandled error encountered
        // TODO: check if false charring with errno is the issue
        // local_variables.lv.error_code = local_variables.lv.error_code | ((local_variables.lv.r == -1 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && !local_variables.lv.error_code) * errno);
    }

    if(local_variables.lv.error_code) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error during recvmmsg: " + std::string(strerror(local_variables.lv.error_code));
    }
}

}}
