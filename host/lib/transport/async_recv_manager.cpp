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

// TODO: check if this must be unique per program or globally
// TODO: move this to common include
static std::atomic<int> bgid(1);

async_recv_manager::async_recv_manager( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet, const size_t device_total_rx_channels)
:
_num_ch(recv_sockets.size()),
padded_uint_fast8_t_size(std::ceil( (uint_fast32_t)sizeof(uint_fast8_t) / (double)CACHE_LINE_SIZE ) * CACHE_LINE_SIZE),
_header_size(header_size),
_padded_header_size(std::ceil( header_size / (double)CACHE_LINE_SIZE ) * CACHE_LINE_SIZE),
// + _header_size for tmp debugging
_packet_data_size(max_sample_bytes_per_packet + _header_size),

// NOTE: Theoretically padding to the cache line is required to prevent interference between threads, experimentally padding to full pages are required

_mmmsghdr_iovec_subbuffer_size((uint_fast32_t) std::ceil((sizeof(mmsghdr) + (2 * sizeof(iovec))) * PACKETS_PER_BUFFER / (double)PAGE_SIZE) * PAGE_SIZE),
_vitahdr_subbuffer_size((uint_fast32_t) std::ceil(_padded_header_size * PACKETS_PER_BUFFER / (double)PAGE_SIZE) * PAGE_SIZE),
// Size of each packet buffer + padding to be a whole number of pages
_data_subbuffer_size((size_t) std::ceil((PACKETS_PER_BUFFER * _packet_data_size) / (double)PAGE_SIZE) * PAGE_SIZE),

_packet_pre_pad(PAGE_SIZE - _header_size),
_padded_individual_packet_size(/*Data portion padded to full page*/(std::ceil((_packet_data_size) / (double)PAGE_SIZE) * PAGE_SIZE) + /* Vita header + padding */ _header_size + _packet_pre_pad),

// NOTE: Avoid aligned_alloc and use mmap instead. aligned_alloc causes random latency spikes when said memory is being used

// Size of each receive buffer
_individual_network_buffer_size(std::ceil((_mmmsghdr_iovec_subbuffer_size + _vitahdr_subbuffer_size + _data_subbuffer_size) / (double) PAGE_SIZE) * PAGE_SIZE ),
// Allocates buffer to store all mmsghdrs, iovecs, Vita headers, Vita payload
_network_buffer((uint8_t*) mmap(nullptr, _num_ch * NUM_BUFFERS * _individual_network_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)),

_all_ch_packet_buffers((uint8_t*) mmap(nullptr, _num_ch * NUM_BUFFERS * PACKETS_PER_BUFFER * _padded_individual_packet_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)),

_buffer_write_count_buffer_size((uint_fast32_t) std::ceil(PAGE_SIZE * NUM_BUFFERS / (double) PAGE_SIZE) * PAGE_SIZE),
_buffer_write_count_buffer((uint8_t*) mmap(nullptr, _num_ch * _buffer_write_count_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)),
_packets_stored_buffer_size((uint_fast32_t) std::ceil(PAGE_SIZE * NUM_BUFFERS / (double) PAGE_SIZE) * PAGE_SIZE),
_packets_stored_buffer((uint8_t*) mmap(nullptr, _num_ch * _packets_stored_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)),
// TODO: replace with aligned alloc if padding is reduced
_io_uring_control_structs((uint8_t*) mmap(nullptr, _num_ch * _padded_io_uring_control_struct_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)),
// Create buffer for flush complete flag in seperate cache lines
flush_complete((uint8_t*) aligned_alloc(CACHE_LINE_SIZE, _num_ch * padded_uint_fast8_t_size))
{
    if(device_total_rx_channels > MAX_CHANNELS) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unsupported number of channels, constants must be updated";
        throw assertion_error("Unsupported number of channels");
    }

    // Check if memory allocation failed
    if(_network_buffer == MAP_FAILED) {
        throw uhd::environment_error( "Failed to allocate internal buffer" );
    }

    // Flag to prevent huge pages for the large buffers
    // Not disabling huge pages can cause latency spikes
    // Theoretically huge pages could be used to improve performance, but doing so would require extensive testing and trial and error
    // TODO: try optimizing for huge pages
    madvise(_io_uring_control_structs, _num_ch * _padded_io_uring_control_struct_size, MADV_NOHUGEPAGE);
    madvise(_all_ch_packet_buffers, _num_ch * NUM_BUFFERS * PACKETS_PER_BUFFER * _padded_individual_packet_size, MADV_NOHUGEPAGE);
    madvise(_network_buffer, _num_ch * NUM_BUFFERS * _individual_network_buffer_size, MADV_NOHUGEPAGE);
    madvise(_buffer_write_count_buffer, _num_ch * _buffer_write_count_buffer_size, MADV_NOHUGEPAGE);
    madvise(_packets_stored_buffer, _num_ch * _packets_stored_buffer_size, MADV_NOHUGEPAGE);

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
    }

    // Set entire buffer to 0 to avoid issues with lazy allocation
    memset(_io_uring_control_structs, 0, _num_ch * _padded_io_uring_control_struct_size);
    memset(_all_ch_packet_buffers, 0, _num_ch * NUM_BUFFERS * PACKETS_PER_BUFFER * _padded_individual_packet_size);
    memset(_network_buffer, 0, _num_ch * NUM_BUFFERS * _individual_network_buffer_size);
    memset(_buffer_write_count_buffer, 0, _num_ch * _buffer_write_count_buffer_size);
    memset(_packets_stored_buffer, 0, _num_ch * _packets_stored_buffer_size);

    int64_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    // Initialize the uring for each channel
    for(size_t ch = 0; ch < _num_ch; ch++) {
        uring_init(ch);
    }

    int64_t ch_per_thread = (int64_t) std::ceil( ( MAX_RESOURCE_FRACTION * total_rx_channels) / (double)num_cores );

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

    for(size_t ch = 0; ch < _num_ch; ch++) {
        io_uring_queue_exit(access_io_urings(ch, 0));
    }

    // Frees packets and mmsghdr buffers
    munmap(_io_uring_control_structs, _num_ch * _padded_io_uring_control_struct_size);
    munmap(_all_ch_packet_buffers, _num_ch * NUM_BUFFERS * PACKETS_PER_BUFFER * _padded_individual_packet_size);
    munmap(_network_buffer, _num_ch * NUM_BUFFERS * _individual_network_buffer_size);
    munmap(_buffer_write_count_buffer, _num_ch * _buffer_write_count_buffer_size);
    munmap(_packets_stored_buffer, _num_ch * _packets_stored_buffer_size);
    free(flush_complete);
    free(active_consumer_buffer);
    free(num_packets_consumed);
    free(recv_loops);
}

void async_recv_manager::uring_init(size_t ch) {
    struct io_uring_params uring_params;
    memset(&uring_params, 0, sizeof(io_uring_params));

    // Number of entries that can fit in the submission queue
    uring_params.sq_entries = NUM_URING_ENTRIES;
    // Number of entries that can fit in the completion queue
    uring_params.cq_entries = NUM_URING_ENTRIES;
    // IORING_SETUP_IOPOLL: use busy poll instead of interrupts - only implemented for storage devices so far
    // TODO: figure out how to get IORING_SETUP_IOPOLL working
    // IORING_SETUP_SQPOLL: allows io_uring_submit to skip syscall
    // IORING_SETUP_SINGLE_ISSUER: hint to the kernel that only 1 thread will submit requests
    // IORING_SETUP_CQSIZE: pay attention to cq_entries
    uring_params.flags = /*IORING_SETUP_IOPOLL |*/ IORING_SETUP_SQPOLL | IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_CQSIZE;
    // Does nothing unless flag IORING_SETUP_SQ_AFF is set
    // uring_params.sq_thread_cpu;
    // How long the Kernel busy wait thread will wait. If this time is exceed the next io_uring_submit will involve a syscall
    uring_params.sq_thread_idle = 100000;
    // Kernel sets this according to features supported
    // uring_params.features;
    // Does nothing unless flag IORING_SETUP_ATTACH_WQ is set
    // uring_params.wq_fd;
    // Must be all 0
    // uring_params.resv[3];
    // Filled by Kernel with info needed to access submission queue
    // uring_params.sq_off;
    // Filled by Kernel with info needed to access submission queue
    // uring_params.cq_off;

    struct io_uring* ring = access_io_urings(ch, 0);
    // NOTE: allow for more entires in ring buffer than needed in case it takes a while to acknowledge that we are finished with an entry
    // Initializes the ring to service requests
    // NUM_URING_ENTRIES: number elements in the ring
    // ring: Information used to access the ring
    int error = io_uring_queue_init_params(NUM_URING_ENTRIES, ring, &uring_params);
    // TODO: improve error message
    if(error) {
        fprintf(stderr, "Error when creating io_uring: %s\n", strerror(-error));
        throw uhd::system_error("io_uring error");
    }

    // Initializes the ring buffer containing the location to write to
    struct io_uring_buf_ring** buffer_ring = access_io_uring_buf_rings(ch, 0);
    int ret = 0;
    // Determine the current buffer group id, then increment the counter to avoid duplicate
    // TODO: set bgid to be seperate for each channel, currently implementing 1 channel as a proof of concept
    uint32_t active_bgid = bgid;//bgid++;

    // TODO: see if buffer_ring need to be accessed after this function is over
    *buffer_ring = io_uring_setup_buf_ring(ring, NUM_URING_ENTRIES, active_bgid, 0, &ret);

    // TODO: improve error message
    if(ret) {
        fprintf(stderr, "Error when creating io_uring: %s\n", strerror(-ret));
        throw uhd::system_error("io_uring_setup_buf_ring");
    }

    int buffers_added = 0;
    for(uint32_t b = 0; b < NUM_BUFFERS; b++) {
        for(uint32_t p = 0; p < PACKETS_PER_BUFFER; p++) {
            uint8_t* packet_buffer_to_add = access_packet(ch, 0, b, p);

            // Adds the packet to the list for registration (added to the ring buffer)
            // Use whichever number the buffer is (buffers_added) as it's bid
            io_uring_buf_ring_add(*buffer_ring, packet_buffer_to_add, /*_header_size +*/ _packet_data_size, buffers_added, io_uring_buf_ring_mask(NUM_URING_ENTRIES), buffers_added);

            // Registers the packet buffers in the ring buffer
            io_uring_buf_ring_advance(*buffer_ring, 1);
        }
    }
    // TODO: figure out why ring_advance works when done 1 at a time but not all at once
    // // Registers the packet buffers in the ring buffer
    // io_uring_buf_ring_advance(*buffer_ring, NUM_URING_ENTRIES);

    printf("IO_URING init passed\n");
}

void async_recv_manager::arm_recv_multishot(size_t ch, int fd) {
    struct io_uring* ring = access_io_urings(ch, 0);

    // Get submission queue
    struct io_uring_sqe *sqe;
    sqe = io_uring_get_sqe(ring);

    // Clear user data field for determinism, since it is not cleared by the library's functions
    // User data is a way of sending data with a request that will be provided when completed
    // It is useful for debugging but no during normal operation with our setup
    io_uring_sqe_set_data64(sqe, 0);

    // Prepare multishot recv
    // Multishot calls recv repeatedly
    // buf is nullptr and len 0 since the buffer is provided by buffer_ring instead of this function
    io_uring_prep_recv_multishot(sqe, fd, nullptr, 0, 0);

    // TODO: replace with system that can handle multile channels, currently bgid of 1 is always used
    sqe->buf_group = bgid;

    // IOSQE_BUFFER_SELECT: indicates to use a registered buffer from io_uring_buf_ring_add
    // IOSQE_FIXED_FILE: has something to do with registering the file earlier
    // TODO: implement IOSQE_FIXED_FILE to see if it helps performance
    // IOSQE_IO_LINK/IOSQE_IO_HARDLINK: forces ordering within a submission. Probably not useful for multishot
    io_uring_sqe_set_flags(sqe, IOSQE_BUFFER_SELECT);

    int ret = io_uring_submit(ring);
    if(ret > 1) {
        printf("To many submissions: %i\n", ret);
    } else if(ret == 0) {
        printf("No submissions: %i\n", ret);
    } else if (ret < 0) {
        printf("Submit failed with error code: %i\n", -ret);
    } else {
        printf("Multishot setup completed\n");
    }
}

void async_recv_manager::recv_loop(async_recv_manager* const self_, const std::vector<int> sockets_, const size_t ch_offset_) {
    // Struct contianing all local variables used by the main receive loop
    // TODO: look into  improving cache locality
    // Use of this struct improves worst case performance, at the cost of preventing compiler optimizations to help average performance
    struct lv_i_s {
        // The manager this receives data for
        async_recv_manager* self;
        // Control variable to cycle through channels
        uint64_t ch;
        // Where in the original list of channels this loop starts from
        uint64_t ch_offset;
        // The number of channels this loop receives for
        uint64_t num_ch;
        // Buffer currently being written to for each channel
        uint64_t b[MAX_CHANNELS];
        // Cache of buffer write count to update this loop
        int64_t* buffer_write_count;
        // Number of buffer writes so far for each channel
        int64_t buffer_writes_count[MAX_CHANNELS];
        // Sockets to receive on
        int sockets[MAX_CHANNELS];
        // Return value of recvmmsg
        int r;
        // Number to multiply by for branchless operations (bool will always be 1 or 0)
        bool are_packets_received;
        // Error code of recvmmsg
        int error_code;
    };

    // Union to pad lv_i_s to a full page to prevent interference from other threads
    // Theoretically only padding/aligning to cache line is required, experimentally padding to a full page is required
    union lv_i_u {
        struct lv_i_s lv;
        uint8_t padding[PAGE_SIZE];
    };


    union lv_i_u lv_i __attribute__ ((aligned (PAGE_SIZE)));
    assert(sizeof(lv_i) == PAGE_SIZE);

    // MADV_WILLNEED since this will be accessed often
    madvise(&lv_i, sizeof(lv_i), MADV_WILLNEED);

    madvise(&lv_i, sizeof(lv_i), MADV_NOHUGEPAGE);

    lv_i.lv.self = self_;
    lv_i.lv.ch = 0;
    lv_i.lv.ch_offset = ch_offset_;
    lv_i.lv.num_ch = sockets_.size();
    for(size_t init_ch = 0; init_ch < lv_i.lv.num_ch; init_ch++) {
        lv_i.lv.sockets[init_ch] = sockets_[init_ch];
        lv_i.lv.b[init_ch] = 0;
        lv_i.lv.buffer_writes_count[init_ch] = 0;
    }
    lv_i.lv.error_code = 0;



    // Enables use of a realtime schedueler which will prevent this program from being interrupted and causes it to be bound to a core, but will result in it's core being fully utilized
    uhd::set_thread_priority_safe(1, true);

    // Set the thread and socket's affinity to the current core, improves speed and reliability
    unsigned int cpu;
    // Syscall used because getcpu is does not exist on Oracle
    int r = syscall(SYS_getcpu, &cpu, nullptr);
    if(!r) {
        for(uint_fast32_t ch = 0; ch < lv_i.lv.num_ch; ch++) {
            std::vector<size_t> target_cpu(1, cpu);
            set_thread_affinity(target_cpu);

            r = setsockopt(lv_i.lv.sockets[ch], SOL_SOCKET, SO_INCOMING_CPU, &cpu, sizeof(cpu));
            if(r) {
                UHD_LOGGER_WARNING("ASYNC_RECV_MANAGER") << "Unable to set socket affinity. Error code: " + std::string(strerror(errno));
            }
        }
    } else {
        UHD_LOGGER_WARNING("ASYNC_RECV_MANAGER") << "getcpu failed, unable to set receive socket affinity to current core. Performance may be impacted. Error code: " + std::string(strerror(errno));
    }

    // Configure iovecs
    for(uint_fast32_t ch = 0; ch < lv_i.lv.num_ch; ch++) {
        for(uint_fast32_t b = 0; b < NUM_BUFFERS; b++) {
            // iovecs are stored in the same buffer as mmsghdrs, after all the msghdrs
            struct iovec* iovecs = lv_i.lv.self->access_iovec_buffer(ch, lv_i.lv.ch_offset, b);
            for(uint_fast32_t p = 0; p < PACKETS_PER_BUFFER; p++) {

                uint_fast32_t header_iovec = 2 * p;
                uint_fast32_t data_iovec = 2 * p + 1;

                // Point iovecs to the location to store the vita header
                iovecs[header_iovec].iov_base = (void*) lv_i.lv.self->access_vita_hdr(ch, lv_i.lv.ch_offset, b, p);
                iovecs[header_iovec].iov_len = lv_i.lv.self->_header_size;

                // Points iovecs to the corresponding point in the buffers
                iovecs[data_iovec].iov_base = (void*) lv_i.lv.self->access_packet_data(ch, lv_i.lv.ch_offset, b, p);
                iovecs[data_iovec].iov_len = lv_i.lv.self->_packet_data_size;

                // Points mmsghdrs to the corresponding io_vec
                // Since there is only one location data should be written to per packet just take the address of the location to write to
                lv_i.lv.self->access_mmsghdr(ch, lv_i.lv.ch_offset, b, p)->msg_hdr.msg_iov = &iovecs[header_iovec];
                lv_i.lv.self->access_mmsghdr(ch, lv_i.lv.ch_offset, b, p)->msg_hdr.msg_iovlen = 2;
            }
        }
    }

    // Flush packets
    for(size_t flush_ch = 0; flush_ch < lv_i.lv.num_ch; flush_ch++) {
        int r = -1;
        bool error_already_occured = false;
        // Receive packets until none are received
        while(!lv_i.lv.self->stop_flag) {
            r = recvmmsg(lv_i.lv.sockets[flush_ch], (mmsghdr*) lv_i.lv.self->access_mmsghdr_buffer(flush_ch, lv_i.lv.ch_offset, 0), PACKETS_PER_BUFFER, MSG_DONTWAIT, 0);
            // If no packets and received the error code for no packets and using MSG_DONTWAIT, continue to next channel
            if(r == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                *lv_i.lv.self->access_flush_complete(flush_ch, lv_i.lv.ch_offset) = 1;
                break;
            } else if(r == -1) {
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error while flushing recvmmsg during initialization: " + std::string(strerror(errno));
                // If this is the second time seeing an unplanned error while flushing this socket, throw and error
                if(error_already_occured) {
                    lv_i.lv.self->stop_flag = true;
                    throw std::runtime_error("Unable to recover from recvmmsg error during initialization: " + std::string(strerror(errno)));
                }
                error_already_occured = true;
            }
        }
    }

    for(size_t ch = 0; ch < lv_i.lv.num_ch; ch++) {
        lv_i.lv.self->arm_recv_multishot(ch + lv_i.lv.ch_offset, lv_i.lv.sockets[ch]);
    }

    size_t completions_received = 0;
    size_t completions_successful = 0;
    while(!lv_i.lv.self->stop_flag) [[likely]] {

        struct io_uring* ring = lv_i.lv.self->access_io_urings(lv_i.lv.ch, lv_i.lv.ch_offset);
        // Receives all requests
        // TODO move these to lv_i.lv
        io_uring_cqe *cqe_ptr;
        struct __kernel_timespec io_timeout;
        io_timeout.tv_sec = 30;
        io_timeout.tv_nsec = 0;
        sigset_t sigmask;
        sigemptyset(&sigmask);
        // TODO: consider using io_uring_wait_cqes to see if it helps
        // TODO: try io_uring_peek_cqe
        int r = io_uring_wait_cqe_timeout(ring, &cqe_ptr, &io_timeout);
        if(r == 0) {
            completions_received++;
        } else {
            // TODO: handle timeouts (or use io_uring_peek_cqe that doesn't have them)
            printf("Completion failed: %s\n", strerror(-r));
            printf("E1 completions_received: %lu\n", completions_received);
            printf("E1 completions_successful: %lu\n", completions_successful);
        }

        if(cqe_ptr->res > 0) {
            completions_successful++;
            // TODO: see if reducing the number or io_uring_buf_ring_cq_advance calls by grouping helps
            // TODO: cache access_io_uring_buf_rings result in lv_i.lv.
            io_uring_buf_ring_cq_advance(ring, *lv_i.lv.self->access_io_uring_buf_rings(lv_i.lv.ch, lv_i.lv.ch_offset), 1);

            // TODO: notify other thread the event completed
        } else {
            printf("completions_received before failure: %lu\n", completions_received);
            printf("completions_successful before failure: %lu\n", completions_successful);
            throw std::runtime_error("recv failed with: " + std::string(strerror(-cqe_ptr->res)));

        }
    }

    printf("completions_received: %lu\n", completions_received);
    printf("completions_successful: %lu\n", completions_successful);

    // Several times this loop uses !! to ensure something is a bool (range 0 or 1)
    while(!lv_i.lv.self->stop_flag) [[likely]] {

        /// Get pointer to count used to detect if provider thread overwrote the packet while the consumer thread was accessing it
        lv_i.lv.buffer_write_count = lv_i.lv.self->access_buffer_writes_count(lv_i.lv.ch, lv_i.lv.ch_offset, lv_i.lv.b[lv_i.lv.ch]);

        // Increment the count to an odd number to indicate at writting to the buffer has begun
        // If the count is already odd skip incrementing since that indicates that the write process started but the previous recvmmsg didn't return any packets
        lv_i.lv.buffer_writes_count[lv_i.lv.ch]+= !(lv_i.lv.buffer_writes_count[lv_i.lv.ch] & 1);
        *lv_i.lv.buffer_write_count = lv_i.lv.buffer_writes_count[lv_i.lv.ch];

        // Fence to ensure buffer_write_count is set to an off number before recvmmsg
        std::atomic_thread_fence(std::memory_order_release);

        // Receives any packets already in the buffer
        lv_i.lv.r = recvmmsg(lv_i.lv.sockets[lv_i.lv.ch], (mmsghdr*) lv_i.lv.self->access_mmsghdr_buffer(lv_i.lv.ch, lv_i.lv.ch_offset, lv_i.lv.b[lv_i.lv.ch]), PACKETS_PER_BUFFER, MSG_DONTWAIT, 0);

        // Record if packets are received. Use bool since it will always be 0 or 1 which is useful for later branchless code
        lv_i.lv.are_packets_received = lv_i.lv.r > 0;

        // Set counter for number of packets stored
        *lv_i.lv.self->access_num_packets_stored(lv_i.lv.ch, lv_i.lv.ch_offset, lv_i.lv.b[lv_i.lv.ch]) = (lv_i.lv.r * lv_i.lv.are_packets_received);

        // Fence to ensure writes to recvmmsg and num_packets_stored are completed before buffer_write_count is complete
        std::atomic_thread_fence(std::memory_order_release);

        // Accessing errno can cause latency spikes, enable check only when needed for debugging
#ifdef ASYNC_RECV_MANAGER_DEBUG
        // Set error_code to the first unhandled error encountered
        if(lv_i.lv.r == -1) {
            lv_i.lv.error_code = (lv_i.lv.r == -1 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && !lv_i.lv.error_code) * errno;
        }
#endif

        // Increment the count from an odd number to an even number to indicate recvmmsg and updating the number of packets has been completed
        lv_i.lv.buffer_writes_count[lv_i.lv.ch] += lv_i.lv.are_packets_received;
        *lv_i.lv.buffer_write_count = lv_i.lv.buffer_writes_count[lv_i.lv.ch];

        // Shift to the next buffer is any packets received, the & loops back to the first buffer
        lv_i.lv.b[lv_i.lv.ch] = (lv_i.lv.b[lv_i.lv.ch] + lv_i.lv.are_packets_received) & BUFFER_MASK;

        // Move onto the next channel, looping back to the start once reaching the end
        // Achieves results like a for loop while reducing branches
        lv_i.lv.ch++;
        lv_i.lv.ch = lv_i.lv.ch * !(lv_i.lv.ch >= lv_i.lv.num_ch);
    }

    // NOTE: lv_i.lv.error_code is only set if ASYNC_RECV_MANAGER_DEBUG is defiend
    if(lv_i.lv.error_code) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error during recvmmsg: " + std::string(strerror(lv_i.lv.error_code));
    }
}

}}
