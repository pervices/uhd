// Copyright 2024 Per Vices Corporation

#include <uhd/transport/async_recv_manager.hpp>

#include <iostream>
#include <unistd.h>
#include <uhd/exception.hpp>
#include <string.h>
#include <uhd/utils/thread.hpp>
#include <uhdlib/utils/system_time.hpp>
#include <algorithm>
#include <sys/mman.h>
#include <sys/syscall.h>

#include <sys/resource.h>

namespace uhd { namespace transport {

async_recv_manager::async_recv_manager( const size_t device_total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet)
:
_num_ch(recv_sockets.size()),
_recv_sockets(recv_sockets),
_header_size(header_size),
_packet_data_size(max_sample_bytes_per_packet),

// NOTE: Theoretically padding to the cache line is required to prevent interference between threads, experimentally padding to full pages are required

_packet_pre_pad(PAGE_SIZE - _header_size),
_padded_individual_packet_size(/*Data portion padded to full page*/(std::ceil((_packet_data_size) / (double)PAGE_SIZE) * PAGE_SIZE) + /* Vita header + padding */ _header_size + _packet_pre_pad),

// NOTE: Avoid aligned_alloc and use mmap instead. aligned_alloc causes random latency spikes when said memory is being used

_all_ch_packet_buffers((uint8_t*) mmap(nullptr, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0)),

// TODO: replace with aligned alloc if padding is reduced
_io_uring_control_structs((uint8_t*) mmap(nullptr, _num_ch * _padded_io_uring_control_struct_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0))

// Create buffer for flush complete flag in seperate cache lines
{
    if(device_total_rx_channels > MAX_CHANNELS) {
        UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unsupported number of channels, constants must be updated";
        throw assertion_error("Unsupported number of channels");
    }

    // Check if memory allocation failed
    // TODO: verify all mallocs and mmaps succeeded
    if(_all_ch_packet_buffers == MAP_FAILED || _io_uring_control_structs == MAP_FAILED) {
        if(_all_ch_packet_buffers == MAP_FAILED) {
            uint8_t* tmp = ((uint8_t*) mmap(nullptr, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0));
            if(tmp != MAP_FAILED) {
                printf("Why?\n");
            } else {
                printf("mmap error: %i\n", errno);
                printf("mmap error: %s\n", strerror(errno));
                struct rlimit rlim;
                getrlimit(RLIMIT_DATA, &rlim);
                printf("rlim.rlim_cur: %lu\n", rlim.rlim_cur);
                printf("rlim.rlim_max: %lu\n", rlim.rlim_max);
            }
        }
        throw uhd::environment_error( "Failed to allocate internal buffer" );
    }

    // Flag to prevent huge pages for the large buffers
    // Not disabling huge pages can cause latency spikes
    // Theoretically huge pages could be used to improve performance, but doing so would require extensive testing and trial and error
    // TODO: try optimizing for huge pages
    // madvise(_io_uring_control_structs, _num_ch * _padded_io_uring_control_struct_size, MADV_NOHUGEPAGE);
    // madvise(_all_ch_packet_buffers, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size, MADV_NOHUGEPAGE);

    // Initialize control variables to 0
    for(size_t ch = 0; ch < _num_ch; ch++) {
        _num_packets_consumed[ch] = 0;
        _packets_advanced[ch] = 0;

        // TODO: create system to ensure a unique bgid for every channel across streamers
        _bgid_storage[ch] = (int64_t) ch + 1;
    }

    // Set entire buffer to 0 to avoid issues with lazy allocation
    memset(_io_uring_control_structs, 0, _num_ch * _padded_io_uring_control_struct_size);
    memset(_all_ch_packet_buffers, 0, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size);

    int64_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    // Initialize the uring for each channel
    for(size_t ch = 0; ch < _num_ch; ch++) {
        uring_init(ch);
    }
    // DEBUG: wait in case this is causing problems
    sleep(1);

    // TODO: see if/how to handle low core count systems
    // int64_t ch_per_thread = (int64_t) std::ceil( ( MAX_RESOURCE_FRACTION * device_total_rx_channels) / (double)num_cores );

    for(size_t n = 0; n < _num_ch; n++) {
        uhd::time_spec_t start = uhd::get_system_time();

        // TODO: flush buffers
        // while() {
        //     if(start + 30.0 < uhd::get_system_time()) {
        //         UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "A timeout occured while flushing sockets. It is likely that the device is already streaming";
        //         throw std::runtime_error("Timeout while flushing buffers");
        //     }
        // }
    }

    for(size_t ch = 0; ch < _num_ch; ch++) {
        arm_recv_multishot(ch, _recv_sockets[ch]);
    }
}

async_recv_manager::~async_recv_manager()
{
    // Stop liburing's other threads
    for(size_t ch = 0; ch < _num_ch; ch++) {
        io_uring_queue_exit(access_io_urings(ch, 0));
    }

    // Frees packets and mmsghdr buffers
    munmap(_io_uring_control_structs, _num_ch * _padded_io_uring_control_struct_size);
    munmap(_all_ch_packet_buffers, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size);
}

void async_recv_manager::uring_init(size_t ch) {
    struct io_uring_params uring_params;
    memset(&uring_params, 0, sizeof(io_uring_params));

    // Number of entries that can fit in the submission queue
    // Only 1 submission entry is needed since we are using multishot
    // TODO: see if submission queue can be set to length 1
    uring_params.sq_entries = NUM_SQ_URING_ENTRIES;
    // Number of entries that can fit in the completion queue
    uring_params.cq_entries = NUM_CQ_URING_ENTRIES;
    // IORING_SETUP_IOPOLL: use busy poll instead of interrupts - only implemented for storage devices so far
    // IORING_SETUP_SQPOLL: allows io_uring_submit to skip syscall
    // IORING_SETUP_SINGLE_ISSUER: hint to the kernel that only 1 thread will submit requests
    // IORING_SETUP_CQSIZE: pay attention to cq_entries
    // IORING_SETUP_NO_SQARRAY: for some reason this fixes dropped packets when streaming multiple channels
// TODO check if feature is supported
// Manually define constant if it is not provided by the compiler
#ifndef IORING_SETUP_NO_SQARRAY
    #define IORING_SETUP_NO_SQARRAY         (1U << 16)
#endif
    uring_params.flags = IORING_SETUP_SQ_AFF | IORING_SETUP_SQPOLL | IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_CQSIZE | IORING_SETUP_NO_SQARRAY;
    // Does nothing unless flag IORING_SETUP_SQ_AFF is set
    // TODO: find permenant means of setting core to bind to
    uring_params.sq_thread_cpu = (ch +1) * 2;
    // How long the Kernel busy wait thread will wait. If this time is exceed the next io_uring_submit will involve a syscall
    uring_params.sq_thread_idle = 0xfffffff;
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
    // NUM_SQ_URING_ENTRIES: number elements in the submission ring (TODO: figure out difference between it and sq_entries)
    // ring: Information used to access the ring
    int error = io_uring_queue_init_params(NUM_SQ_URING_ENTRIES, ring, &uring_params);
    // Ideas to try:
    // IORING_SETUP_DEFER_TASKRUN
    // IORING_SETUP_NO_MMAP
    // IORING_SETUP_REGISTERED_FD_ONLY
    // IORING_SETUP_NO_SQARRAY
    // TODO: improve error message
    if(error) {
        fprintf(stderr, "Error when creating io_uring: %s\n", strerror(-error));
        throw uhd::system_error("io_uring error");
    }

    // Initializes the ring buffer containing the location to write to
    struct io_uring_buf_ring** buffer_ring = access_io_uring_buf_rings(ch, 0);
    int ret = 0;

    // Create ring buffe to store locations to store packets
    *buffer_ring = io_uring_setup_buf_ring(ring, PACKET_BUFFER_SIZE, _bgid_storage[ch], 0, &ret);

    // TODO: improve error message
    if(ret) {
        fprintf(stderr, "Error when creating io_uring: %s\n", strerror(-ret));
        throw uhd::system_error("io_uring_setup_buf_ring");
    }

    int buffers_added = 0;
    for(uint32_t p = 0; p < PACKET_BUFFER_SIZE; p++) {
        uint8_t* packet_buffer_to_add = access_packet_vita_header(ch, 0, p);

        // Adds the packet to the list for registration (added to the ring buffer)
        // Use whichever number the buffer is (buffers_added) as it's bid
        io_uring_buf_ring_add(*buffer_ring, packet_buffer_to_add, _header_size + _packet_data_size, buffers_added, io_uring_buf_ring_mask(PACKET_BUFFER_SIZE), p);

    }
    // Commits registration of the ring buffers added by io_uring_buf_ring_add
    io_uring_buf_ring_advance(*buffer_ring, PACKET_BUFFER_SIZE);

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
    sqe->buf_group = _bgid_storage[ch];

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
        printf("Submit failed with error code: %s\n", strerror(-ret));
    } else {
        // printf("Multishot setup completed\n");
    }
}

}}
