// Copyright 2024 Per Vices Corporation

#include <uhd/transport/io_uring_recv_manager.hpp>

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
#include <atomic>

namespace uhd { namespace transport {

// A counter for the number of buffer group IDs already obtained across all streamers. Used to get a unique bgid
// NOTE: This system will need to be updated if/when liburing is implemented for anything else
static std::atomic<int64_t> bgid_counter(0);

io_uring_recv_manager::io_uring_recv_manager( const size_t device_total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet)
: async_recv_manager( device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet ),
_io_uring_control_structs((uint8_t*) allocate_buffer(_num_ch * _padded_io_uring_control_struct_size))
{

    for(size_t ch = 0; ch < _num_ch; ch++) {
        // Gets a buffer group ID equal to the number of buffer groups IDs already requested
        _bgid_storage[ch] = bgid_counter++;

        // Initialize control variables to 0
        _packets_advanced[ch] = 0;
        _num_packets_consumed[ch] = 0;
    }

    // Set entire buffer to 0 to avoid issues with lazy allocation
    memset(_io_uring_control_structs, 0, _num_ch * _padded_io_uring_control_struct_size);

    // Initialize the uring for each channel
    for(size_t ch = 0; ch < _num_ch; ch++) {
        uring_init(ch);
    }

    for(size_t ch = 0; ch < _num_ch; ch++) {
        arm_recv_multishot(ch, _recv_sockets[ch]);
    }
}

io_uring_recv_manager::~io_uring_recv_manager()
{
    // Stop liburing's other threads
    for(size_t ch = 0; ch < _num_ch; ch++) {
        io_uring_queue_exit(access_io_urings(ch, 0));
    }

    munmap(_io_uring_control_structs, _num_ch * _padded_io_uring_control_struct_size);
}

void io_uring_recv_manager::uring_init(size_t ch) {
    struct io_uring_params uring_params;
    memset(&uring_params, 0, sizeof(io_uring_params));

    // Number of entries that can fit in the submission queue
    // Only 1 submission entry is needed since we are using multishot
    uring_params.sq_entries = NUM_SQ_URING_ENTRIES;
    // Number of entries that can fit in the completion queue
    uring_params.cq_entries = NUM_CQ_URING_ENTRIES;
    // IORING_SETUP_IOPOLL: use busy poll instead of interrupts - only implemented for storage devices so far
    // IORING_SETUP_SQPOLL: allows io_uring_submit to skip syscall
    // IORING_SETUP_SINGLE_ISSUER: hint to the kernel that only 1 thread will submit requests
    // IORING_SETUP_CQSIZE: pay attention to cq_entries
    // TODO: confirm if IORING_SETUP_NO_SQARRAY helps
    // IORING_SETUP_NO_SQARRAY: for some reason this fixes dropped packets when streaming multiple channels
    uring_params.flags = /*IORING_SETUP_SQ_AFF | */ /*IORING_SETUP_SQPOLL |*/ /*IORING_SETUP_SINGLE_ISSUER |*/ IORING_SETUP_CQSIZE;
    // Select the core to bind the kernel thread to
    // Does nothing unless flag IORING_SETUP_SQ_AFF is set.
    //uring_params.sq_thread_cpu;
    // Ignored if IORING_SETUP_SQPOLL not set
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
    // Filled by Kernel with info needed to access completion queue
    // uring_params.cq_off;

    struct io_uring* ring = access_io_urings(ch, 0);
    // NOTE: allow for more entires in ring buffer than needed in case it takes a while to acknowledge that we are finished with an entry
    // Initializes the ring to service requests
    // NUM_SQ_URING_ENTRIES: number elements in the submission ring (TODO: figure out difference between it and sq_entries)
    // ring: Information used to access the ring
    int error = io_uring_queue_init_params(NUM_SQ_URING_ENTRIES, ring, &uring_params);

    if(error) {
        UHD_LOG_ERROR("IO_URING_RECV_MANAGER", "Error when initializing io_uring: " + std::string(strerror(-error)));
        throw uhd::system_error("io_uring error");
    }

    // Initializes the ring buffer containing the location to write to
    struct io_uring_buf_ring** buffer_ring = access_io_uring_buf_rings(ch, 0);
    int ret = 0;

    // Create ring buffe to store locations to store packets
    *buffer_ring = io_uring_setup_buf_ring(ring, PACKET_BUFFER_SIZE, _bgid_storage[ch], 0, &ret);

    if(ret) {
        UHD_LOG_ERROR("IO_URING_RECV_MANAGER", "Error when setting up io_uring: " + std::string(strerror(-error)));
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
}

void io_uring_recv_manager::arm_recv_multishot(size_t ch, int fd) {
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

    sqe->buf_group = _bgid_storage[ch];

    // IOSQE_BUFFER_SELECT: indicates to use a registered buffer from io_uring_buf_ring_add
    io_uring_sqe_set_flags(sqe, IOSQE_BUFFER_SELECT);

    int ret = io_uring_submit(ring);

    // Error detection for submit
    // These should all be impossible
    if(ret > 1) {
        std::string message = "Multiple requests submitted to io_uring even though only 1 was intended. This should be impossible";
        UHD_LOG_ERROR("IO_URING_RECV_MANAGER", message);
        throw std::runtime_error(message);
    } else if(ret == 0) {
        std::string message = "0 requests submitted to io_uring but success was reported. This should be impossible";
        UHD_LOG_ERROR("IO_URING_RECV_MANAGER", message);
        throw std::runtime_error(message);
    } else if (ret < 0) {
        std::string message = "io_uring submit failed with: " + std::string(strerror(-ret));
        UHD_LOG_ERROR("IO_URING_RECV_MANAGER", message);
        throw std::runtime_error(message);
    }
}

void io_uring_recv_manager::get_next_async_packet_info(const size_t ch, async_packet_info* info) {

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
        std::string message = "recv failed with: " + std::string(strerror(-cqe_ptr->res));
        UHD_LOG_ERROR("IO_URING_RECV_MANAGER", message);
        throw std::runtime_error(message);
    }
}

io_uring_recv_manager* io_uring_recv_manager::make( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet ) {
        // Give the manager it's own cache line to avoid false sharing
        size_t recv_manager_size = (size_t) ceil(sizeof(io_uring_recv_manager) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
        // Use placement new to avoid false sharing
        io_uring_recv_manager* recv_manager = (io_uring_recv_manager*) aligned_alloc(CACHE_LINE_SIZE, recv_manager_size);

        new (recv_manager) io_uring_recv_manager(total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet);

        return recv_manager;
}

void io_uring_recv_manager::unmake( io_uring_recv_manager* recv_manager ) {
    // Destructor must be manually called when using placement new
    recv_manager->~io_uring_recv_manager();
    free(recv_manager);
}

}}
