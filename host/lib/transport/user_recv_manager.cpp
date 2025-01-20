// Copyright 2024 Per Vices Corporation

#include <uhd/transport/user_recv_manager.hpp>

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

user_recv_manager::user_recv_manager( const size_t device_total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet)
: async_recv_manager( device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet)

{
    if(device_total_rx_channels > MAX_CHANNELS) {
        UHD_LOG_ERROR("USER_RECV_MANAGER", "Unsupported number of channels, constants must be updated");
        throw assertion_error("Unsupported number of channels");
    }

    // Initialize control variables to 0
    for(size_t ch = 0; ch < _num_ch; ch++) {
        _num_packets_consumed[ch] = 0;
        _packets_advanced[ch] = 0;

        // Gets a buffer group ID equal to the number of buffer groups IDs already requested
        _bgid_storage[ch] = bgid_counter++;
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

    // TODO: handle multiple channels on low core count systems

        uhd::time_spec_t start = uhd::get_system_time();
    for(size_t ch = 0; ch < _num_ch; ch++) {

        int r;

        // Repeatedly receives packets on the socket to flush it
        while(true) {
            uint8_t flush_buffer[1];
            r = recv(_recv_sockets[ch], flush_buffer, 1, MSG_DONTWAIT);
            // No packets received, flush complete
            if(r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break;
            // Unexpected errnor occured
            } else if (r < 0) {
                throw std::runtime_error("Error while flusing socket: " + std::string(strerror(errno)));
            }
            if(start + 30.0 < uhd::get_system_time()) {
                UHD_LOGGER_ERROR("USER_RECV_MANAGER") << "A timeout occured while flushing sockets. It is likely that the device is already streaming";
                throw std::runtime_error("Timeout while flushing buffers");
            }
        }
    }

    for(size_t ch = 0; ch < _num_ch; ch++) {
        arm_recv_multishot(ch, _recv_sockets[ch]);
    }
}

user_recv_manager::~user_recv_manager()
{
    // Stop liburing's other threads
    for(size_t ch = 0; ch < _num_ch; ch++) {
        io_uring_queue_exit(access_io_urings(ch, 0));
    }

    // Frees packets and mmsghdr buffers
    munmap(_io_uring_control_structs, _num_ch * _padded_io_uring_control_struct_size);
    munmap(_all_ch_packet_buffers, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size);
}

user_recv_manager* user_recv_manager::make( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet ) {
        // Give the manager it's own cache line to avoid false sharing
        size_t recv_manager_size = (size_t) ceil(sizeof(user_recv_manager) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
        // Use placement new to avoid false sharing
        user_recv_manager* recv_manager = (user_recv_manager*) aligned_alloc(CACHE_LINE_SIZE, recv_manager_size);

        new (recv_manager) user_recv_manager(total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet);

        return recv_manager;
}

void user_recv_manager::unmake( user_recv_manager* recv_manager ) {
    // Destructor must be manually called when using placement new
    recv_manager->~user_recv_manager();
    free(recv_manager);
}

}}
