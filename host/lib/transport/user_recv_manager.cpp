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

user_recv_manager::user_recv_manager( const size_t device_total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet)
: async_recv_manager( device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet)

{
    if(device_total_rx_channels > MAX_CHANNELS) {
        UHD_LOG_ERROR("USER_RECV_MANAGER", "Unsupported number of channels, constants must be updated");
        throw assertion_error("Unsupported number of channels");
    }

    // Initialize the uring for each channel
    for(size_t ch = 0; ch < _num_ch; ch++) {
        // TODO: Initialize recv loop
    }

    for(size_t ch = 0; ch < _num_ch; ch++) {
        // TODO: start receive thread
    }
}

user_recv_manager::~user_recv_manager()
{
    // TODO: stop recv threads
}

void user_recv_manager::get_next_async_packet_info(const size_t ch, async_packet_info* info) {
    // TODO: implement
}

void user_recv_manager::clear_packets(const size_t ch, const unsigned n) {
    // TODO: implement (might want to inline)
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
