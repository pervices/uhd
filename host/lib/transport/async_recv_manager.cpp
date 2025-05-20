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
#include <atomic>

// Include all children for use by autoselect
#include <uhd/transport/user_recv_manager.hpp>
#ifdef HAVE_LIBURING
    // Recv manager relying on liburing
    #include <uhd/transport/io_uring_recv_manager.hpp>
    // uname for checking if the kernel is recent enough for liburing
    #include <sys/utsname.h>
#endif

namespace uhd { namespace transport {

async_recv_manager::async_recv_manager( const size_t device_total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet )
:
_num_ch(recv_sockets.size()),
_recv_sockets(recv_sockets),
_header_size(header_size),
_packet_data_size(max_sample_bytes_per_packet),

_vita_header_offset(SIMD_ALIGNMENT - _header_size),
_padded_individual_packet_size(/*Data portion padded to full page*/(std::ceil((_packet_data_size) / (double)SIMD_ALIGNMENT) * SIMD_ALIGNMENT) + /* Vita header + padding */ _header_size + _vita_header_offset),

_all_ch_packet_buffers((uint8_t*) allocate_hugetlb_buffer_with_fallback(_num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size))

// Create buffer for flush complete flag in seperate cache lines
{
    if(device_total_rx_channels > MAX_CHANNELS) {
        UHD_LOG_ERROR("ASYNC_RECV_MANAGER", "Unsupported number of channels, constants must be updated");
        throw assertion_error("Unsupported number of channels");
    }

    // Set entire buffer to 0 to avoid issues with lazy allocation
    memset(_all_ch_packet_buffers, 0, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size);

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
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "A timeout occured while flushing sockets. It is likely that the device is already streaming";
                throw std::runtime_error("Timeout while flushing buffers");
            }
        }
    }
}

async_recv_manager::~async_recv_manager()
{
    // Make sure all recv threads are stopped during the child's destructor

    // Frees buffers, make sure this is after child threads stopped
    munmap(_all_ch_packet_buffers, _num_ch * PACKET_BUFFER_SIZE * _padded_individual_packet_size);
}

void* async_recv_manager::allocate_hugetlb_buffer_with_fallback(size_t size) {
    // Allocate buffer using huge pages (MAP_HUGETLB)
    void* hugeltb_buffer = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
    // If it worked return buffer
    if(hugeltb_buffer != MAP_FAILED) {
        return hugeltb_buffer;
    // Fallback to not using huge pages
    } else {
        // Recomend the user request twice and many huge pages as required in case some are used by other processes
        UHD_LOG_ERROR("ASYNC_RECV_MANAGER", "Failed to allocate buffer of size " + std::to_string(size) + " bytes using huge pages. Try increasing the value of /proc/sys/vm/nr_hugepages, starting with " + std::to_string( 2 * (size_t)std::ceil(size/HUGE_PAGE_SIZE)) + " * number of channels. Reattempting without huge pages, which may harm performance.");
        return allocate_buffer(size);
    }
}

void* async_recv_manager::allocate_buffer(size_t size) {
    // MMAP is used instead of aligned_alloc since aligned_alloc may have caused inconsistent performance
    void* buffer = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if(buffer != MAP_FAILED) {
        return buffer;
    } else {
        UHD_LOG_ERROR("ASYNC_RECV_MANAGER", "Failed to allocate buffer of size " + std::to_string(size) + "bytes. Error code: " + std::string(strerror(errno)));
        throw uhd::environment_error(std::string(strerror(errno)));
    }
}

async_recv_manager* async_recv_manager::auto_make( const size_t total_rx_channels, const std::vector<int>& recv_sockets, const size_t header_size, const size_t max_sample_bytes_per_packet ) {
// If liburing is not enable at compile time use user_recv_manager
#ifndef HAVE_LIBURING
    return user_recv_manager::make(total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet);
#else
    // TODO check if the kernel supports liburing (6.0 or later)
    bool io_uring_supported = true;

    utsname uname_info;

    int r = uname(&uname_info);

    if(r == -1) {
        // TODO throw error, uname was unable to get info
    }

    printf("system name = %s\n", uname_info.sysname);
    printf("node name   = %s\n", uname_info.nodename);
    printf("release     = %s\n", uname_info.release);
    printf("version     = %s\n", uname_info.version);
    printf("machine     = %s\n", uname_info.machine);

    if(io_uring_supported) {
        return io_uring_recv_manager::make(total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet);
    } else {
        return user_recv_manager::make(total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet);
    }

#endif
}

void async_recv_manager::auto_unmake( async_recv_manager* recv_manager ) {
    if(typeid(*recv_manager) == typeid(user_recv_manager)) {
        user_recv_manager::unmake((user_recv_manager*) recv_manager);
    } else if (typeid(*recv_manager) == typeid(io_uring_recv_manager)) {
        io_uring_recv_manager::unmake((io_uring_recv_manager*) recv_manager);
    } else {
        throw std::invalid_argument("Invalid recv manager type. This should be unreachable.");
    }
}

}}
