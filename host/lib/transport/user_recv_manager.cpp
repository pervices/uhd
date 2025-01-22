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
: async_recv_manager( device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet),
_mmsghdr_buffer((uint8_t*) allocate_hugetlb_buffer_with_fallback(mmghdr_buffer_size())),
_iovec_buffer((uint8_t*) allocate_hugetlb_buffer_with_fallback(iovec_buffer_size())),
_call_buffer_heads((uint8_t*) aligned_alloc(CACHE_LINE_SIZE, _num_ch * CACHE_LINE_SIZE)),
_call_buffer_tails((uint8_t*) aligned_alloc(CACHE_LINE_SIZE, _num_ch * CACHE_LINE_SIZE)),
_packets_in_call_buffer((uint8_t*) aligned_alloc(CACHE_LINE_SIZE, _num_ch * NUM_CALL_BUFFERS * CACHE_LINE_SIZE)),
_num_packets_consumed_current_buffer((uint8_t*) aligned_alloc(CACHE_LINE_SIZE, _num_ch * CACHE_LINE_SIZE))
{
    // Clear buffers
    memset(_call_buffer_heads, 0, _num_ch * CACHE_LINE_SIZE);
    memset(_call_buffer_tails, 0, _num_ch * CACHE_LINE_SIZE);
    memset(_packets_in_call_buffer, 0, _num_ch * NUM_CALL_BUFFERS * CACHE_LINE_SIZE);
    memset(_num_packets_consumed_current_buffer, 0, _num_ch * CACHE_LINE_SIZE);

    size_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    // Number of channels per thread
    // Ideally 1, but may need to be more depending on how many cores the host has
    size_t ch_per_thread = (size_t) std::ceil( ( MAX_RESOURCE_FRACTION * device_total_rx_channels) / (double)num_cores );

    init_mmsghdr_iovecs();

    // Creates thread to receive data
    for(size_t ch_offset = 0; ch_offset < _num_ch; ch_offset += ch_per_thread) {
        std::vector<int> thread_sockets(recv_sockets.begin() + ch_offset, recv_sockets.begin() + std::min(ch_offset + ch_per_thread, recv_sockets.size()));

        recv_loops.emplace_back(std::thread(recv_loop, this, thread_sockets, ch_offset));
    }
}

user_recv_manager::~user_recv_manager()
{
    // Tell recv loop to exit
    stop_flag = 1;
    // Fence to make sure the flag is updated in other threads
    _mm_sfence();

    for(size_t n = 0; n < recv_loops.size(); n++) {
        recv_loops[n].join();
    }

    munmap(_mmsghdr_buffer, mmghdr_buffer_size());
    munmap(_iovec_buffer, iovec_buffer_size());
    free(_call_buffer_heads);
    free(_call_buffer_tails);
    free(_packets_in_call_buffer);
    free(_num_packets_consumed_current_buffer);
}

void user_recv_manager::get_next_async_packet_info(const size_t ch, async_packet_info* info) {
    uint64_t* call_buffer_tail = access_call_buffer_tail(ch);

    if(*call_buffer_tail < *access_call_buffer_head(ch)) {
        size_t b = *call_buffer_tail & (NUM_CALL_BUFFERS - 1);
        size_t p = *access_num_packets_consumed_current_buffer(ch);

        info->length = *access_packet_length(ch, 0, call_to_consolidated(b, p));
        info->vita_header = access_packet_vita_header(ch, 0, call_to_consolidated(b, p));
        info->samples = access_packet_samples(ch, 0, call_to_consolidated(b, p));
    // No packets ready
    } else {
        info->length = 0;
        info->vita_header = nullptr;
        info->samples = nullptr;
    }
}

void user_recv_manager::init_mmsghdr_iovecs() {
    // Clear buffers for determinism
    memset(_mmsghdr_buffer, 0, mmghdr_buffer_size());
    memset(_iovec_buffer, 0, iovec_buffer_size());

    for(uint_fast32_t ch = 0; ch < _num_ch; ch++) {
        for(uint_fast32_t b = 0; b < NUM_CALL_BUFFERS; b++) {
            for(uint_fast32_t p = 0; p < CALL_BUFFER_SIZE; p++) {

                struct mmsghdr* current_mmsghdr = access_mmsghdr(ch, 0, b, p);
                struct iovec* current_iovec = access_iovec(ch, 0, b, p);

                // Tell the iovec where to store the packet
                // At present the samples are next to the Vita header, if that changes
                current_iovec->iov_base = access_packet_vita_header(ch, 0, call_to_consolidated(b, p));
                current_iovec->iov_len = _header_size + _packet_data_size;

                // Tell the mmsghdr which iovec to use
                current_mmsghdr->msg_hdr.msg_iov = current_iovec;
                current_mmsghdr->msg_hdr.msg_iovlen = 1;
            }
        }
    }
}

void user_recv_manager::recv_loop(user_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset) {

    // Number of channels received by this thread
    size_t ch_this_thread = sockets.size();

    // Enables use of a realtime schedueler which will prevent this program from being interrupted and causes it to be bound to a core, but will result in it's core being fully utilized
    uhd::set_thread_priority_safe(1, true);
    // Sets the affinity to the current core
    uhd::set_thread_affinity_active_core();

    // Set the thread and socket's affinity to the current core, improves speed and reliability
    unsigned int cpu;
    // Syscall used because getcpu is does not exist on Oracle
    int r = syscall(SYS_getcpu, &cpu, nullptr);
    if(!r) {
        for(uint_fast32_t ch = 0; ch < ch_this_thread; ch++) {
            std::vector<size_t> target_cpu(1, cpu);
            set_thread_affinity(target_cpu);

            r = setsockopt(sockets[ch], SOL_SOCKET, SO_INCOMING_CPU, &cpu, sizeof(cpu));
            if(r) {
                UHD_LOG_WARNING("USER_RECV_MANAGER", "Unable to set socket affinity. Error code: " + std::string(strerror(errno)));
            }
        }
    } else {
        UHD_LOG_WARNING("USER_RECV_MANAGER", "getcpu failed, unable to set receive socket affinity to current core. Performance may be impacted. Error code: " + std::string(strerror(errno)));
    }

    while(!self->stop_flag) [[likely]] {
        for(size_t ch = 0; ch < ch_this_thread; ch++) {
            uint64_t* call_buffer_head = self->access_call_buffer_head(ch, ch_offset);

            // The call buffer currently in use
            uint64_t b = *call_buffer_head & (NUM_CALL_BUFFERS - 1);

            // Load fence to make sure getting the call buffer head and stop flags don't get optimized out
            _mm_lfence();

            // Check if the next call buffer in the ring buffer of call buffers is free
            if(*call_buffer_head >= *self->access_call_buffer_tail(ch, ch_offset) + NUM_CALL_BUFFERS) {
                // Skips to the next channel if said buffer is still in use
                continue;
            }

            int r = recvmmsg(sockets[ch], self->access_mmsghdr(ch, ch_offset, b, 0), CALL_BUFFER_SIZE, MSG_DONTWAIT, 0);

            // No packets ready, continue to next channel
            // EAGAIN, EWOULDBLOCK indicate MSG_DONTWAIT was used and no packets were ready and can be ignored
            // EINTR indicates the program received a signal while the recvmmsg call was in progress and also can be ignored
            if(r == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                continue;
            } else if(r == -1) {
                UHD_LOG_ERROR("USER_RECV_MANAGER", "Unexpected error during recvmmsg: " + std::string(strerror(errno)));
            }

            // Records how many packets are in the current call buffer
            *self->access_packets_in_call_buffer(ch, ch_offset, b) = (uint64_t) r;

            // Copy the length of the packet received to where the parent class expects it to be stored
            // This can probably be optimized. It is being done this way to maximize similarities with io_uring_recv_manager
            for(size_t p = 0; p < (size_t) r; p++) {
                *self->access_packet_length(ch, ch_offset, call_to_consolidated(b, p)) = self->access_mmsghdr(ch, ch_offset, b, p)->msg_len;
            }

            // Store fence to ensure the above writes are completed before the call buffer is marked as ready
            _mm_sfence();

            // Advance to the next call buffer
            (*call_buffer_head)++;
        }
    }
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
