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
packet_buffer_ptrs(recv_sockets.size(), std::vector<uint8_t*>(NUM_BUFFERS, nullptr)),
mmsghdrs(recv_sockets.size(), std::vector<struct mmsghdr*>(NUM_BUFFERS)),
flush_complete(recv_sockets.size()),
num_packets_stored(recv_sockets.size()),
known_num_packets_stored(recv_sockets.size(), 0),
active_consumer_buffer(recv_sockets.size(), 0),
num_packets_consumed(recv_sockets.size(), 0),
packet_size(header_size + max_sample_bytes_per_packet),
recv_rings(recv_sockets.size())
{
    const size_t page_size = getpagesize();
    // Initializing atomic variables
    for(size_t ch = 0; ch < num_packets_stored.size(); ch++) {
        flush_complete[ch] = 0;
        // Make each channel's counters page aligned to reduce contention between threads
        num_packets_stored[ch] = new(std::align_val_t(page_size)) std::vector<std::atomic<int_fast64_t>>(NUM_BUFFERS);
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            num_packets_stored[ch]->at(b) = 0;
        }
    }
    recv_error = 0;
    stop_flag = false;

    // Gets the amount of RAM the system has
    const int_fast64_t system_ram = get_phys_pages() * page_size;

    const int_fast64_t max_buffer_size = std::min((int_fast64_t) ((double) system_ram / (total_rx_channels * MAX_RESOURCE_FRACTION * NUM_BUFFERS)), HARD_MAX_BUFFER_SIZE);
    packets_per_buffer = std::max(max_buffer_size / packet_size, HARD_MIN_BUFFER_SIZE_PACKETS);
    // The actual size of the buffers in bytes
    int_fast64_t actual_buffer_size = packets_per_buffer * packet_size;

    // Add padding to ensure this buffer is not on the same cache line as anything else
    actual_buffer_size += actual_buffer_size % std::hardware_destructive_interference_size;

    // Creates setting for liburing
    struct io_uring_params uring_params;
    memset(&uring_params, 0, sizeof(io_uring_params));

    // Number of entries that can fit in the submission queue
    uring_params.sq_entries = MAX_IO_RING_ENTRIES;
    // Number of entries that can fit in the completion queue
    uring_params.cq_entries = 2 * MAX_IO_RING_ENTRIES;
    // IORING_SETUP_IOPOLL: use busy poll instead of interrupts - only implemented for storage devices so far
    // TODO: figure out how to get IORING_SETUP_IOPOLL working
    // IORING_SETUP_SQPOLL: allows io_uring_submit to skip syscall
    // IORING_SETUP_SINGLE_ISSUER: hint to the kernel that only 1 thread will submit requests
    // IORING_FEAT_NODROP: don't drop events even if the completion queue is full (will result in a performance hit when the kernel needs to resize related buffer)
    uring_params.flags = /*IORING_SETUP_IOPOLL |*/ IORING_SETUP_SQPOLL | IORING_SETUP_SINGLE_ISSUER | IORING_FEAT_NODROP;
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

    // Allocates buffes to store packets, message headers and io rings
    for(size_t ch = 0; ch < recv_sockets.size(); ch++) {
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // Create buffers that are aligned to pages to reduce contention between threads
            // TODO: change from page aligned to aligned with cache line size (std::hardware_destructive_interference_size)
            packet_buffer_ptrs[ch][b] = (uint8_t*) aligned_alloc(page_size, actual_buffer_size);
            if(packet_buffer_ptrs[ch][b] == nullptr) {
                throw uhd::environment_error( "out of memory when attempting to allocate:" + std::to_string(actual_buffer_size) + " bytes");
            }
            // Set the data buffer to all 0, not directly required but prevents issues from lazy allocation later on
            memset(packet_buffer_ptrs[ch][b], 0, actual_buffer_size);

            // Resizes mmsghdr buffers to be able to contain all the packets
            size_t mmmsghdr_buffer_size = sizeof(struct mmsghdr) * packets_per_buffer;
            mmmsghdr_buffer_size += mmmsghdr_buffer_size % std::hardware_destructive_interference_size;
            mmsghdrs[ch][b] = (struct mmsghdr*) aligned_alloc(page_size, mmmsghdr_buffer_size);
            if(mmsghdrs[ch][b] == nullptr) {
                throw uhd::environment_error( "out of memory when attempting to allocate:" + std::to_string(mmmsghdr_buffer_size) + " bytes");
            }
            memset(mmsghdrs[ch][b], 0, mmmsghdr_buffer_size);
        }

        // Allocate io_ring
        // pad io_ring to avoid false sharing
        size_t padded_io_ring_size = (size_t) (ceil(((double)sizeof(io_uring))/ std::hardware_destructive_interference_size) * std::hardware_destructive_interference_size);
        recv_rings[ch] = (io_uring*) aligned_alloc(std::hardware_destructive_interference_size, padded_io_ring_size);
        memset(recv_rings[ch], 0, padded_io_ring_size);

        int error = io_uring_queue_init_params(MAX_IO_RING_ENTRIES, recv_rings[ch], &uring_params);
        if(error) {
            UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Error when creating io_uring: " << strerror(-error);
            throw uhd::system_error("io_uring error");
        }
    }

    int64_t num_cores = std::thread::hardware_concurrency();
    // If unable to get number of cores assume the system is 4 core
    if(num_cores == 0) {
        num_cores = 4;
    }

    int64_t ch_per_thread = (int64_t) ceil( ( MAX_RESOURCE_FRACTION * total_rx_channels) / (double)num_cores );

    recv_loops.resize((int64_t) ceil( recv_sockets.size() / (double) ch_per_thread ));

    // Creates thread to receive data
    size_t ch_offset = 0;
    for(size_t n = 0; n < recv_loops.size(); n++) {
        std::vector<int> thread_sockets(recv_sockets.begin() + ch_offset, recv_sockets.begin() + std::min(ch_offset + ch_per_thread, recv_sockets.size()));

        recv_loops[n] = std::thread(recv_loop, this, thread_sockets, ch_offset);

        ch_offset+=ch_per_thread;
    }

    uhd::time_spec_t start = uhd::get_system_time();
    for(size_t n = 0; n < flush_complete.size(); n++) {
        while(!flush_complete[n]) {
            if(start + 30.0 < uhd::get_system_time()) {
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "A timeout occured while flushing sockets. It is likely that the device is already streaming";
                throw std::runtime_error("Timeout while flushing buffers");
            }
        }
    }
}

async_recv_manager::~async_recv_manager()
{
    stop_flag = true;
    for(size_t n = 0; n < recv_loops.size(); n++) {
        recv_loops[n].join();
    }

    // Frees packets and mmsghdr buffers
    for(size_t ch = 0; ch < packet_buffer_ptrs.size(); ch++) {
        free(num_packets_stored[ch]);
        free(recv_rings[ch]);
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            free(packet_buffer_ptrs[ch][b]);
            free(mmsghdrs[ch][b]);
        }
    }
}

void async_recv_manager::recv_loop(async_recv_manager* self, const std::vector<int> sockets, const size_t ch_offset) {
    // Enables use of a realtime schedueler which will prevent this program from being interrupted, but will result in it's core being fully utilized
    uhd::set_thread_priority_safe();

    size_t num_ch = sockets.size();
    std::vector<std::vector<std::vector<struct iovec>>> iovecs(num_ch, std::vector<std::vector<struct iovec>>(NUM_BUFFERS, std::vector<struct iovec>(self->packets_per_buffer)));

    for(size_t ch = 0; ch < num_ch; ch++) {
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            for(int_fast64_t p = 0; p < self->packets_per_buffer; p++) {
                // Points iovecs to the corresponding point in the buffers
                uint8_t* tmp = self->packet_buffer_ptrs[ch + ch_offset][b] + (p * self->packet_size);
                iovecs[ch][b][p].iov_base = tmp;
                iovecs[ch][b][p].iov_len = self->packet_size;

                // Points mmsghdrs to the corresponding io_vec
                // Since there is only one location data should be written to per packet just take the address of the location to write to
                self->mmsghdrs[ch + ch_offset][b][p].msg_hdr.msg_iov = &iovecs[ch][b][p];
                self->mmsghdrs[ch + ch_offset][b][p].msg_hdr.msg_iovlen = 1;
            }
        }
    }
    // Tracks which buffer is currently being written to by each channel
    std::vector<size_t> b(num_ch, 0);
    // Tracks if the loop has already started filling the current buffer (uint8_t
    std::vector<uint_fast8_t> b_active(num_ch, 0);

    uint32_t max_packets;
    if(num_ch == 1) {
        max_packets = UINT32_MAX;
    } else {
        // Limit maximum number of packets received per call to prevent one channel's receive from taking to long and causing another packet to drop
        max_packets = MAX_PACKETS_PER_RECVMMSG;
    }

    // Memory order to use when setting flush_complete
    // Used to avoid non relaxed writes and branches
    std::vector<std::memory_order> flush_order(num_ch, std::memory_order_release);

    std::vector<std::memory_order> b_aquire_load_order(num_ch, std::memory_order_consume);

    std::memory_order stop_flag_order = std::memory_order_relaxed;

    uint_fast8_t loop_counts = 0;
    while(!self->stop_flag.load(stop_flag_order)) {
        // Use atomic check for stop flag every 0x3f samples to balance need to update flag and avoiding interprocess communication
        loop_counts++;
        stop_flag_order = (std::memory_order) ((size_t) (std::memory_order_consume * (loop_counts >> 6)) + (size_t) std::memory_order_relaxed - (size_t) (std::memory_order_relaxed * (loop_counts >> 6)));
        loop_counts &= 0x3f;

        for(size_t ch = 0; ch < num_ch; ch++) {

            // Aquire the buffer if not already aquired
            // Buffer can be aquired if it is empty
            b_active[ch] = !(self->num_packets_stored[ch + ch_offset]->at(b[ch]).load(b_aquire_load_order[ch])) || b_active[ch];

            if(!b_active[ch]) {
                // If unable to aquire buffer, switch to atomic checks to see future changes immediately
                b_aquire_load_order[ch] = std::memory_order_consume;
                continue;
            }

            // Switch to relaxed memory ordering for check if the buffer is available since synchronization is no longer needed
            b_aquire_load_order[ch] = std::memory_order_relaxed;

            // memory_order_relaxed is used since only this thread is writing to this variable once a buffer is activ
            int_fast64_t packets_in_buffer = self->num_packets_stored[ch + ch_offset]->at(b[ch]).load(std::memory_order_relaxed);

            // Number of packets to be received this recvmmsge
            uint32_t packets_to_recv = (uint32_t) std::min(self->packets_per_buffer - packets_in_buffer, (int_fast64_t) max_packets);

            // Receives any packets already in the buffer
            int packets_received = recvmmsg(sockets[ch], &self->mmsghdrs[ch + ch_offset][b[ch]][packets_in_buffer], packets_to_recv, MSG_DONTWAIT, 0);

            // If packets received
            if(packets_received >= 0) {
                // Increment the counter for number of packets stored
                // Relaxed load is acceptable here since the only time the other thread writes to this is beore the buffer is aqquired
                // * flush_complete skips recording that packets were received until the sockets have been flushed
                self->num_packets_stored[ch + ch_offset]->at(b[ch]).store(self->num_packets_stored[ch + ch_offset]->at(b[ch]).load(std::memory_order_relaxed) + (packets_received * self->flush_complete[ch + ch_offset].load(std::memory_order_relaxed)), std::memory_order_release);

                 if(packets_in_buffer + packets_received >= self->packets_per_buffer) {

                    // This buffer is full, shift to the next one, loop back to first buffer
                    b[ch] = (b[ch] + 1) & (NUM_BUFFERS -1);
                    // Reset flag that indicates that the buffer has been aqquired
                    b_active[ch] = false;
                }

            // EAGAIN, EWOULDBLOCK are caused by MSG_DONTWAIT when no packets are available and expected
            // EINTR is caused by the program receiving an interrupt during recvmmsg and is expected (such as the user pressing ctrl c to exit)
            } else if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                // Sets the flag to indicate that the buffers have been cleared once a recvmmsg returns nothing
                // Only sets it if not already set to avoid need for non relaxed atomic access
                self->flush_complete[ch + ch_offset].store(1, flush_order[ch]);
                flush_order[ch] = std::memory_order_relaxed;
                continue;
            } else {
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error during recvmmsg: " + std::string(strerror(errno));
                self->recv_error = errno;
                self->stop_flag = true;
                return;
            }
        }
    }
}

uint8_t* async_recv_manager::get_next_packet(const size_t ch) {
    size_t b = active_consumer_buffer[ch];
    // Check if this thread already knows that the next packet is ready (reduces inter-thread communication)
    if(known_num_packets_stored[ch] > num_packets_consumed[ch]) {
        return packet_buffer_ptrs[ch][b] + (packet_size * num_packets_consumed[ch]);
    }
    // Check the variable set by the other thread if the next packet is ready
    else if(num_packets_stored[ch]->at(b) > num_packets_consumed[ch]) {
        known_num_packets_stored[ch] = num_packets_stored[ch]->at(b).load(std::memory_order_relaxed);
        return packet_buffer_ptrs[ch][b] + (packet_size * num_packets_consumed[ch]);
    }
    else {
        return nullptr;
    }
}

uint32_t async_recv_manager::get_next_packet_length(const size_t ch) {
    size_t b = active_consumer_buffer[ch];
    return mmsghdrs[ch][b][num_packets_consumed[ch]].msg_len;
}

void async_recv_manager::advance_packet(const size_t ch) {
    size_t b = active_consumer_buffer[ch];
    num_packets_consumed[ch]++;
    if(num_packets_consumed[ch] >= packets_per_buffer) {
        // Marks this buffer as clear
        num_packets_stored[ch]->at(b).store(0, std::memory_order_release);

        // Moves to the next buffer
        // & is to roll over the the first buffer once the limit is reached
        active_consumer_buffer[ch] = (active_consumer_buffer[ch] + 1) & (NUM_BUFFERS -1);

        // Reset the known (for this thread) packets stored counter for this buffer
        known_num_packets_stored[ch] = 0;

        // Resets count for number of samples consumed in the active buffer
        num_packets_consumed[ch] = 0;
    }
}

}}
