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
packet_size(header_size + max_sample_bytes_per_packet)
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
    stop_flag = false;

    // Have 1 page worth of packet mmsghdrs and iovec per buffer
    // NOTE: Achieving 1 mmsghdr and 1 iovec per buffer asummes iovec has a 1 element
    // TODO allocate mmsghdr and iovec to that they are on the same page
    packets_per_buffer = page_size / (sizeof(mmsghdr) + sizeof(iovec));
    std::cout << "packets_per_buffer: " << packets_per_buffer << std::endl;
    // The actual size of the buffers in bytes
    int_fast64_t actual_buffer_size = packets_per_buffer * packet_size;

    // Add padding to ensure this buffer is not on the same cache line as anything else
    actual_buffer_size += actual_buffer_size % std::hardware_destructive_interference_size;

    // Each mmmsghdr_buffer_size buffer contains both mmsghdrs and their corresponding iovec
    size_t mmmsghdr_buffer_size = (sizeof(mmsghdr) + sizeof(iovec)) * packets_per_buffer;

    std::cout << "actual_buffer_size: " << actual_buffer_size << std::endl;
    std::cout << "mmmsghdr_buffer_size: " << mmmsghdr_buffer_size << std::endl;
    std::cout << "actual_buffer_size * NUM_BUFFERS: " << actual_buffer_size * NUM_BUFFERS << std::endl;
    std::cout << "mmmsghdr_buffer_size * NUM_BUFFERS: " << mmmsghdr_buffer_size * NUM_BUFFERS << std::endl;
    // Allocates buffes to store packets and message headers
    for(size_t ch = 0; ch < recv_sockets.size(); ch++) {
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // Create buffers that are aligned to pages to reduce contention between threads
            packet_buffer_ptrs[ch][b] = (uint8_t*) aligned_alloc(page_size, actual_buffer_size);
            if(packet_buffer_ptrs[ch][b] == nullptr) {
                throw uhd::environment_error( "out of memory when attempting to allocate:" + std::to_string(actual_buffer_size) + " bytes");
            }
            // Set the data buffer to all 0, not directly required but prevents issues from lazy allocation later on
            memset(packet_buffer_ptrs[ch][b], 0, actual_buffer_size);

            // Resizes mmsghdr buffers to be able to contain all the packets
            mmmsghdr_buffer_size += mmmsghdr_buffer_size % page_size;
            mmsghdrs[ch][b] = (struct mmsghdr*) aligned_alloc(page_size, mmmsghdr_buffer_size);
            if(mmsghdrs[ch][b] == nullptr) {
                throw uhd::environment_error( "out of memory when attempting to allocate:" + std::to_string(mmmsghdr_buffer_size) + " bytes");
            }
            memset(mmsghdrs[ch][b], 0, mmmsghdr_buffer_size);
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
    std::cout << "flush complete\n";
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

    // Configure iovecs
    for(size_t ch = 0; ch < num_ch; ch++) {
        for(size_t b = 0; b < NUM_BUFFERS; b++) {
            // iovecs are stored in the same buffer as mmsghdrs, after all the msghdrs
            struct iovec* iovecs =(iovec*) &self->mmsghdrs[ch + ch_offset][b][self->packets_per_buffer];
            for(int_fast64_t p = 0; p < self->packets_per_buffer; p++) {

                // Points iovecs to the corresponding point in the buffers
                iovecs[p].iov_base = self->packet_buffer_ptrs[ch + ch_offset][b] + (p * self->packet_size);;
                iovecs[p].iov_len = self->packet_size;

                // Points mmsghdrs to the corresponding io_vec
                // Since there is only one location data should be written to per packet just take the address of the location to write to
                self->mmsghdrs[ch + ch_offset][b][p].msg_hdr.msg_iov = &iovecs[p];
                self->mmsghdrs[ch + ch_offset][b][p].msg_hdr.msg_iovlen = 1;
            }
        }
    }
    // Tracks which buffer is currently being written to by each channel
    std::vector<size_t> b(num_ch, 0);
    // Tracks if the loop has already started filling the current buffer (uint8_t
    std::vector<uint_fast8_t> b_active(num_ch, 0);


    // Memory order to use when setting flush_complete
    // Used to avoid non relaxed writes and branches
    std::vector<std::memory_order> flush_order(num_ch, std::memory_order_release);

    std::vector<std::memory_order> b_aquire_load_order(num_ch, std::memory_order_consume);

    std::memory_order stop_flag_order = std::memory_order_relaxed;

    std::vector<size_t> num_packets_received(num_ch, 0);

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
                std::cout << "Main thread slow\n";
                continue;
            }

            // Switch to relaxed memory ordering for check if the buffer is available since synchronization is no longer needed
            b_aquire_load_order[ch] = std::memory_order_relaxed;

            // Receives any packets already in the buffer
            int packets_received = recvmmsg(sockets[ch], self->mmsghdrs[ch + ch_offset][b[ch]], self->packets_per_buffer, MSG_DONTWAIT, 0);

            // If packets received
            if(packets_received >= 0) {
                num_packets_received[ch]+=packets_received;
                // Increment/multiple by flush_complete because packets should not be counted before flush is complete

                // Increment the counter for number of packets stored
                // Relaxed load is acceptable here since the only time the other thread writes to this is beore the buffer is aqquired
                // * flush_complete skips recording that packets were received until the sockets have been flushed
                self->num_packets_stored[ch + ch_offset]->at(b[ch]).store(packets_received * self->flush_complete[ch + ch_offset].load(std::memory_order_relaxed), std::memory_order_release);

                // Shift to the next buffer if there is any data ready in the buffer, loop back to first buffer
                b[ch] = (b[ch] + self->flush_complete[ch + ch_offset].load(std::memory_order_relaxed)) & (NUM_BUFFERS -1);
                // Reset flag that indicates that the buffer has been aqquired
                b_active[ch] = false;

            // EAGAIN, EWOULDBLOCK are caused by MSG_DONTWAIT when no packets are available and expected
            // EINTR is caused by the program receiving an interrupt during recvmmsg and is expected (such as the user pressing ctrl c to exit)
            } else if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                // Sets the flag to indicate that the buffers have been cleared once a recvmmsg returns nothing
                // Only sets it if not already set to avoid need for non relaxed atomic access
                self->flush_complete[ch + ch_offset].store(1, flush_order[ch]);
                flush_order[ch] = std::memory_order_relaxed;
                continue;
            } else {
                // TODO: handle case where packets were received during flush
                UHD_LOGGER_ERROR("ASYNC_RECV_MANAGER") << "Unhandled error during recvmmsg: " + std::string(strerror(errno));
                self->stop_flag = true;
                return;
            }
        }
    }
    for(size_t ch = 0; ch < num_ch; ch++) {
        std::cout << "num_packets_received[ch]: " << num_packets_received[ch] << std::endl;
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
    // TODO: optimize accessing num_packets_stored to reduce thread contention
    // Move to the next buffer once all packets in this buffer are consumed
    if(num_packets_consumed[ch] >= num_packets_stored[ch]->at(b)) {
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
