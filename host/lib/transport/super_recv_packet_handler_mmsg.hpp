//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2023-2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP
#define INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP

#include "../../transport/super_recv_packet_handler.hpp"
#include <uhd/config.hpp>
#include <uhd/convert.hpp>
#include <uhd/exception.hpp>
#include <uhd/stream.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhdlib/utils/performance_mode.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>

#include <error.h>
#include <uhd/exception.hpp>
#include <sys/ioctl.h>
#include <net/if.h>

// TODO: add cmake stuff and dependancies to make sure this works on all systems
#include <liburing.h>
// liburing notes
// start with io_uring_prep_recvmsg for simplicity
// Implement linking to force the correct order: IOSQE_IO_LINK

#define MIN_MTU 9000

namespace uhd { namespace transport { namespace sph {

    // Socket priority for rx sockets
    // One less than the socket priority of tx
    const int RX_SO_PRIORITY = 5;

/***********************************************************************
 * Super receive packet handler
 *
 * A receive packet handler represents a group of channels.
 * The channel group shares a common sample rate.
 * All channels are received in unison in recv().
 **********************************************************************/
class recv_packet_handler_mmsg : public recv_packet_handler
{
public:

    size_t sqe_requests = 0;
    size_t cqe_seen = 0;

    /*!
     * Make a new packet handler for receive
     * \param dst_ip the IPV4 desination IP address of packets for each channe;
     * \param dst_port the detination port of packet for each channel
     * \param max_sample_bytes_per_packet the number of transport channels
     * \param header_size the number of transport channels
     * \param cpu_format datatype of samples on the host system
     * \param wire_format datatype of samples in the packets
     * \param wire_little_endian true if the device is configured to send little endian data. If cpu_format == wire_format and wire_little_endian no converter is required, boosting performance
     */
    recv_packet_handler_mmsg(const std::vector<std::string>& dst_ip, std::vector<int>& dst_port, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian)
    : recv_packet_handler(max_sample_bytes_per_packet + header_size), _NUM_CHANNELS(dst_ip.size()), _MAX_SAMPLE_BYTES_PER_PACKET(max_sample_bytes_per_packet),
    _HEADER_SIZE(header_size),
    _TRAILER_SIZE(trailer_size),
    _intermediate_recv_buffer_pointers(_NUM_CHANNELS),
    _intermediate_recv_buffer_wrapper(_intermediate_recv_buffer_pointers.data(), _NUM_CHANNELS)
    {
        printf("_MAX_SAMPLE_BYTES_PER_PACKET: %lu\n", _MAX_SAMPLE_BYTES_PER_PACKET);
        if (wire_format=="sc16") {
            _BYTES_PER_SAMPLE = 4;
        } else if (wire_format=="sc12") {
            _BYTES_PER_SAMPLE = 3;
        } else {
            throw uhd::runtime_error( "Unsupported wire format:" + wire_format);
        }

        // Creates and binds to sockets
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            struct sockaddr_in dst_address;
            int recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if(recv_socket_fd < 0) {
                throw uhd::runtime_error( "Failed to create recv socket. Error code:" + std::string(strerror(errno)));
            }

            dst_address.sin_family = AF_INET;
            dst_address.sin_addr.s_addr = inet_addr(dst_ip[n].c_str());
            dst_address.sin_port = htons(dst_port[n]);

            // The OS takes an indeterminate amount of time to unbind addresses from previous runs
            // As a workaround, repeatedly try to bind the port until the timeout is reached
            struct timespec bind_timeout_time;
            clock_gettime(CLOCK_MONOTONIC_COARSE, &bind_timeout_time);
            bind_timeout_time.tv_sec+=30;
            struct timespec current_time;
            // Stores the return value of bind, used to check for errors
            int bind_r;
            do {
                bind_r = bind(recv_socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address));
                // bind_r >= 0: bind succeeded
                // errno != EADDRINUSE: the issue causing bind to fail is not related to old binds not being cleaned up by the OS,
                // stop the loop since retrying won't fix it
                if(bind_r >= 0 || errno != EADDRINUSE) {
                    break;
                }
                ::usleep(5000);
                clock_gettime(CLOCK_MONOTONIC_COARSE, &current_time);
            } while (current_time.tv_sec < bind_timeout_time.tv_sec || (current_time.tv_sec == bind_timeout_time.tv_sec && current_time.tv_nsec < bind_timeout_time.tv_nsec));

            if(bind_r < 0)
            {
                fprintf(stderr, "ERROR Unable to bind to IP address %s and port %i\n", dst_ip[n].c_str(), dst_port[n]);
                if(errno == EADDRINUSE) {
                    fprintf(stderr, "Address already in use. This is usually caused by attempting to run multiple UHD programs at once\n");
                } else {
                    fprintf(stderr, "Bind failed with error: %s\n", strerror(errno));
                }
            }

            // Sets the recv buffer size
            setsockopt(recv_socket_fd, SOL_SOCKET, SO_RCVBUF, &_DEFAULT_RECV_BUFFER_SIZE, sizeof(_DEFAULT_RECV_BUFFER_SIZE));

            // Checks the recv buffer size
            socklen_t opt_len = sizeof(_ACTUAL_RECV_BUFFER_SIZE);
            getsockopt(recv_socket_fd, SOL_SOCKET, SO_RCVBUF, &_ACTUAL_RECV_BUFFER_SIZE, &opt_len);

            // NOTE: The kernel will set the actual size to be double the requested. So the expected amount is double the requested
            if(_ACTUAL_RECV_BUFFER_SIZE < 2*_DEFAULT_RECV_BUFFER_SIZE) {
                fprintf(stderr, "Unable to set recv buffer size. Performance may be affected\nTarget size %i\nActual size %i\nPlease run \"sudo sysctl -w net.core.rmem_max=%i\"\n", _DEFAULT_RECV_BUFFER_SIZE, _ACTUAL_RECV_BUFFER_SIZE/2, _DEFAULT_RECV_BUFFER_SIZE);
                throw uhd::system_error("Unable to set recv socket size");
            }

            int mtu = get_mtu(recv_socket_fd, dst_ip[n].c_str());
            if(mtu < MIN_MTU) {
                fprintf(stderr, "MTU of interface associated with %s is to small. %i required, current value is %i", dst_ip[n].c_str(), MIN_MTU, mtu);
                throw uhd::system_error("MTU size to small");
            }

            int set_priority_ret = setsockopt(recv_socket_fd, SOL_SOCKET, SO_PRIORITY, &RX_SO_PRIORITY, sizeof(RX_SO_PRIORITY));
            if(set_priority_ret) {
                fprintf(stderr, "Attempting to set rx socket priority failed with error code: %s", strerror(errno));
            }

            // recvmmsg should attempt to recv at most the amount to fill 1/_NUM_CHANNELS of the socket buffer
            _MAX_PACKETS_TO_RECV = (int)((_ACTUAL_RECV_BUFFER_SIZE/(_NUM_CHANNELS + 1))/(_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET + 42));

            recv_sockets.push_back(recv_socket_fd);

            struct io_uring_params uring_params;
            memset(&uring_params, 0, sizeof(io_uring_params));

            // TODO: optimize this. 100 results in everything being extremely slow
            const int NUM_ENTRIES = 20;
            // Number of entries that can fit in the submission queue
            uring_params.sq_entries = NUM_ENTRIES;
            // Number of entries that can fit in the completion queue
            uring_params.cq_entries = NUM_ENTRIES;
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

            struct io_uring ring;
            // NOTE: allow for more entires in ring buffer than needed in case it takes a while to acknowledge that we are finished with an entry
            int error = io_uring_queue_init_params(NUM_ENTRIES, &ring, &uring_params);
            if(error) {
                fprintf(stderr, "Error when creating io_uring: %s\n", strerror(-error));
                throw uhd::system_error("io_uring error");
            }

            io_rings.push_back(ring);
        }

        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            ch_recv_buffer_info tmp = {
                (size_t) 0, // sample_cache_used
                std::vector<std::vector<int8_t>>(_num_header_buffers, std::vector<int8_t>(_HEADER_SIZE, 0)), // headers
                std::vector<vrt::if_packet_info_t>(_num_header_buffers), // vrt_metadata
                sequence_number_mask, //previous_sequence_number
                std::vector<size_t>(_num_header_buffers, 0), // data_bytes_from_packet
                std::vector<int8_t>(_MAX_SAMPLE_BYTES_PER_PACKET, 0), // sample_cache
                (size_t) 0, // previous_sample_cache_used
                (size_t) _num_header_buffers, // max_num_packets_to_flush
                std::vector<std::vector<int8_t>>(_num_header_buffers, std::vector<int8_t>(_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET, 0)), // flush_buffer
                std::vector<mmsghdr>(_num_header_buffers),
                std::vector<iovec>(2*_num_header_buffers+1),
                std::vector<int8_t>(0)
            };
            // Sets all mmsghdrs to 0, to avoid both non-deterministic behaviour and slowdowns from lazy memory allocation
            memset(tmp.msgs.data(), 0, sizeof(mmsghdr)*_num_header_buffers);
            // Contains data about where to store received data
            // Alternating between pointer to header, pointer to data
            memset(tmp.iovecs.data(), 0, sizeof(iovec)*tmp.iovecs.size());

            ch_recv_buffer_info_group.push_back(tmp);
        }

        // Empties the socket buffer
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            flush_packets(n, 0, true);
        }

        setup_converter(cpu_format, wire_format, wire_little_endian);

        // Check if any core is not set to performance mode, used to decide if an info message should be printed if overflows occur
        _using_performance_governor = true;
        std::vector<std::string> governors = uhd::get_performance_governors();
        for(auto& g : governors) {
            if(g.find("performance") == std::string::npos) {
                _using_performance_governor = false;
                break;
            }
        }
    }

    ~recv_packet_handler_mmsg(void)
    {
        for(size_t n = 0; n < recv_sockets.size(); n++) {
            int r = close(recv_sockets[n]);
            if(r) {
                fprintf(stderr, "close failed on data receive socket with: %s\nThe program may not have closed cleanly\n", strerror(errno));
            }
            // TODO: figure out order of queue_exit and close
            io_uring_queue_exit(&io_rings[n]);
        }
    }

    /*******************************************************************
     * Receive:
     * The entry point for the fast-path receive calls.
     * Dispatch into combinations of single packet receive calls.
     ******************************************************************/
    UHD_INLINE size_t recv(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)
    {
        // Clears the metadata struct, theoretically not required but included to make sure mistakes don't cause non-deterministic behaviour
        metadata.reset();

        size_t bytes_per_buff = nsamps_per_buff * _BYTES_PER_SAMPLE;

        // If no converter is required data will be written directly into buffs, otherwise it is written to an intermediate buffer
        const uhd::rx_streamer::buffs_type *recv_buffer = (converter_used) ? prepare_intermediate_buffers(bytes_per_buff) : &buffs;

        std::vector<size_t> nsamps_received(_NUM_CHANNELS, 0);

        // Number of bytes copied from the cache (should be the same for every channel)
        size_t cached_bytes_to_copy = 0;
        // Number of samples to copy, this will be reduced by the number cached samples copied
        size_t bytes_to_recv = bytes_per_buff;
        // Copies the data from the sample cache
        for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
            // Channel info group for this ch
            ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];
            // Copies cached data from previous recv
            cached_bytes_to_copy = std::min(ch_recv_buffer_info_i.sample_cache_used, bytes_per_buff);
            memcpy((*recv_buffer)[ch], ch_recv_buffer_info_i.sample_cache.data(), cached_bytes_to_copy);
            // How many bytes still need to be received after copying from the cache
            size_t remaining_nbytes_per_buff = bytes_per_buff - cached_bytes_to_copy;

            if(one_packet && 0) {
                bytes_to_recv = std::min(remaining_nbytes_per_buff, _MAX_SAMPLE_BYTES_PER_PACKET - cached_bytes_to_copy);
            } else {
                bytes_to_recv = remaining_nbytes_per_buff;
            }
            // Indicates that the cache is clear
            ch_recv_buffer_info_i.sample_cache_used-= cached_bytes_to_copy;

            nsamps_received[ch] += cached_bytes_to_copy / _BYTES_PER_SAMPLE;
        }

        // Returns the number of samples requested, if there were enough samples in the cache
        if(!bytes_to_recv || cached_end_of_burst) {
            metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;

            // Sets the end of burst flag if cached, the clears it
            metadata.end_of_burst = cached_end_of_burst;
            cached_end_of_burst = false;

            metadata.has_time_spec = true;
            metadata.time_spec = previous_timestamp + time_spec_t::from_ticks(previous_num_samples, _sample_rate);
            previous_timestamp = metadata.time_spec;

            previous_num_samples = nsamps_per_buff;
            return nsamps_per_buff;
        }

        // Receives packets, data is stores in recv_buffer, metadata is stored in ch_recv_buffer_info.headers
        metadata.error_code = recv_multiple_packets(*recv_buffer, cached_bytes_to_copy, cached_bytes_to_copy + bytes_to_recv, timeout);

        // Setting metadata fields

        metadata.has_time_spec = true;
        metadata.time_spec = time_spec_t::from_ticks(ch_recv_buffer_info_group[0].vrt_metadata[0].tsf - /* Simulate what timestamp would'bve been if it was for the start of the cached samples */cached_bytes_to_copy, _sample_rate);
        previous_timestamp = metadata.time_spec;

        // Check for overflow errors and flushed packets to keep buffers aligned
        size_t aligned_bytes = align_buffs(metadata.error_code) + cached_bytes_to_copy;


        metadata.end_of_burst = detect_end_of_burst();

        size_t final_nsamps = aligned_bytes/_BYTES_PER_SAMPLE;

        if(converter_used) {
            convert_samples(buffs, final_nsamps);
        }

        previous_num_samples = final_nsamps;

        return final_nsamps;
    }

    // Set the rate of samples per second
    void set_sample_rate(const double rate)
    {
        _sample_rate = rate;
    }

protected:
    size_t _NUM_CHANNELS;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _BYTES_PER_SAMPLE;

    virtual void if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

private:
    // TODO dynamically adjust recv buffer size based on system RAM, number of channels, and maximum sample rate. Should be capable of storing 100ms of data
    // Desired recv buffer size
    const int _DEFAULT_RECV_BUFFER_SIZE = 500000000;
    // Actual recv buffer size, not the Kernel will set the real size to be double the requested
    int _ACTUAL_RECV_BUFFER_SIZE;
    // Maximum number of packets to recv (should be able to fit in the half the real buffer)
    int _MAX_PACKETS_TO_RECV;
    size_t _HEADER_SIZE;
    // Trailer is not needed for anything so receive will discard it
    size_t _TRAILER_SIZE;
    // Number of existing header buffers, which is the current maximum number of packets to receive at a time
    size_t _num_header_buffers = 32;
    std::vector<int> recv_sockets;
    // Stores uring rings
    std::vector<io_uring> io_rings;
    size_t previous_sample_cache_used = 0;
    // Maximum sequence number
    const size_t sequence_number_mask = 0xf;
    // Stores if end of burst
    bool cached_end_of_burst = false;

    // Number of samples in the previous recv. Used for simulating timestamp for cached samples
    time_spec_t previous_timestamp = time_spec_t(0.0);
    size_t previous_num_samples = 0;

    // Stores information about packets received for each channel
    // Note: this is not meant to be persistent between reads, it is done this way to avoid deallocating and reallocating memory
    struct ch_recv_buffer_info {
        // Stores number of headers used in this recv
        size_t num_headers_used;
        // Stores the headers of each packet
        std::vector<std::vector<int8_t>> headers;
        // Metadata contained in vrt header;
        std::vector<vrt::if_packet_info_t> vrt_metadata;
        // Sequence number from the last packet processed
        size_t previous_sequence_number;
        //Stores how many bytes of sample data are from each packet
        std::vector<size_t> data_bytes_from_packet;
        // Stores extra data from packets between recvs
        std::vector<int8_t> sample_cache;
        // Stores amount of extra data cached from previous recv in byte
        size_t sample_cache_used;
        // Maximum number of packets that can be flushed at a time
        size_t max_num_packets_to_flush;
        // Dummy recv buffer to be used when flushing
        std::vector<std::vector<int8_t>> flush_buffer;
        // Stores data about the recv for each packet
        std::vector<mmsghdr> msgs;
        /// Pointers to where to store recveived data in
        std::vector<iovec> iovecs;

        // Buffer used to store data before converting from wire format to CPU format. Unused if wire and CPU format match
        std::vector<int8_t> intermediate_recv_buffer;

    };
    // Group of recv info for each channels
    std::vector<ch_recv_buffer_info> ch_recv_buffer_info_group;

    // Whether or not a conversion is required between CPU and wire formats
    bool converter_used;

    // Pointers to the start of the recv buffer for each channel
    std::vector<void*> _intermediate_recv_buffer_pointers;
    // Wrapper to be use the same dataype as the regular buffer
    uhd::rx_streamer::buffs_type _intermediate_recv_buffer_wrapper;

    // Converts samples between wire and cpu formats
    uhd::convert::converter::sptr _converter;

    // Sample rate in samples per second
    double _sample_rate = 0;

    // Stores whether or not the CPU governor is set to performance mode
    // NOTE: getting this is done at the start, but the warning related to it only prints during streaming, assumes the governor does not change while hte program is running
    bool _using_performance_governor;
    // The warning for using non performance governor has already been printed
    bool _performance_warning_printed = false;

        /*******************************************************************
     * recv_multiple_packets:
     * receives multiple packets on a given channel
     * sample_buffer: vector of pointer to the start of the recv buffer for each channel
     * sample_buffer_offset: offset of where to start writing to in the recv buffers
     * buffer_length_bytes: size of recv buffers (inluding before the offset
     * timeout: timeout, not implemented yet
     * returns error code
     ******************************************************************/
    UHD_INLINE uhd::rx_metadata_t::error_code_t recv_multiple_packets(const uhd::rx_streamer::buffs_type& sample_buffers, size_t sample_buffer_offset, size_t buffer_length_bytes, double timeout) {

        size_t nbytes_to_recv = buffer_length_bytes - sample_buffer_offset;

        // Pointers for where to write samples to from each packet using scatter gather
        std::vector<std::vector<void*>> samples_sg_dst(_NUM_CHANNELS);

        for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
            ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];
            // Clears number of headers (which is also a count of number of packets received
            ch_recv_buffer_info_i.num_headers_used = 0;
            // Resets the count for amount of data in the cache
            ch_recv_buffer_info_i.sample_cache_used = 0;

            //Fills the pointer to where in the buffer to write samples from the packet to
            for(size_t p = sample_buffer_offset; p < buffer_length_bytes; p += _MAX_SAMPLE_BYTES_PER_PACKET) {
                samples_sg_dst[ch].push_back(p+(uint8_t*)(sample_buffers[ch]));
            }
        }

        size_t num_packets_to_recv = samples_sg_dst[0].size();

        // Adds more room to store headers if required
        if(num_packets_to_recv > _num_header_buffers) {
            _num_header_buffers = num_packets_to_recv;
            for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
                ch_recv_buffer_info_i.headers.resize(_num_header_buffers, std::vector<int8_t>(_HEADER_SIZE, 0));
                ch_recv_buffer_info_i.vrt_metadata.resize(_num_header_buffers);
                ch_recv_buffer_info_i.data_bytes_from_packet.resize(_num_header_buffers, 0);
                ch_recv_buffer_info_i.msgs.resize(_num_header_buffers);
                // Sets all mmsghdrs to 0, to avoid both non-deterministic behaviour and slowdowns from lazy memory allocation
                memset(ch_recv_buffer_info_i.msgs.data(), 0, sizeof(mmsghdr)*_num_header_buffers);
                // Contains data about where to store received data
                // Alternating between pointer to header, pointer to data
                ch_recv_buffer_info_i.iovecs.resize(2*_num_header_buffers+1);
                memset(ch_recv_buffer_info_i.iovecs.data(), 0, sizeof(iovec)*ch_recv_buffer_info_i.iovecs.size());
            }
        }

        // Amount of data in the last packet copied directly to buffer
        // Amount of data stored in the cache betwen recv
        size_t expected_excess_data_in_last_packet = num_packets_to_recv * _MAX_SAMPLE_BYTES_PER_PACKET - nbytes_to_recv;
        size_t expected_data_in_last_packet = _MAX_SAMPLE_BYTES_PER_PACKET - expected_excess_data_in_last_packet;

        for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
            ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];
            for (size_t n = 0; n < num_packets_to_recv - 1; n++) {
                // Location to write header data to
                ch_recv_buffer_info_i.iovecs[2*n].iov_base = 0;
                ch_recv_buffer_info_i.iovecs[2*n].iov_base = ch_recv_buffer_info_i.headers[n].data();
                ch_recv_buffer_info_i.iovecs[2*n].iov_len = _HEADER_SIZE;
                ch_recv_buffer_info_i.iovecs[2*n+1].iov_base = 0;
                // Location to write sample data to
                ch_recv_buffer_info_i.iovecs[2*n+1].iov_base = samples_sg_dst[ch][n];
                ch_recv_buffer_info_i.iovecs[2*n+1].iov_len = _MAX_SAMPLE_BYTES_PER_PACKET;
                ch_recv_buffer_info_i.msgs[n].msg_hdr.msg_iov = &ch_recv_buffer_info_i.iovecs[2*n];
                ch_recv_buffer_info_i.msgs[n].msg_hdr.msg_iovlen = 2;
            }


            size_t n_last_packet = num_packets_to_recv - 1;
            // Location to write header data to
            ch_recv_buffer_info_i.iovecs[2*n_last_packet].iov_base =ch_recv_buffer_info_i.headers[n_last_packet].data();
            ch_recv_buffer_info_i.iovecs[2*n_last_packet].iov_len = _HEADER_SIZE;
            // Location to write sample data to
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+1].iov_base = samples_sg_dst[ch][n_last_packet];
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+1].iov_len = expected_data_in_last_packet;
            // Location to write samples that don't fit in sample_buffer to
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+2].iov_base = ch_recv_buffer_info_i.sample_cache.data();
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+2].iov_len = expected_excess_data_in_last_packet;
            ch_recv_buffer_info_i.msgs[n_last_packet].msg_hdr.msg_iov = &ch_recv_buffer_info_i.iovecs[2*n_last_packet];
            ch_recv_buffer_info_i.msgs[n_last_packet].msg_hdr.msg_iovlen = 3;
        }

        // TODO: make it work with higher numbers of sample requested
        // Adds receive requests to io_uring queue
        for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
            ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];
            for(size_t n = 0; n < num_packets_to_recv; n++) {

                // Gets where to store info for request
                struct io_uring_sqe *sqe;
                sqe = io_uring_get_sqe(&io_rings[ch]);

                // Happens when kernel thread takes a while to process io_uring_cqe_seen
                // TODO: handle this gracefully
                if(sqe == NULL) {
                    printf("num_packets_to_recv: %lu\n", num_packets_to_recv);
                    printf("sqe_requests: %lu\n", sqe_requests);
                    printf("cqe_seen: %lu\n", cqe_seen);
                    printf("unconsumed entries: %u\n", io_uring_cq_ready(&io_rings[ch]));
                    throw uhd::runtime_error( "io queue full" );
                }

                sqe_requests++;

                // Prepares request
                io_uring_prep_recvmsg(sqe, recv_sockets[ch], &ch_recv_buffer_info_i.msgs[n].msg_hdr, 0);

                // Set flag to ensure reads are in the correct order
                // Doesn't work across submits
                // sqe->flags |= IOSQE_IO_LINK;

                // Forces requests to be done in the order they appear in (works between submits)
                sqe->flags |= IOSQE_IO_DRAIN;

                // Submits requests
                int requests_submitted = io_uring_submit(&io_rings[ch]);
                // TODO: gracefully handle these conditions
                if(requests_submitted < 0) {
                    printf("io_uring_submit failed: %s\n", strerror(-requests_submitted));
                    throw uhd::runtime_error( "io_uring_submit error" );
                }
            }
        }

        printf("Finished submitting\n");

        // Gets the start time for use in the timeout, uses CLOCK_MONOTONIC_COARSE because it is faster and precision doesn't matter for timeouts
        struct timespec recv_start_time;
        clock_gettime(CLOCK_MONOTONIC_COARSE, &recv_start_time);
        int64_t recv_timeout_time_ns = (recv_start_time.tv_sec * 1000000000) + recv_start_time.tv_nsec + (int64_t)(timeout * 1000000000);

        // Flag to indicate if a timeout occured. Note: timeout should only be reported if no data was received
        bool timeout_occured = false;
        size_t num_channels_serviced = 0;
        while(num_channels_serviced < _NUM_CHANNELS) {

            // TODO: remove dropped io_uring requests
            // Check for timeout
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC_COARSE, &current_time);
            int64_t current_time_ns = (current_time.tv_sec * 1000000000) + current_time.tv_nsec;
            // Disabled because io_uring needs to flush the requests
            if(/*current_time_ns > recv_timeout_time_ns*/ 0) {
                printf("TIMEOUT\n");
                timeout_occured = true;
                break;
            }

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];

                // Skip this channel if it has already received enough packets
                if(ch_recv_buffer_info_i.num_headers_used >= num_packets_to_recv) {
                    continue;
                }

                // Gets the next completed receive
                struct io_uring_cqe *cqe_ptr;
                int recv_ready = io_uring_peek_cqe(&io_rings[ch], &cqe_ptr);

                // Indicates no reply to request has been received yet
                if(recv_ready == -EAGAIN) {
                    continue;
                // Errors other than EAGAIN should be impossible
                } if(recv_ready != 0) {
                    throw uhd::runtime_error( "io_uring_peek_cqe error" );
                }

                // Will return the normal return value of recvmsg on success, what would be -errno of after recvmsg on failure
                int recv_return = cqe_ptr->res;

                // Tell the ring buffer that the cqe_ptr has been processed
                io_uring_cqe_seen(&io_rings[ch], cqe_ptr);
                cqe_seen++;

                int num_packets_received_this_recv;
                if(recv_return > 0) {
                    num_packets_received_this_recv = 1;
                    ch_recv_buffer_info_i.msgs[ch_recv_buffer_info_i.num_headers_used].msg_len = recv_return;
                } else {
                    num_packets_received_this_recv = 0;
                }

                //Records number of packets received if no error
                if(num_packets_received_this_recv >= 0) {
                    ch_recv_buffer_info_i.num_headers_used += num_packets_received_this_recv;
                    if(ch_recv_buffer_info_i.num_headers_used >= num_packets_to_recv)
                    {
                        //Record that a channel has received all its packets
                        num_channels_serviced++;
                    }
                }
                // Moves onto next channel, these errors are expected if using MSG_DONTWAIT and no packets are ready
                else if (-recv_return == EAGAIN || -recv_return == EWOULDBLOCK) {
                    continue;
                // Error cause when program received interrupt during recv
                } else if (-recv_return == EINTR) {
                    return rx_metadata_t::ERROR_CODE_EINTR;
                // Unexpected error
                } else {
                    throw uhd::runtime_error( "System recvmmsg error:" + std::string(strerror(-recv_return)));
                }
            }
        }

        extract_vrt_metadata();

        for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
            size_t num_bytes_received = 0;

            // Clear count for number of samples in cache. Will be set in checking data received in the last packet if applicable
            ch_recv_buffer_info_i.sample_cache_used = 0;

            // Records the amount of data received from each packet
            for(size_t n = 0; n < ch_recv_buffer_info_i.num_headers_used; n++) {
                // Check if an invalid packet was received
                if(ch_recv_buffer_info_i.msgs[n].msg_len < _HEADER_SIZE) {
                    throw std::runtime_error("Received sample packet smaller than header size");
                }
                uint32_t num_bytes_this_packets = ch_recv_buffer_info_i.vrt_metadata[n].num_payload_words32 * sizeof(int32_t);

                // Records the amount of data received in the last packet if the desired number of packets were received (which means data could have been written to the cache)
                if(n + 1 == num_packets_to_recv) {
                    if(num_bytes_this_packets > expected_data_in_last_packet) {
                        ch_recv_buffer_info_i.data_bytes_from_packet[n] = expected_data_in_last_packet;
                        ch_recv_buffer_info_i.sample_cache_used = num_bytes_this_packets - expected_data_in_last_packet;
                        num_bytes_received += expected_data_in_last_packet;
                    } else {
                        ch_recv_buffer_info_i.data_bytes_from_packet[n] = num_bytes_this_packets;
                        num_bytes_received += num_bytes_this_packets;
                    }
                // Records the amount of data received from most packets
                } else {
                    ch_recv_buffer_info_i.data_bytes_from_packet[n] = num_bytes_this_packets;
                    num_bytes_received += num_bytes_this_packets;
                }
            }
        }

        if(timeout_occured) {
            for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
                if(ch_recv_buffer_info_i.num_headers_used == 0) {
                    return rx_metadata_t::ERROR_CODE_TIMEOUT;
                }
            }
            // Only return timeout if one of the channels received no data
            return rx_metadata_t::ERROR_CODE_NONE;
        } else {
            return rx_metadata_t::ERROR_CODE_NONE;
        }
    }

    /*******************************************************************
     * extract_vrt_metadata:
     * extracts metadata fromthe vrt headers in ch_recv_buffer_info.headers and stores in ch_recv_buffer_info.vrt_metadata
     ******************************************************************/
    UHD_INLINE void extract_vrt_metadata() {

        for(size_t ch_i = 0; ch_i < ch_recv_buffer_info_group.size(); ch_i++) {
            for(size_t packet_i = 0; packet_i < ch_recv_buffer_info_group[ch_i].num_headers_used; packet_i++) {
                // Number of 32 bit words per vrt packet
                // will be compared against packet length field
                ch_recv_buffer_info_group[ch_i].vrt_metadata[packet_i].num_packet_words32 = (_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET +_TRAILER_SIZE)/sizeof(uint32_t);
                // First word of the packet
                const uint32_t* vrt_hdr = (uint32_t*) ch_recv_buffer_info_group[ch_i].headers[packet_i].data();
                //ifpi.has_tsf = true;
                if_hdr_unpack(vrt_hdr, ch_recv_buffer_info_group[ch_i].vrt_metadata[packet_i]);
            }
        }
    }

    /*******************************************************************
     * align_buffs:
     * Checks for sequence number or timestamp errors and drops samples to keep channels aligned
     * error_code where to store the error code
     * return: Number of aligned bytes
     ******************************************************************/
    UHD_INLINE uint64_t align_buffs(uhd::rx_metadata_t::error_code_t& error_code) {

        bool oflow_error = false;

        std::vector<uint64_t> aligned_bytes(_NUM_CHANNELS, 0);

        // Checks for overflows
        size_t ch = 0;
        for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
            // Each channel should end up with the same number of aligned bytes so its fine to reset the counter each channel which will end up using the last one
            for(size_t header_i = 0; header_i < ch_recv_buffer_info_i.num_headers_used; header_i++) {
                //assume vrt_metadata.hst_tsf is true
                // Checks if sequence number is correct, ignore check if timestamp is 0
                if((ch_recv_buffer_info_i.vrt_metadata[header_i].packet_count != (sequence_number_mask & (ch_recv_buffer_info_i.previous_sequence_number + 1)))  && (ch_recv_buffer_info_i.vrt_metadata[header_i].tsf != 0)) {
                    oflow_error = true;
                    UHD_LOG_FASTPATH("D" + std::to_string(ch_recv_buffer_info_i.vrt_metadata[header_i].tsf) + "\n");
                    // UHD_LOG_FASTPATH("D");
                    if(!_using_performance_governor && !_performance_warning_printed) {
                        UHD_LOG_FASTPATH("\nRecv overflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                        _performance_warning_printed = true;
                    }
                }
                ch_recv_buffer_info_i.previous_sequence_number = ch_recv_buffer_info_i.vrt_metadata[header_i].packet_count;
                aligned_bytes[ch] += ch_recv_buffer_info_i.data_bytes_from_packet[header_i];
            }
            ch++;
        }

        bool alignment_required = (error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) || oflow_error;

        // Will acts as if the channel with the lowest number of samples is the amount of samples received for all
        uint64_t smallest_aligned_bytes = aligned_bytes[0];
        for(size_t n = 1; n < _NUM_CHANNELS; n++) {
            // Alignment is required if a different number of samples was received on each channel
            alignment_required = alignment_required || (smallest_aligned_bytes != aligned_bytes[n]);
            smallest_aligned_bytes = std::min(smallest_aligned_bytes, aligned_bytes[n]);
        }

        if(alignment_required) {
            // Sets the error code to overflow if not already seat to something
            // Done here to avoid an extra if in a pass without errors
            if(error_code == uhd::rx_metadata_t::ERROR_CODE_NONE && oflow_error) {
                error_code = uhd::rx_metadata_t::ERROR_CODE_OVERFLOW;
            }
            std::vector<uint64_t> last_tsf(_NUM_CHANNELS);
            uint64_t latest_tsf = 0;
            uint64_t last_packet_count = 0;
            // Figures out how many packets to drop
            for(size_t n = 0; n < _NUM_CHANNELS; n++) {
                // Skips if no packets were received on this channel
                if(ch_recv_buffer_info_group[n].num_headers_used == 0) {
                    continue;
                }
                // Gets the last timestamp and last sequence number on every channel
                size_t last_header = ch_recv_buffer_info_group[n].num_headers_used - 1;
                last_tsf[n] = ch_recv_buffer_info_group[n].vrt_metadata[last_header].tsf;
                if(latest_tsf< last_tsf[n]) {
                    latest_tsf = last_tsf[n];
                    last_packet_count = ch_recv_buffer_info_group[n].vrt_metadata[last_header].packet_count;
                }
            }
            for(size_t n = 0; n < _NUM_CHANNELS; n++) {
                // Receives packets and does nothing with them to realign
                // Not the most robust system, but anything more through would slow
                int num_packet_to_drop =(int) ((latest_tsf - last_tsf[n]) / (_MAX_SAMPLE_BYTES_PER_PACKET/_BYTES_PER_SAMPLE));
                flush_packets(n, num_packet_to_drop);
                // Sets packets count to match what
                ch_recv_buffer_info_group[n].previous_sequence_number = last_packet_count;
            }
        }

        return smallest_aligned_bytes;
    }

    /*******************************************************************
     * flush_packets:
     * Flushes the buffer by receiving packets but doing nothing with them
     * ch: channel to receive packets for
     * limit: maximum number of packets
     * no_limit: ignore the limit
     * aligned_samples: stores the number of aligned sample
     * return: rx metadata error code
     ******************************************************************/
    size_t flush_packets(size_t ch, int limit, bool no_limit = false) {
        int num_packets;
        ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];
        if(no_limit) {
            num_packets = ch_recv_buffer_info_i.max_num_packets_to_flush;
        } else {
            //TODO resize buffer if requested limit exceeds the buffer size
            num_packets = std::min((int)ch_recv_buffer_info_i.max_num_packets_to_flush, limit);
        }
        struct mmsghdr msgs[num_packets];
        struct iovec iovecs[num_packets];

        memset(msgs, 0, sizeof(msgs));

        for (size_t n = 0; n < (size_t) num_packets; n++) {
            iovecs[n].iov_base = ch_recv_buffer_info_i.flush_buffer[n].data();
            iovecs[n].iov_len = _HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET;
            msgs[n].msg_hdr.msg_iov = &iovecs[n];
            msgs[n].msg_hdr.msg_iovlen = 1;
        }

        int total_packets_received = 0;
        do {
            int packets_received = recvmmsg(recv_sockets[ch], msgs, num_packets, MSG_DONTWAIT, 0);
            if(packets_received == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                // MSG_DONTWAIT + EAGAIN or EWOULDBLOCK indicate that there are no more packets to receive in the buffer
                return total_packets_received;
            } else if(packets_received == -1) {
                throw uhd::runtime_error( "System recvmmsg error while flushing buffer:" + std::string(strerror(errno)));
            } else {
                total_packets_received+=packets_received;
            }
        } while(total_packets_received < limit || no_limit);
        return total_packets_received;
    }

    /*!
     * Prepares the converter
     * Called as part of the constructor, only in its own function to improve readability
     * \param cpu_format datatype of samples on the host system (only sc16 and fc32)
     * \param wire_format datatype of samples in the packets (only sc16 or sc12)
     * \param wire_little_endian data format in packets is little endian
     */
    UHD_INLINE void setup_converter(const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian) {
        // No converter required, scatter gather will be used
        if(cpu_format == wire_format && wire_little_endian) {
            converter_used = false;
            return;
        } else {
            converter_used = true;
            //set the converter
            uhd::convert::id_type converter_id;
            if(wire_little_endian) {
                // item32 results in entire 32 bit words being interpreted as little endian
                // i.e. _item32_le means Q LSB, Q MSB, I LSB, I MSB
                // we want _item32_le means I LSB, I MSB, Q LSB, Q MSB
                // We want 16 bit halves to be little endian, which chdr provides
                // NOTE: chdr is a legacy data format for old Ettus stuff
                // If it ever gets removes create an identical implementation named _item_16_le
                converter_id.input_format = wire_format + "_chdr";
            } else {
                converter_id.input_format = wire_format + "_item32_be";
            }
            converter_id.num_inputs = 1;
            converter_id.output_format = cpu_format;
            converter_id.num_outputs = 1;

            _converter = uhd::convert::get_converter(converter_id)();

            double cpu_max;
            if ("fc32" == cpu_format) {
                cpu_max = 1;
            } else if("sc16" == cpu_format) {
                cpu_max = 0x7fff;
            } else {
                throw uhd::runtime_error( "Unsupported CPU format: " + cpu_format);
            }

            double wire_max;
            if("sc16" == wire_format) {
                wire_max = 0x7fff;
            } else if("sc12" == wire_format) {
                wire_max = 0x7ff;
            } else {
                throw uhd::runtime_error( "Unsupported wire format: " + cpu_format);
            }

            _converter->set_scalar(cpu_max / wire_max);
        }
    }

    // Resizes the intermediate buffers (if needed) and updates the
    // Returns the wrapper (set to the data in the constructor, done as a workaround because the UHD uses a vector wrapper)
    UHD_INLINE uhd::rx_streamer::buffs_type* prepare_intermediate_buffers(size_t bytes) {
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            if(ch_recv_buffer_info_group[n].intermediate_recv_buffer.size() < bytes) {
                // Resizes intermediate buffer
                ch_recv_buffer_info_group[n].intermediate_recv_buffer.resize(bytes);
            }
            // Updates the pointer to the intermediate buffer
            _intermediate_recv_buffer_pointers[n] = ch_recv_buffer_info_group[n].intermediate_recv_buffer.data();
        }

        return &_intermediate_recv_buffer_wrapper;
    }

    // Copies samples from _intermediate_recv_buffer_pointers to user_buffer
    void convert_samples(const uhd::rx_streamer::buffs_type& user_buffer_ptrs, size_t num_samples) {
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            // TODO figure out how the converter works to optimize this, it might be possible to do all at once
            const ref_vector<void*> user_buffer_ch(user_buffer_ptrs[n]);
            // Converts the samples
            _converter->conv(_intermediate_recv_buffer_pointers[n], user_buffer_ch, num_samples);
        }
    }

    bool detect_end_of_burst() {
        //TODO: fix FPGA and re-enable this function. At time of writing Crimson will always have eob flag and Cyan will never have it
        return false;
        // bool end_of_burst_received = false;
        // for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
        //     if(ch_recv_buffer_info_i.num_headers_used > 0) {
        //         // Set end of burst if received on any channel
        //         if(ch_recv_buffer_info_i.vrt_metadata[ch_recv_buffer_info_i.num_headers_used-1].eob) {
        //             end_of_burst_received = true;
        //         }
        //     }
        // }
        //
        // bool should_cache_eob = end_of_burst_received;
        // if(end_of_burst_received) {
        //     for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
        //         if(ch_recv_buffer_info_i.sample_cache_used == 0) {
        //             should_cache_eob = false;
        //         }
        //     }
        // }
        //
        // // EOB received, but there as samples in the cache so it should be saved until the next receive
        // if(should_cache_eob && end_of_burst_received) {
        //     cached_end_of_burst = true;
        //     return false;
        // // EOB received and there are no samples in the cache so clear EOB cache and set metadata flag
        // } else if(end_of_burst_received) {
        //     cached_end_of_burst = false;
        //     return true;
        // } else {
        // // Clear EOB cache, should be unreachable because
        //     cached_end_of_burst = false;
        //     return false;
        // }
    }

    int get_mtu(int socket_fd, std::string ip) {
        //Start of linked list containing interface info
        struct ifaddrs *ifaces = nullptr;

        // Gets a linked list of all interfaces
        getifaddrs(&ifaces);
        for(ifaddrs *iface = ifaces; iface != NULL; iface = iface->ifa_next) {

            // Verifies this interface has a broadcast address
            if(iface->ifa_broadaddr != nullptr) {
                // Verifies said broadcast address is IPV4
                if(iface->ifa_broadaddr->sa_family == AF_INET) {
                    // Converts broadcast address to human readable format
                    char broadcast_buffer[INET_ADDRSTRLEN] = {0, };
                    auto ret = inet_ntop(AF_INET,  &((struct sockaddr_in*)(iface->ifa_broadaddr))->sin_addr, broadcast_buffer, INET_ADDRSTRLEN);
                    if(ret == nullptr) {
                        //TODO: throw error if inet_ntop failed
                    }

                    // Converts IP address to byte array
                    uint8_t interface_ip[4];
                    sscanf(broadcast_buffer, "%hhu.%hhu.%hhu.%hhu", &interface_ip[0], &interface_ip[1], &interface_ip[2], &interface_ip[3]);
                    uint8_t device_ip[4];
                    sscanf(ip.c_str(), "%hhu.%hhu.%hhu.%hhu", &device_ip[0], &device_ip[1], &device_ip[2], &device_ip[3]);

                    // Checks if the interface subnet matches the Crimson ip to be checked
                    bool ip_matches = true;
                    for(int n = 0; n < 4; n++) {
                        // Checks if the IPs match or the interface is 255 (which corresponds to any)
                        if(interface_ip[n] != device_ip[n] && interface_ip[n] != 255) {
                            ip_matches = false;
                            break;
                        }
                    }
                    if(!ip_matches) {
                        continue;
                    }

                    struct ifreq ifr;
                    ifr.ifr_addr.sa_family = AF_INET;//address family = IPV4
                    strncpy(ifr.ifr_name, iface->ifa_name, sizeof(ifr.ifr_name));//interface name of MTU to get
                    // Gets MTU
                    if (ioctl(socket_fd, SIOCGIFMTU, (caddr_t)&ifr) < 0) {
                        throw uhd::system_error("ioctl error when attempting to check MTU\n");
                    }
                    freeifaddrs(ifaces);
                    return ifr.ifr_mtu;
                }
            }
        }
        freeifaddrs(ifaces);
        throw uhd::system_error("No interface with subnet matching ip found");
    }

};

class recv_packet_streamer_mmsg : public recv_packet_handler_mmsg, public rx_streamer
{
public:
    recv_packet_streamer_mmsg(const std::vector<std::string>& dst_ip, std::vector<int>& dst_port, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian)
    : recv_packet_handler_mmsg(dst_ip, dst_port, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian)
    {
    }

    //Consider merging recv_packet_streamer_mmsg and recv_packet_handler_mmsg
    //This is here to implement a virtual function from rx_streamer
    size_t recv(const rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet) override
    {
        return recv_packet_handler_mmsg::recv(
            buffs, nsamps_per_buff, metadata, timeout, one_packet);
    }

    UHD_INLINE size_t get_num_channels(void) const{
        return _NUM_CHANNELS;
    }

    // Gets the maximum number of samples per packet
    UHD_INLINE size_t get_max_num_samps(void) const{
        return _MAX_SAMPLE_BYTES_PER_PACKET/_BYTES_PER_SAMPLE;
    }
};

}}} // namespace uhd::transport::sph

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP */
