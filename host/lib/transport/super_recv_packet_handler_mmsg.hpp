//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
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
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <error.h>

namespace uhd { namespace transport { namespace sph {

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

    /*!
     * Make a new packet handler for receive
     * \param dst_ip the IPV4 desination IP address of packets for each channe;
     * \param dst_port the detination port of packet for each channel
     * \param max_sample_bytes_per_packet the number of transport channels
     * \param header_size the number of transport channels
     * \param bytes_per_sample the number of transport channels
     */
    recv_packet_handler_mmsg(const std::vector<std::string>& dst_ip, std::vector<int>& dst_port, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t bytes_per_sample )
    : recv_packet_handler(max_sample_bytes_per_packet + header_size), _NUM_CHANNELS(dst_ip.size()), _MAX_SAMPLE_BYTES_PER_PACKET(max_sample_bytes_per_packet), _BYTES_PER_SAMPLE(bytes_per_sample), _HEADER_SIZE(header_size)
    {

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

            if(bind(recv_socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address)) < 0)
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
            }

            // recvmmsg should attempt to recv at most the amount to fill 1/_NUM_CHANNELS of the socket buffer
            _MAX_PACKETS_TO_RECV = (int)((_ACTUAL_RECV_BUFFER_SIZE/(_NUM_CHANNELS + 1))/(_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET + 42));

            recv_sockets.push_back(recv_socket_fd);
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
                std::vector<iovec>(2*_num_header_buffers+1)
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

    }

    ~recv_packet_handler_mmsg(void)
    {
        for(size_t n = 0; n < recv_sockets.size(); n++) {
            close(recv_sockets[n]);
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
        printf("\nbytes_per_buff: %lu\n", bytes_per_buff);

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
            memcpy(buffs[ch], ch_recv_buffer_info_i.sample_cache.data(), cached_bytes_to_copy);
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
            printf("4 bytes_to_recv: %lu\n", bytes_to_recv);
        }

        printf("T20\n");
        printf("5 bytes_to_recv: %lu\n", bytes_to_recv);

        // Returns the number of samples requested, if there were enough samples in the cache
        if(!bytes_to_recv) {
            metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;
            return nsamps_per_buff;
        }

        // Receives packets, data is stores in buffs, metadata is stored in ch_recv_buffer_info.headers
        metadata.error_code = recv_multiple_packets(buffs, cached_bytes_to_copy, cached_bytes_to_copy + bytes_to_recv, timeout);

        extract_vrt_metadata();

        // Check for overflow errors and (when implemented) shifts data to keep buffers aligned after an overflow)
        size_t aligned_bytes = align_buffs(metadata.error_code) + cached_bytes_to_copy;
        printf("cached_bytes_to_copy: %lu\n", cached_bytes_to_copy);

        size_t final_nsamps = aligned_bytes/_BYTES_PER_SAMPLE;
        std::cout << "metadata.error_code: " << metadata.error_code << std::endl;
        printf("final_nsamps: %lu\n", final_nsamps);
        return final_nsamps;
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
    // Number of existing header buffers, which is the current maximum number of packets to receive at a time
    size_t _num_header_buffers = 32;
    std::vector<int> recv_sockets;
    size_t previous_sample_cache_used = 0;
    // Maximum sequence number
    const size_t sequence_number_mask = 0xf;
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

    };
    // Group of recv info for each channels
    std::vector<ch_recv_buffer_info> ch_recv_buffer_info_group;

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
        printf("T40\n");
        printf("sample_buffer_offset: %lu\n", sample_buffer_offset);
        printf("buffer_length_bytes: %lu\n", buffer_length_bytes);

        // TODO: currently being written to write directly to the buffer to return to the user, need to implement a conversion for other cpu formats
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
                samples_sg_dst[ch].push_back(p+sample_buffers[ch]);
            }
        }
        printf("T50\n");

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


        // Amount of data stored in the cache betwen recv
        size_t excess_data_in_last_packet = num_packets_to_recv * _MAX_SAMPLE_BYTES_PER_PACKET - nbytes_to_recv;
        // Amount of data in the last packet copied directly to buffer
        size_t data_in_last_packet = _MAX_SAMPLE_BYTES_PER_PACKET - excess_data_in_last_packet;

        printf("num_packets_to_recv: %lu\n", num_packets_to_recv);

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
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+1].iov_len = data_in_last_packet;
            // Location to write samples that don't fit in sample_buffer to
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+2].iov_base = ch_recv_buffer_info_i.sample_cache.data();
            ch_recv_buffer_info_i.iovecs[2*n_last_packet+2].iov_len = excess_data_in_last_packet;
            ch_recv_buffer_info_i.msgs[n_last_packet].msg_hdr.msg_iov = &ch_recv_buffer_info_i.iovecs[2*n_last_packet];
            ch_recv_buffer_info_i.msgs[n_last_packet].msg_hdr.msg_iovlen = 3;
        }

        printf("T100\n");

        // Gets the start time for use in the timeout, uses CLOCK_MONOTONIC_COARSE because it is faster and precision doesn't matter for timeouts
        struct timespec recv_start_time;
        clock_gettime(CLOCK_MONOTONIC_COARSE, &recv_start_time);
        int64_t recv_timeout_time_ns = (recv_start_time.tv_sec * 1000000000) + recv_start_time.tv_nsec + (int64_t)(timeout * 1000000000);

        size_t num_channels_serviced = 0;
        while(num_channels_serviced < _NUM_CHANNELS) {
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC_COARSE, &current_time);
            int64_t current_time_ns = (current_time.tv_sec * 1000000000) + current_time.tv_nsec;
            if(current_time_ns > recv_timeout_time_ns) {
                return rx_metadata_t::ERROR_CODE_TIMEOUT;
            }

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                ch_recv_buffer_info& ch_recv_buffer_info_i = ch_recv_buffer_info_group[ch];
                // Skip this channel if it has already received enough packets
                if(ch_recv_buffer_info_i.num_headers_used >= num_packets_to_recv) {
                    continue;
                }
                //TODO: implement timeout
                // Receive packets system call
                int num_packets_received_this_recv = recvmmsg(recv_sockets[ch], &ch_recv_buffer_info_i.msgs[ch_recv_buffer_info_i.num_headers_used], std::min((int)(num_packets_to_recv - ch_recv_buffer_info_i.num_headers_used), _MAX_PACKETS_TO_RECV), MSG_DONTWAIT, 0);

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
                else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                // Error cause when program received interrupt during recv
                } else if (errno == EINTR) {
                    return rx_metadata_t::ERROR_CODE_EINTR;
                // Unexpected error
                } else {
                    throw uhd::runtime_error( "System recvmmsg error:" + std::string(strerror(errno)));
                }
            }
        }

        for(auto& ch_recv_buffer_info_i : ch_recv_buffer_info_group) {
            size_t num_bytes_received = 0;
            // Records the amount of data received from each packet
            for(size_t n = 0; n < ch_recv_buffer_info_i.num_headers_used; n++) {
                // Check if an invalid packet was received
                if(ch_recv_buffer_info_i.msgs[n].msg_len < _HEADER_SIZE) {
                    throw std::runtime_error("Received sample packet smaller than header size");
                }
                uint32_t num_bytes_this_packets = ch_recv_buffer_info_i.msgs[n].msg_len - _HEADER_SIZE;

                // Records the amount of data received in the last packet if the desired number of packets were received (which means data could have been written to the cache)
                if(n + 1 == num_packets_to_recv) {
                    size_t received_data_in_last_packet = ch_recv_buffer_info_i.msgs[n].msg_len - _HEADER_SIZE;
                    if(received_data_in_last_packet > data_in_last_packet) {
                        printf("T105\n");
                        ch_recv_buffer_info_i.data_bytes_from_packet[n] = data_in_last_packet;
                        ch_recv_buffer_info_i.sample_cache_used = received_data_in_last_packet - data_in_last_packet;
                        num_bytes_received += data_in_last_packet;
                    } else {
                        printf("T110\n");
                        ch_recv_buffer_info_i.data_bytes_from_packet[n] = received_data_in_last_packet;
                        // sample_cache_used already set to 0 before this loop so doesn;t need to be set to 0 here
                        num_bytes_received += received_data_in_last_packet;
                    }
                // Records the amount of data received from most packets
                } else {
                    ch_recv_buffer_info_i.data_bytes_from_packet[n] = ch_recv_buffer_info_i.msgs[n].msg_len - _HEADER_SIZE;
                    num_bytes_received += num_bytes_this_packets;
                }
            }

            // Records the amount of data stored in the cache
            if(num_packets_to_recv == ch_recv_buffer_info_i.num_headers_used) {
                ch_recv_buffer_info_i.sample_cache_used = excess_data_in_last_packet;
            }
        }

        return rx_metadata_t::ERROR_CODE_NONE;
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
                ch_recv_buffer_info_group[ch_i].vrt_metadata[packet_i].num_packet_words32 = (_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET)/sizeof(uint32_t);
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
                    //TODO: implement aligning buffs after an overflow
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
            printf("alignment_required: %hhu, smallest_aligned_bytes: %lu, aligned_bytes[n]: %lu\n", alignment_required, smallest_aligned_bytes, aligned_bytes[n]);
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


};

class recv_packet_streamer_mmsg : public recv_packet_handler_mmsg, public rx_streamer
{
public:
    recv_packet_streamer_mmsg(const std::vector<std::string>& dst_ip, std::vector<int>& dst_port, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t bytes_per_sample )
    : recv_packet_handler_mmsg(dst_ip, dst_port, max_sample_bytes_per_packet, header_size, bytes_per_sample)
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
