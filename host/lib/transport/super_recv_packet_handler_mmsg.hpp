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
    // TODO: get real values for each of the effective constants
    recv_packet_handler_mmsg(const std::vector<std::string>& dst_ip, std::vector<int>& dst_port, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t bytes_per_sample )
    : recv_packet_handler(max_sample_bytes_per_packet + header_size), NUM_CHANNELS(dst_ip.size()), MAX_DATA_PER_PACKET(max_sample_bytes_per_packet), BYTES_PER_SAMPLE(bytes_per_sample), HEADER_SIZE(header_size)
    {

        // Creates and binds to sockets
        for(size_t n = 0; n < NUM_CHANNELS; n++) {
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
                //TODO: make this error more descriptive
                std::cerr << "Unable to bind ip adress, receive may not work. \n IP: " << dst_ip[n] << std::endl;
            }

            // TODO add the warning from old UHD that says how to change the socket buffer size limit
            // Sets receive buffer size to (probably) maximum
            // TODO: verify if recv buffer can be set higher
            int recv_buff_size = 1048576;
            if(setsockopt(recv_socket_fd, SOL_SOCKET, SO_RCVBUF, &recv_buff_size, sizeof(recv_buff_size))) {
                std::cerr << "Error while setting recv buffer size, performance may be affected" << std::endl;
            }

            // TODO: empty socket buffer

            recv_sockets.push_back(recv_socket_fd);
        }

        for(size_t n = 0; n < NUM_CHANNELS; n++) {
            ch_recv_buffer_info tmp = {
                (size_t) 0, // sample_cache_used
                std::vector<std::vector<int8_t>>(_num_header_buffers, std::vector<int8_t>(HEADER_SIZE, 0)), // headers
                std::vector<vrt::if_packet_info_t>(_num_header_buffers), // vrt_metadata
                0, //previous_packet_count
                std::vector<size_t>(_num_header_buffers, 0), // data_bytes_from_packet
                std::vector<int8_t>(MAX_DATA_PER_PACKET, 0), // sample_cache
                (size_t) 0 // previous_sample_cache_used
            };
            ch_recv_buffer_info_group.push_back(tmp);
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

        // Clears number of headers used
        for(size_t n = 0; n < ch_recv_buffer_info_group.size(); n++) {
            ch_recv_buffer_info_group[0].num_headers_used = 0;
        }

        size_t bytes_per_buff = nsamps_per_buff * BYTES_PER_SAMPLE;

        std::vector<size_t> nsamps_received(NUM_CHANNELS, 0);

        // Number of bytes copied from the cache (should be the same for every channel)
        size_t cached_bytes_to_copy;
        // received data for every channel sequentially
        // TODO: experiment if parallelizing this helps performance
        // ch refers to which element in ch_recv_buffer_info_group not the actual channel number
        for(size_t ch = 0; ch < NUM_CHANNELS; ch++) {
            // Copies cached data from previous recv
            cached_bytes_to_copy = std::min(ch_recv_buffer_info_group[ch].sample_cache_used, bytes_per_buff);
            memcpy(buffs[ch], ch_recv_buffer_info_group[ch].sample_cache.data(), cached_bytes_to_copy);
            // How many bytes still need to be received after copying from the cache
            size_t remaining_nbytes_per_buff = bytes_per_buff - cached_bytes_to_copy;
            if(one_packet) {
                remaining_nbytes_per_buff = std::min(remaining_nbytes_per_buff, MAX_DATA_PER_PACKET - cached_bytes_to_copy);
            }
            // Indicates that the cache is clear
            ch_recv_buffer_info_group[ch].sample_cache_used-= cached_bytes_to_copy;

            nsamps_received[ch] += cached_bytes_to_copy / BYTES_PER_SAMPLE;

            // Skip receiving data from network if the cache had all the data requested
            // TODO: make sure this doesn't skip conversion to other CPU formats after that is implemented
            if(!remaining_nbytes_per_buff) {
                break;
            }

            // Receives packets, data is stores in buffs, metadata is stored in ch_recv_buffer_info.headers
            metadata.error_code = recv_multiple_packets(ch, buffs[ch]+cached_bytes_to_copy, remaining_nbytes_per_buff, timeout);

            // TODO implement returning data that is received prior to encountering an error, currently acts as if no samples received
            if(metadata.error_code) {
                return 0;
            }
        }

        extract_vrt_metadata();

        size_t aligned_bytes = 0;
        // Check for overflow errors and (when implemented) shifts data to keep buffers aligned after an overflow)
        metadata.error_code = align_buffs(buffs, cached_bytes_to_copy, bytes_per_buff, aligned_bytes);

        return aligned_bytes/BYTES_PER_SAMPLE;
    }

    /*******************************************************************
     * recv_multiple_packets:
     * channel: which ch_recv_buffer_info_group to receive for
     * receives multiple packets on a given channel
     * sample_buffer: start of location in memory to store samples from received packets
     * timeout: timout, TODO: make sure call for other channels take into account time taken by the previous channel
     * one_packet: only receive one packet
     * returns error code
     ******************************************************************/
    UHD_INLINE uhd::rx_metadata_t::error_code_t recv_multiple_packets(size_t channel, void* sample_buffer, size_t nbytes_per_buff, double timeout) {

        // TODO: currently being written to write directly to the buffer to return to the user, need to implement a conversion for other cpu formats
        // Pointers for where to write samples to from each packet using scatter gather
        std::vector<void*> samples_sg_dst;

        for(size_t p = 0; p < nbytes_per_buff; p += MAX_DATA_PER_PACKET) {
            samples_sg_dst.push_back(p+sample_buffer);
        }

        size_t num_packets_to_recv = samples_sg_dst.size();

        // Adds for room to store metadata if required
        if(num_packets_to_recv + ch_recv_buffer_info_group[channel].num_headers_used > _num_header_buffers) {
            for(size_t n = 0; n < NUM_CHANNELS; n++) {
                _num_header_buffers = num_packets_to_recv + ch_recv_buffer_info_group[channel].num_headers_used;
                ch_recv_buffer_info_group[n].headers.resize(_num_header_buffers, std::vector<int8_t>(HEADER_SIZE, 0));
                ch_recv_buffer_info_group[n].vrt_metadata.resize(_num_header_buffers);
                ch_recv_buffer_info_group[channel].data_bytes_from_packet.resize(_num_header_buffers, 0);
            }
        }

        // Stores data about the recv for each packet
        struct mmsghdr msgs[num_packets_to_recv];
        // Contains data about where to store received data
        // Alternating between pointer to header, pointer to data
        struct iovec iovecs[2*num_packets_to_recv+1];

        memset(msgs, 0, sizeof(msgs));
        for (size_t n = 0; n + ch_recv_buffer_info_group[channel].num_headers_used < num_packets_to_recv - 1; n++) {
            // Location to write header data to
            iovecs[2*n].iov_base =ch_recv_buffer_info_group[channel].headers[n+ch_recv_buffer_info_group[channel].num_headers_used].data();
            iovecs[2*n].iov_len = HEADER_SIZE;
            // Location to write sample data to
            iovecs[2*n+1].iov_base = samples_sg_dst[n];
            iovecs[2*n+1].iov_len = MAX_DATA_PER_PACKET;
            msgs[n].msg_hdr.msg_iov = &iovecs[2*n];
            msgs[n].msg_hdr.msg_iovlen = 2;
        }

        size_t n_last_packet = num_packets_to_recv - 1;
        // Amount of data stored in the cache betwen recv
        size_t excess_data_in_last_packet = num_packets_to_recv * MAX_DATA_PER_PACKET - nbytes_per_buff;
        // Amount of data in the last packet copied directly to buffer
        size_t data_in_last_packet = MAX_DATA_PER_PACKET - excess_data_in_last_packet;
        // Location to write header data to
        iovecs[2*n_last_packet].iov_base =ch_recv_buffer_info_group[channel].headers[n_last_packet].data();
        iovecs[2*n_last_packet].iov_len = HEADER_SIZE;
        // Location to write sample data to
        iovecs[2*n_last_packet+1].iov_base = samples_sg_dst[n_last_packet];
        iovecs[2*n_last_packet+1].iov_len = data_in_last_packet;
        // Location to write samples that don't fit in sample_buffer to
        iovecs[2*n_last_packet+2].iov_base = ch_recv_buffer_info_group[channel].sample_cache.data();
        iovecs[2*n_last_packet+2].iov_len = excess_data_in_last_packet;
        msgs[n_last_packet].msg_hdr.msg_iov = &iovecs[2*n_last_packet];
        msgs[n_last_packet].msg_hdr.msg_iovlen = 3;

        struct timespec ts_timeout{(int)timeout, (int) ((timeout - ((int)timeout))*1000000000)};

        int num_packets_received = 0;
        int num_packets_received_this_recv = 0;
        do {
            //MSG_DONTWAIT is used so to make sure the schedueler doesn't deprioritize this thread while waiting for data
            num_packets_received_this_recv = recvmmsg(recv_sockets[channel], &msgs[num_packets_received], (int) num_packets_to_recv - num_packets_received, MSG_DONTWAIT, 0);
            if(num_packets_received_this_recv >= 0) {
                num_packets_received += num_packets_received_this_recv;
            }
        } while ((num_packets_received_this_recv == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) || num_packets_received < num_packets_to_recv );

        if(num_packets_received == -1) {
            if(errno == EINTR) {
                return rx_metadata_t::ERROR_CODE_EINTR;
            } else {
                throw uhd::runtime_error( "System recvmmsg error:" + std::string(strerror(errno)));
            }
        }

        ch_recv_buffer_info_group[channel].num_headers_used = (size_t) num_packets_received;

        // Resets the count for amount of data in the cache, if any data was written there it will be recorded below
        ch_recv_buffer_info_group[channel].sample_cache_used = 0;

        size_t num_bytes_received = 0;
        // Records the amount of data received from each packet
        for(size_t n = 0; n < ch_recv_buffer_info_group[channel].num_headers_used; n++) {
            // Check if an invalid packet was received
            if(msgs[n].msg_len < HEADER_SIZE) {
                throw std::runtime_error("Received sample packet smaller than header size");
            }
            uint32_t num_bytes_this_packets = msgs[n].msg_len - HEADER_SIZE;

            // Records the amount of data received in the last packet if the desired number of packets were received (which means data could have been written to the cache)
            if(n + 1 == num_packets_to_recv) {
                size_t received_data_in_last_packet = msgs[n].msg_len - HEADER_SIZE;
                if(received_data_in_last_packet > data_in_last_packet) {
                    ch_recv_buffer_info_group[channel].data_bytes_from_packet[n] = data_in_last_packet;
                    ch_recv_buffer_info_group[channel].sample_cache_used = received_data_in_last_packet - data_in_last_packet;
                    num_bytes_received += data_in_last_packet;
                } else {
                    ch_recv_buffer_info_group[channel].data_bytes_from_packet[n] = received_data_in_last_packet;
                    // sample_cache_used already set to 0 before this loop so doesn;t need to be set to 0 here
                    num_bytes_received += received_data_in_last_packet;
                }
            // Records the amount of data received from most packets
            } else {
                ch_recv_buffer_info_group[channel].data_bytes_from_packet[n] = msgs[n].msg_len - HEADER_SIZE;
                num_bytes_received += num_bytes_this_packets;
            }
        }

        // Records the amount of data stored in the cache
        if(num_packets_to_recv == (size_t)num_packets_received) {
            ch_recv_buffer_info_group[channel].sample_cache_used = excess_data_in_last_packet;
        } else {
        }

        return rx_metadata_t::ERROR_CODE_NONE;
    }


    virtual void if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

    /*******************************************************************
     * extract_vrt_metadata:
     * extracts metadata fromthe vrt headers in ch_recv_buffer_info.headers and stores in ch_recv_buffer_info.vrt_metadata
     ******************************************************************/
    UHD_INLINE void extract_vrt_metadata() {

        for(size_t ch_i = 0; ch_i < ch_recv_buffer_info_group.size(); ch_i++) {
            for(size_t packet_i = 0; packet_i < ch_recv_buffer_info_group[ch_i].num_headers_used; packet_i++) {
                // Number of 32 bit words per vrt packet
                // will be compared against packet length field
                // TODO: get actual amount of bytes received from recvmmg, instead of assuming full packet
                ch_recv_buffer_info_group[ch_i].vrt_metadata[packet_i].num_packet_words32 = (HEADER_SIZE + MAX_DATA_PER_PACKET)/sizeof(uint32_t);
                // First word of the packet
                const uint32_t* vrt_hdr = (uint32_t*) ch_recv_buffer_info_group[ch_i].headers[packet_i].data();
                //ifpi.has_tsf = true;
                if_hdr_unpack(vrt_hdr, ch_recv_buffer_info_group[ch_i].vrt_metadata[packet_i]);
            }
        }
    }

    /*******************************************************************
     * align_buffs:
     * Checks for sequence number or timestamp errors and drops samples to keep channels aligned. Also shifts buffer to account for smaller packets (although variable size packets will result in a massive performance drop off because of all the copying)
     * sample_buffers: buffer containing samples
     * sample_buffer_offset: offset (in bytes) in the samples buffer of where data to align begins. Must correspond to the data associated with headers[0]
     * buffer_size: size of sample_buffer in bytes, used to determine how many samples to copy from the cache if applicable
     * aligned_samples: stores the number of aligned sample
     * return: rx metadata error code
     ******************************************************************/
    UHD_INLINE uhd::rx_metadata_t::error_code_t align_buffs(const uhd::rx_streamer::buffs_type& sample_buffers, size_t sample_buffer_offset, size_t buffer_size, size_t& aligned_bytes) {

        uhd::rx_metadata_t::error_code_t error_code = rx_metadata_t::ERROR_CODE_NONE;

        // Calculates how many packets to align
        // TODO: implement aligning varying number of packets
        size_t num_packets_to_align = ch_recv_buffer_info_group[0].num_headers_used;
        for(size_t ch = 0; ch < NUM_CHANNELS; ch++) {
            if(num_packets_to_align != ch_recv_buffer_info_group[ch].num_headers_used) {
                std::cerr << "recv number of packets mismatch. Aligning a non matching number of samples not implemented yet" << std::endl;
                std::exit(~0);
            }
        }
        for(size_t ch = 0; ch < NUM_CHANNELS; ch++) {
            // Each channel should end up with the same number of aligned bytes so its fine to reset the counter each channel which will end up using the last one
            aligned_bytes = sample_buffer_offset;
            for(size_t header_i = 0; header_i < ch_recv_buffer_info_group[ch].num_headers_used; header_i++) {
                //assume vrt_metadata.hst_tsf is true
                // Checks if sequence number is correct, ignore check if timestamp is 0
                if((ch_recv_buffer_info_group[ch].vrt_metadata[header_i].packet_count != (0xf & (ch_recv_buffer_info_group[ch].previous_packet_count + 1)))  && (ch_recv_buffer_info_group[ch].vrt_metadata[header_i].tsf != 0)) {
                    error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
                    UHD_LOG_FASTPATH("D");
                    //TODO: implement aligning buffs after an overflow
                }
                ch_recv_buffer_info_group[ch].previous_packet_count = ch_recv_buffer_info_group[ch].vrt_metadata[header_i].packet_count;
                aligned_bytes += ch_recv_buffer_info_group[ch].data_bytes_from_packet[header_i];
            }
        }

        return error_code;
    }

protected:
    size_t NUM_CHANNELS;
    size_t MAX_DATA_PER_PACKET;
    size_t BYTES_PER_SAMPLE;

private:
    size_t HEADER_SIZE;
    // Number of existing header buffers, which is the current maximum number of packets to receive at a time, TODO: dynamically increase this when user requests more data at a time than can be contained in this many packets
    size_t _num_header_buffers = 32;
    std::vector<int> recv_sockets;
    size_t previous_sample_cache_used = 0;
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
        size_t previous_packet_count;
        //Stores how many bytes of sample data are from each packet
        std::vector<size_t> data_bytes_from_packet;
        // Stores extra data from packets between recvs
        std::vector<int8_t> sample_cache;
        // Stores amount of extra data cached from previous recv in byte
        size_t sample_cache_used;

    };
    // Group of recv info for each channels
    std::vector<ch_recv_buffer_info> ch_recv_buffer_info_group;


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
        return NUM_CHANNELS;
    }

    // Gets the maximum number of samples per packet
    UHD_INLINE size_t get_max_num_samps(void) const{
        return MAX_DATA_PER_PACKET/BYTES_PER_SAMPLE;
    }
};

}}} // namespace uhd::transport::sph

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP */
