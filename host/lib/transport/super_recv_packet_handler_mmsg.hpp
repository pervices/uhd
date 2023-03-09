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
     * \param size the number of transport channels
     */
    // TODO: get real values for each of the effective constants
    recv_packet_handler_mmsg(const size_t size = 1)
    : recv_packet_handler(size), MAX_DATA_PER_PACKET(8880), HEADER_SIZE(16), MAX_PACKETS_AT_A_TIME(32), BYTES_PER_SAMPLE(4)
    {
        std::cout << "T50" << std::endl;
        //TODO: get this as a parameter
        channels = {0};
        std::vector<std::string> ips = {"10.10.10.10"};
        std::vector<int> ports = {42836};
        NUM_CHANNELS = channels.size();

        // Creates and binds to sockets
        for(size_t n = 0; n < NUM_CHANNELS; n++) {
            struct sockaddr_in dst_address;
            int recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if(recv_socket_fd < 0) {
                //TODO: make this error more descriptive
                std::cerr << "Failed to create recv socket" << std::endl;
                std::exit(1);
            }

            dst_address.sin_family = AF_INET;
            dst_address.sin_addr.s_addr = inet_addr(ips[n].c_str());
            dst_address.sin_port = htons(ports[n]);

            if(bind(recv_socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address)) < 0)
            {
                //TODO: make this error more descriptive
                std::cerr << "Unable to bind ip adress, receive may not work. \n IP: " << ips[n] << std::endl;
            } else {
                std::cout << "Successfully bind to socket" << std::endl;
            }
            recv_sockets.push_back(recv_socket_fd);
        }

        for(size_t n = 0; n < NUM_CHANNELS; n++) {
            ch_recv_buffer_info tmp = {
                std::vector<std::vector<int8_t>>(MAX_PACKETS_AT_A_TIME, std::vector<int8_t>(HEADER_SIZE, 0)), // headers
                std::vector<size_t>(MAX_PACKETS_AT_A_TIME, 0), // data_bytes_from_packet
                std::vector<int8_t>(MAX_DATA_PER_PACKET, 0), // sample_cache
                (uint64_t) 0 // sample_cache_used
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
        std::cout << "nsamps_per_buff: " << nsamps_per_buff << std::endl;
        size_t bytes_per_buff = nsamps_per_buff * BYTES_PER_SAMPLE;

        std::vector<size_t> nsamps_received(NUM_CHANNELS, 0);

        // received data for every channel sequentially
        // TODO: experiment if parallelizing this helps performance
        for(size_t ch = 0; ch < NUM_CHANNELS; ch++) {
            // Copies cached data from previous recv
            size_t cached_bytes_to_copy = std::min(ch_recv_buffer_info_group[ch].sample_cache_used, bytes_per_buff);
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
            size_t num_bytes_received = recv_multiple_packets(ch, buffs[ch]+cached_bytes_to_copy, remaining_nbytes_per_buff, timeout);
            std::cout << "num_bytes_received: " << num_bytes_received << std::endl;

            // TODO: set this correctly once the rest of the code is completed
            nsamps_received[ch] += num_bytes_received / BYTES_PER_SAMPLE;
        }

        // TODO: extract metadata and set error codes

        // TODO: drop samples in the event of an overflow to keep buffers aligned

        //TODO: fix endianness(swap A, B, C,D bytes to be B, A, D, C

        // Returns the number of samples received on the channel with the lowest number of samples
        size_t lowest_nsamps_received = 0;
        for(size_t n = 0; n < nsamps_received.size(); n++) {
            lowest_nsamps_received = std::min(lowest_nsamps_received, nsamps_received[n]);
        }
        return lowest_nsamps_received;
    }

    /*******************************************************************
     * recv_multiple_packets:
     * channel: which channel to receive data for
     * receives multiple packets on a given channel
     * sample_buffer: start of location in memory to store samples from received packets
     * timeout: timout, TODO: make sure call for other channels take into account time taken by the previous channel
     * one_packet: only receive one packet
     * returns the number of bytes written to buffer (does not include bytes written to the cache
     ******************************************************************/
    UHD_INLINE size_t recv_multiple_packets(size_t channel, void* sample_buffer, size_t nbytes_per_buff, double timeout) {

        // TODO: currently being written to write directly to the buffer to return to the user, need to implement a conversion for other cpu formats
        // Pointers for where to write samples to from each packet using scatter gather
        std::vector<void*> samples_sg_dst;

        for(size_t p = 0; p < nbytes_per_buff; p += MAX_DATA_PER_PACKET) {
            samples_sg_dst.push_back(p+sample_buffer);
        }

        size_t num_packets_to_recv = samples_sg_dst.size();
        // TODO: resize metadata buffer as needed
        if(num_packets_to_recv > MAX_PACKETS_AT_A_TIME) {
            std::cerr << "Receive metadata collection buffer resizing not implemented yet" << std::endl;
            std::exit(~0);
        }

        // Stores data about the recv for each packet
        struct mmsghdr msgs[num_packets_to_recv];
        // Contains data about where to store received data
        // Alternating between pointer to header, pointer to data
        struct iovec iovecs[2*num_packets_to_recv+1];

        memset(msgs, 0, sizeof(msgs));
        for (size_t n = 0; n < num_packets_to_recv - 1; n++) {
            // Location to write header data to
            iovecs[2*n].iov_base =ch_recv_buffer_info_group[channel].headers[n].data();
            iovecs[2*n].iov_len = HEADER_SIZE;
            // Location to write sample data to
            iovecs[2*n+1].iov_base = samples_sg_dst[n];
            iovecs[2*n+1].iov_len = MAX_DATA_PER_PACKET;
            msgs[n].msg_hdr.msg_iov = &iovecs[2*n];
            msgs[n].msg_hdr.msg_iovlen = 2;
        }

        size_t n_last_packet = num_packets_to_recv - 1;
        size_t excess_data_in_last_packet = num_packets_to_recv * MAX_DATA_PER_PACKET - nbytes_per_buff;
        // Location to write header data to
        iovecs[2*n_last_packet].iov_base =ch_recv_buffer_info_group[channel].headers[n_last_packet].data();
        iovecs[2*n_last_packet].iov_len = HEADER_SIZE;
        // Location to write sample data to
        iovecs[2*n_last_packet+1].iov_base = samples_sg_dst[n_last_packet];
        iovecs[2*n_last_packet+1].iov_len = MAX_DATA_PER_PACKET - excess_data_in_last_packet;
        // Location to write samples that don't fit in sample_buffer to
        iovecs[2*n_last_packet+2].iov_base = ch_recv_buffer_info_group[channel].sample_cache.data();
        iovecs[2*n_last_packet+2].iov_len = excess_data_in_last_packet;
        msgs[n_last_packet].msg_hdr.msg_iov = &iovecs[2*n_last_packet];
        msgs[n_last_packet].msg_hdr.msg_iovlen = 3;

        struct timespec ts_timeout{(int)timeout, (int) ((timeout - ((int)timeout))*1000000000)};

        // TODO: consider enabling return after any packets read so give other channels a chance to receive data, before making the thing calling this function repeat to fill up the remaining space in the buffer
        int num_packets_received = recvmmsg(recv_sockets[channel], msgs, num_packets_to_recv, 0, &ts_timeout);

        std::cout << "RECV BUF 1" << std::endl;
        for (int i = 0; i < 30; i++) {
            printf("%02x ", ((unsigned char *) iovecs[1].iov_base)[i]);
        }
        printf("\n");

        if(num_packets_received == -1) {
            std::cout << "recvmmsg error" << std::endl;
            std::cout << "errno" << strerror(errno) << std::endl;
            return 0;
        }

        size_t num_bytes_received = 0;
        for(size_t n = 0; n < num_packets_to_recv; n++) {
            // Check if an invalid packet was received
            if(msgs[n].msg_len < HEADER_SIZE) {
                throw std::runtime_error("Received sample packet smaller than header size");
            }
            uint32_t num_bytes_this_packets = msgs[n].msg_len - HEADER_SIZE;
            if(num_bytes_this_packets != MAX_DATA_PER_PACKET) {
                // TODO: add support for packets not of max length
                std::cerr << "Only max length packets supported, pretending packet is max length" << std::endl;
                num_bytes_received = MAX_DATA_PER_PACKET;
            }
            // Don't count the data stored in sample_cache
            if(n + 1 == num_packets_to_recv) {
                num_bytes_received += num_bytes_this_packets - excess_data_in_last_packet;
            } else {
                num_bytes_received += num_bytes_this_packets;
            }
        }

        ch_recv_buffer_info_group[channel].sample_cache_used = excess_data_in_last_packet;

        return num_bytes_received;
    }

private:
    int MAX_DATA_PER_PACKET;
    int HEADER_SIZE;
    int MAX_PACKETS_AT_A_TIME;
    size_t NUM_CHANNELS;
    size_t BYTES_PER_SAMPLE;
    std::vector<size_t> channels;
    std::vector<int> recv_sockets;
    // Stores information about packets received for each channel
    // Note: this is not meant to be persistent between reads, it is done this way to avoid deallocating and reallocating memory
    struct ch_recv_buffer_info {
        // Stores the headers of each packet
        std::vector<std::vector<int8_t>> headers;
        //Stores how many bytes of sample data are from each packet
        std::vector<size_t> data_bytes_from_packet;
        // Stores extra data from packets between recvs
        std::vector<int8_t> sample_cache;
        // Stores amount of extra data cached from previous recv in byte
        uint64_t sample_cache_used;
    };
    // Group of recv info for each channels
    std::vector<ch_recv_buffer_info> ch_recv_buffer_info_group;


};

class recv_packet_streamer_mmsg : public recv_packet_handler_mmsg, public rx_streamer
{
public:
    recv_packet_streamer_mmsg(const size_t max_num_samps)
    {
        std::cout << "T10" << std::endl;
    }

    //Consider merging recv_packet_streamer_mmsg and recv_packet_handler_mmsg
    //This is here to implement a virtual function from rx_streamer
    size_t recv(const rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet) override
    {
        std::cout << "T80" << std::endl;
        return recv_packet_handler_mmsg::recv(
            buffs, nsamps_per_buff, metadata, timeout, one_packet);
    }
};

}}} // namespace uhd::transport::sph

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP */
