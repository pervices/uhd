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

#include <netinet/in.h>
#include <arpa/inet.h>

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
    : recv_packet_handler(size), MAX_DATA_PER_PACKET(9000), HEADER_SIZE(200), MAX_PACKETS_AT_A_TIME(32)
    {
        std::cout << "T50" << std::endl;
        //TODO: get this as a parameter
        channels = {0};
        std::vector<std::string> ips = {"10.10.10.10"};
        std::vector<int> ports = {42836};

        // Creates and binds to sockets
        for(size_t n = 0; n < channels.size(); n++) {
            struct sockaddr_in dst_address;
            int recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if(recv_socket_fd < 0) {
                std::cerr << "Failed to create recv socket" << std::endl;
                std::exit(1);
            }

            dst_address.sin_family = AF_INET;
            dst_address.sin_addr.s_addr = inet_addr(ips[n].c_str());
            dst_address.sin_port = htons(ports[n]);

            if(bind(recv_socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address)) < 0)
            {
                std::cerr << "Unable to bind ip adress, receive may not work. \n IP: " << ips[n] << std::endl;
            } else {
                std::cout << "Successfully bind to socket" << std::endl;
            }
            recv_sockets.push_back(recv_socket_fd);
        }

        for(size_t n = 0; n < channels.size(); n++) {
            ch_recv_buffer_info tmp = {
                std::vector<std::vector<int8_t>>(MAX_PACKETS_AT_A_TIME, std::vector<int8_t>(HEADER_SIZE, 0)), //headers
                std::vector<int8_t>(MAX_DATA_PER_PACKET, 0), // data_cache
                (uint64_t) 0 //data_cache_used
            };
            ch_recv_buffer_info_group.push_back(tmp);
        }

    }

    //TODO: unbind ports in destructor

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
        std::cout << "T100" << std::endl;

        return 0;
    }

private:
    int MAX_DATA_PER_PACKET;
    int HEADER_SIZE;
    int MAX_PACKETS_AT_A_TIME;
    std::vector<size_t> channels;
    std::vector<int> recv_sockets;
    struct ch_recv_buffer_info {
        std::vector<std::vector<int8_t>> headers;
        // Stores extra data between recv
        std::vector<int8_t> data_cache;
        // Stores amount of extra data cached from previous recv
        uint64_t data_cache_used;
    };
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
