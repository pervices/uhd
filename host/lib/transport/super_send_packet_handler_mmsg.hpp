//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_MMSG_HPP
#define INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_MMSG_HPP

#include "../../transport/super_send_packet_handler.hpp"
#include <uhd/config.hpp>
#include <uhd/exception.hpp>
#include <uhd/convert.hpp>
#include <uhd/stream.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>

#include <cmath>

namespace uhd {
namespace transport {
namespace sph {

/***********************************************************************
 * Super send packet handler
 *
 * A send packet handler represents a group of channels.
 * The channel group shares a common sample rate.
 * All channels are sent in unison in send().
 **********************************************************************/

class send_packet_handler_mmsg : public send_packet_handler
{
public:

    /*!
     * Make a new packet handler for send
     * \param buffer_size size of the buffer on the unit
     */
    send_packet_handler_mmsg(const std::vector<size_t>& channels, size_t max_samples_per_packet, const size_t device_buffer_size)
        : send_packet_handler(device_buffer_size), _max_samples_per_packet(max_samples_per_packet), _num_channels(channels.size())
    {
        std::cout << "T50" << std::endl;
        // TODO IP and port are parameter in constructor
        std::vector<std::string> dst_ips = {"10.10.10.2"};
        std::vector<int> dst_ports = {42836};
        
        // Creates and binds to sockets
        for(size_t n = 0; n < _num_channels; n++) {
            struct sockaddr_in dst_address;
            int send_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if(send_socket_fd < 0) {
                throw uhd::runtime_error( "Failed to create send socket. Error code:" + std::string(strerror(errno)));
            }

            dst_address.sin_family = AF_INET;
            dst_address.sin_addr.s_addr = inet_addr(dst_ips[n].c_str());
            dst_address.sin_port = htons(dst_ports[n]);

            if(connect(send_socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address)) < 0)
            {
                std::cerr << "Unable to bind send ip adress, receive may not work. \n IP: " << dst_ips[n] << " " <<  std::string(strerror(errno)) << std::endl;
            }

            // TODO: implement send buffer resizing

            send_sockets.push_back(send_socket_fd);
        }

    }

    ~send_packet_handler_mmsg(void){
        for(size_t n = 0; n < send_sockets.size(); n++) {
            close(send_sockets[n]);
        }
    }


    /*******************************************************************
     * Send:
     * The entry point for the fast-path send calls.
     * Dispatch into combinations of single packet send calls.
     ******************************************************************/
    UHD_INLINE size_t send(
        const uhd::tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    ) {
        // TODO: implement send
        std::cout << "Start of new send" << std::endl;
        return 0;
    }
    
protected:
    size_t _max_samples_per_packet;
    size_t _num_channels;

private:
    std::vector<int> send_sockets;

};

class send_packet_streamer_mmsg : public send_packet_handler_mmsg, public tx_streamer
{
public:
    send_packet_streamer_mmsg(const std::vector<size_t>& channels, size_t max_samples_per_packet, const size_t device_buffer_size):
    sph::send_packet_handler_mmsg(channels, max_samples_per_packet, device_buffer_size)
    {
    }

    size_t get_num_channels(void) const{
        return _num_channels;
    }

    size_t get_max_num_samps(void) const{
        return _max_samples_per_packet;
    }

    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata,
        const double timeout
    ){
        return send_packet_handler::send(buffs, nsamps_per_buff, metadata, timeout);
    }
    
    //TODO; figure out what this async message is (probably getting buffer level)
    bool recv_async_msg(
        uhd::async_metadata_t &async_metadata, double timeout = 0.1
    ){
        return send_packet_handler_mmsg::recv_async_msg(async_metadata, timeout);
    }
};

} // namespace sph
} // namespace transport
} // namespace uhd

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_MMSG_HPP */
