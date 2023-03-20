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
// Declare constants first so they are initialized before constructor
private:
    const size_t _bytes_per_sample = 4;
    // Size of the vrt header in bytes
    const size_t HEADER_SIZE = 12;
public:

    /*!
     * Make a new packet handler for send
     * \param buffer_size size of the buffer on the unit
     */
    send_packet_handler_mmsg(const std::vector<size_t>& channels, size_t max_samples_per_packet, const size_t device_buffer_size)
        : send_packet_handler(device_buffer_size), _max_samples_per_packet(max_samples_per_packet), _max_sample_bytes_per_packet(max_samples_per_packet * _bytes_per_sample), _num_channels(channels.size())
    {
        std::cout << "_max_samples_per_packet: " << _max_samples_per_packet << std::endl;
        std::cout << "max_samples_per_packet: " << max_samples_per_packet << std::endl;
        std::cout << "_max_sample_bytes_per_packet: " << _max_sample_bytes_per_packet << std::endl;
        std::cout << "_bytes_per_sample: " << _bytes_per_sample << std::endl;
        // TODO get these are parameters in constructor
        std::vector<std::string> dst_ips = {"10.10.10.2"};
        std::vector<int> dst_ports = {42836};
        child_channels = std::vector<size_t>(1, 0);
        
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
        const uhd::tx_streamer::buffs_type &sample_buffs,
        const size_t nsamps_to_send,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    ) {
        std::cout << "nsamps_to_send: " << nsamps_to_send << std::endl;
        // TODO: implement handling for length 0 packets (SOB with no samples and EOB)
        if(nsamps_to_send == 0) {
            std::cout << "0 length packets not implemented yet" << std::endl;
            return 0;
        }

        // Number of packets to send
        int num_packets = std::ceil(((double)nsamps_to_send)/_max_samples_per_packet);

        size_t samples_in_last_packet = nsamps_to_send - (_max_samples_per_packet * (num_packets - 1));

        //TODO: make this work for multiple channels
        // VRT header info for data packets
        std::vector<vrt::if_packet_info_t> packet_header_infos(num_packets);
        for(int n = 0; n < num_packets; n++) {
            packet_header_infos[n].packet_type = vrt::if_packet_info_t::PACKET_TYPE_DATA;
            packet_header_infos[n].has_sid = false;
            packet_header_infos[n].has_cid = false;
            packet_header_infos[n].has_tlr = false; // No trailer
            packet_header_infos[n].has_tsi = false; // No integer timestamp
            packet_header_infos[n].has_tsf = true; // FPGA requires all data packets have fractional timestamps
            packet_header_infos[n].tsf = (metadata_.time_spec + time_spec_t::from_ticks(num_packets * _max_samples_per_packet, _samp_rate)).to_ticks(_tick_rate);
            packet_header_infos[n].sob = (n == 0) && metadata_.start_of_burst;
            // TODO: implement EOB, note EOB packets must not contain real samples but must contain some data
            packet_header_infos[n].eob     = false;
            packet_header_infos[n].fc_ack  = false; // Is not a flow control packet

            packet_header_infos[n].num_payload_bytes = _max_sample_bytes_per_packet;
            packet_header_infos[n].num_payload_words32 = (_max_sample_bytes_per_packet + 3/*round up*/)/sizeof(uint32_t);
        }

        //Set payload size info for last packet
        packet_header_infos[num_packets - 1].num_payload_bytes = samples_in_last_packet * _bytes_per_sample;
        packet_header_infos[num_packets - 1].num_payload_words32 = ((samples_in_last_packet*_bytes_per_sample) + 3/*round up*/)/sizeof(uint32_t);

        std::vector<std::vector<uint32_t>> vrt_headers(num_packets, std::vector<uint32_t>(HEADER_SIZE/sizeof(uint32_t), 0));

        for(int n = 0; n < num_packets; n++) {
            if_hdr_pack(vrt_headers[n].data(), packet_header_infos[n]);
        }

        // Pointer to the start of the data to send in each packet for each channels
        std::vector<const void*> sample_data_start_for_packet(num_packets);
        for(int n = 0; n < num_packets; n++) {
            //TODO: sample_buffs 0 is for ch 0, make this work for more channels
            sample_data_start_for_packet[n] = sample_buffs[0] + (n * _max_sample_bytes_per_packet);
        }

        mmsghdr msgs[num_packets];
        // Pointers to buffers
        // 0 points to header of the first packet, 1 to data, 2 to header of second packet...
        iovec iovecs[2*num_packets];

        std::cout << "_max_sample_bytes_per_packet: " << _max_sample_bytes_per_packet << std::endl;
        for(int n = 0; n < num_packets - 1; n++) {
            // VRT Header
            iovecs[2*n].iov_base = vrt_headers[n].data();
            iovecs[2*n].iov_len = HEADER_SIZE;
            // Samples
            // iovecs.iov_base is const for all practical purposes, const_cast is used to allow it to use data from the buffer which is const
            iovecs[2*n+1].iov_base = const_cast<void*>(sample_data_start_for_packet[n]);
            iovecs[2*n+1].iov_len = _max_sample_bytes_per_packet;

            msgs[n].msg_hdr.msg_iov = &iovecs[2*n];
            msgs[n].msg_hdr.msg_iovlen = 2;

            // Setting optional data to none
            msgs[n].msg_hdr.msg_name = NULL;
            msgs[n].msg_hdr.msg_namelen = 0;
            msgs[n].msg_hdr.msg_control = NULL;
            msgs[n].msg_hdr.msg_controllen = 0;
        }

        int n_last_packet = num_packets - 1;
        iovecs[2*n_last_packet].iov_base = vrt_headers[n_last_packet].data();
        iovecs[2*n_last_packet].iov_len = HEADER_SIZE;

        iovecs[2*n_last_packet+1].iov_base = const_cast<void*>(sample_data_start_for_packet[n_last_packet]);
        iovecs[2*n_last_packet+1].iov_len = samples_in_last_packet * _bytes_per_sample;

        msgs[n_last_packet].msg_hdr.msg_iov = &iovecs[2*n_last_packet];
        msgs[n_last_packet].msg_hdr.msg_iovlen = 2;

        msgs[n_last_packet].msg_hdr.msg_name = NULL;
        msgs[n_last_packet].msg_hdr.msg_namelen = 0;
        msgs[n_last_packet].msg_hdr.msg_control = NULL;
        msgs[n_last_packet].msg_hdr.msg_controllen = 0;

        size_t channels_serviced = 0;
        std::vector<int> packets_sent_per_ch(_num_channels, 0);
        std::vector<size_t> samples_sent_per_ch(_num_channels, 0);
        while(channels_serviced < _num_channels) {
            for(size_t ch_i = 0; ch_i < _num_channels; ch_i++) {
                size_t ch = child_channels[ch_i];
                // TODO: change check_flow_control to get the number of samples that can be sent now instead of a simple true/false, this is for future code that will prevent large send buffers from causing overflows on Crimson
                if (!(_props.at(ch).check_flow_control(0))) {
                    // The time to send for this channel has not reached.
                    continue;
                }

                int num_packets_alread_sent = packets_sent_per_ch[ch_i];
                int num_packets_to_send = num_packets - num_packets_alread_sent;
                int num_packets_sent_this_send = sendmmsg(send_sockets[ch_i], &msgs[num_packets_alread_sent], num_packets_to_send, 0);
                std::cout << "num_packets_sent_this_send: " << num_packets_sent_this_send << std::endl;

                if(num_packets_sent_this_send < 0) {
                    std::cerr << "sendmmsg on ch " << child_channels[ch_i] << "failed with error: " << std::strerror(errno) << std::endl;
                } else {
                    packets_sent_per_ch[ch_i] += num_packets_sent_this_send;
                    // Update buffer level record
                    size_t nsamps_sent;
                    if(num_packets_to_send == num_packets_sent_this_send) {
                        // This send included the last packet (which may not be the maximum length)
                        nsamps_sent = ((num_packets_sent_this_send - 1) * _max_samples_per_packet) + samples_in_last_packet;
                        channels_serviced+=1;
                    } else {
                        // This send did not include the max packet (which which always be the maximum length)
                        nsamps_sent = num_packets_sent_this_send * _max_samples_per_packet;
                    }
                    // Update counter for number of samples sent this send
                    samples_sent_per_ch[ch_i] += nsamps_sent;
                    // Update buffer level count
                    _props.at(child_channels[ch_i]).update_fc_send_count(nsamps_sent);
                }
            }
        }

        // All channels should always send the same number of samples
        std::cout << "samples_sent_per_ch[0]: " << samples_sent_per_ch[0] << std::endl;
        return samples_sent_per_ch[0];
    }
    
protected:
    size_t _max_samples_per_packet;
    size_t _max_sample_bytes_per_packet;
    size_t _num_channels;

    /*******************************************************************
     * converts vrt packet info into header
     * packet_buff: buffer to write vrt data to
     * if_packet_info: packet info to be used to calculate the header
     ******************************************************************/
    virtual void if_hdr_pack(uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

private:
    std::vector<int> send_sockets;
    //TODO: rename this to just channels when seperating this class from old version
    std::vector<size_t> child_channels;

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
