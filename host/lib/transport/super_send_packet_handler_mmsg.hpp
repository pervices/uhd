//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_MMSG_HPP
#define INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_MMSG_HPP

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

#include <uhd/transport/buffer_tracker.hpp>
#include <uhd/transport/bounded_buffer.hpp>

#include <uhdlib/utils/system_time.hpp>

#define MIN_MTU 9000

namespace uhd {
namespace transport {
namespace sph {

    // Socket priority for tx sockets
    // Highest possible thread priority without CAP_NET_ADMIN
    const int TX_SO_PRIORITY = 6;

/***********************************************************************
 * Super send packet handler
 *
 * A send packet handler represents a group of channels.
 * The channel group shares a common sample rate.
 * All channels are sent in unison in send().
 **********************************************************************/

class send_packet_handler_mmsg
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
    send_packet_handler_mmsg(const std::vector<size_t>& channels, ssize_t max_samples_per_packet, const int64_t device_buffer_size, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, ssize_t device_packet_nsamp_multiple, double tick_rate, const std::shared_ptr<bounded_buffer<async_metadata_t>> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian)
        : _max_samples_per_packet(max_samples_per_packet),
        _MAX_SAMPLE_BYTES_PER_PACKET(max_samples_per_packet * _bytes_per_sample),
        _NUM_CHANNELS(channels.size()),
        _async_msg_fifo(async_msg_fifo),
        _channels(channels),
        _DEVICE_BUFFER_SIZE(device_buffer_size),
        _DEVICE_TARGET_NSAMPS(device_target_nsamps),
        _DEVICE_PACKET_NSAMP_MULTIPLE(device_packet_nsamp_multiple),
        _TICK_RATE(tick_rate),
        _intermediate_send_buffer_pointers(_NUM_CHANNELS),
        _intermediate_send_buffer_wrapper(_intermediate_send_buffer_pointers.data(), _NUM_CHANNELS)
    {
        ch_send_buffer_info_group = std::vector<ch_send_buffer_info>(_NUM_CHANNELS, ch_send_buffer_info(0, HEADER_SIZE, _bytes_per_sample * (_DEVICE_PACKET_NSAMP_MULTIPLE - 1), _DEVICE_TARGET_NSAMPS, _sample_rate));

        // Creates and binds to sockets
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
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
                fprintf(stderr, "ERROR Unable to connect to IP address %s and port %i\n", dst_ips[n].c_str(), dst_ports[n]);
                if(errno == EADDRINUSE) {
                    fprintf(stderr, "Address already in use. This is usually caused by attempting to run multiple UHD programs at once\n");
                } else {
                    fprintf(stderr, "Connect failed with error: %s\n", strerror(errno));
                }
            }

            // Sets the recv buffer size
            setsockopt(send_socket_fd, SOL_SOCKET, SO_SNDBUF, &_DEFAULT_SEND_BUFFER_SIZE, sizeof(_DEFAULT_SEND_BUFFER_SIZE));

            // Checks the recv buffer size
            socklen_t opt_len = sizeof(_ACTUAL_SEND_BUFFER_SIZE);
            getsockopt(send_socket_fd, SOL_SOCKET, SO_SNDBUF, &_ACTUAL_SEND_BUFFER_SIZE, &opt_len);

            // NOTE: The kernel will set the actual size to be double the requested. So the expected amount is double the requested
            if(_ACTUAL_SEND_BUFFER_SIZE < 2*_DEFAULT_SEND_BUFFER_SIZE) {
                fprintf(stderr, "Unable to set send buffer size. Performance may be affected\nTarget size %i\nActual size %i\nPlease run \"sudo sysctl -w net.core.wmem_max=%i\"\n", _DEFAULT_SEND_BUFFER_SIZE, _ACTUAL_SEND_BUFFER_SIZE/2, _DEFAULT_SEND_BUFFER_SIZE);
            }

            int mtu = get_mtu(send_socket_fd, dst_ips[n].c_str());
            if(mtu < MIN_MTU) {
                fprintf(stderr, "MTU of interface associated with %s is to small. %i required, current value is %i", dst_ips[n].c_str(), MIN_MTU, mtu);
                throw uhd::system_error("MTU size to small");
            }

            int set_priority_ret = setsockopt(send_socket_fd, SOL_SOCKET, SO_PRIORITY, &TX_SO_PRIORITY, sizeof(TX_SO_PRIORITY));
            if(set_priority_ret) {
                fprintf(stderr, "Attempting to set tx socket priority failed with error code: %s", strerror(errno));
            }

            send_sockets.push_back(send_socket_fd);
        }

        setup_converter(cpu_format, wire_format, wire_little_endian);
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
private:
    bool cached_sob = false;
    uhd::time_spec_t sob_time_cache;
public:
    UHD_INLINE size_t send(
        const uhd::tx_streamer::buffs_type &sample_buffs,
        const size_t nsamps_to_send,
        const uhd::tx_metadata_t &metadata,
        const double timeout
    ) {
        // If no converter is required data will be written directly into buffs, otherwise it is written to an intermediate buffer
        const uhd::tx_streamer::buffs_type *send_buffer = (converter_used) ? prepare_intermediate_buffers_and_convert(sample_buffs, nsamps_to_send) : &sample_buffs;

        size_t previous_cached_nsamps = cached_nsamps;

        // FPGAs can sometimes only receive multiples of a set number of samples
        size_t actual_nsamps_to_send = (((cached_nsamps + nsamps_to_send) / _DEVICE_PACKET_NSAMP_MULTIPLE) * _DEVICE_PACKET_NSAMP_MULTIPLE);
        size_t nsamps_to_cache = nsamps_to_send + cached_nsamps - actual_nsamps_to_send;

        if(actual_nsamps_to_send == 0) {
            // If a start of burst command has no packets, cache timestamp and keep until next call
            if(metadata.start_of_burst) {
                cached_sob = true;
                sob_time_cache = metadata.time_spec;
                return 0;
            } else if(metadata.end_of_burst) {
                send_eob_packet(metadata, timeout);
                return 0;
            } else {
                return 0;
            }
        }

        uhd::tx_metadata_t modified_metadata = metadata;
        if(cached_sob) {
            cached_sob = false;
            modified_metadata.start_of_burst = true;
            modified_metadata.has_time_spec = true;
            modified_metadata.time_spec = sob_time_cache;
        }
        // FPGA cannot handle eob request and samples. Samples must be sent before end of burst
        bool eob_requested = false;
        if(modified_metadata.end_of_burst) {
            modified_metadata.end_of_burst = false;
            eob_requested = true;
        }

        // Create and sends packets
        size_t actual_samples_sent = send_multiple_packets(*send_buffer, actual_nsamps_to_send, modified_metadata, timeout);

        // Sends the eob if requested
        if(eob_requested) {
            modified_metadata.end_of_burst = true;
            send_eob_packet(metadata, timeout);
        }

        size_t cached_samples_sent;
        // Copies samples that won't fit as a multiple of _DEVICE_PACKET_NSAMP_MULTIPLE to the cache
        if(actual_samples_sent == 0) {
            // Do not update the cache if no samples were actually sent
            cached_samples_sent = 0;
        } else if(actual_samples_sent < previous_cached_nsamps) {
            // If fewer samples were sent than were in the cache move the remaining samples to front of the cache
            for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
                memmove(ch_send_buffer_info_group[ch_i].sample_cache.data(), ch_send_buffer_info_group[ch_i].sample_cache.data() + actual_samples_sent, (previous_cached_nsamps - actual_samples_sent) * _bytes_per_sample);
            }
            cached_samples_sent = actual_samples_sent;
            cached_nsamps = previous_cached_nsamps - actual_samples_sent;
        } else if(actual_nsamps_to_send < actual_samples_sent) {
            // If not the samples meant to actually be sent were sent, clear the cache and do not cache any samples
            // The sample cache is meant to handle the case where the send was successful, but the number of samples the user requested isn't a multiple of the required amount
            // Since in this case the send didn't send all the intended samples anyway, we don't need to bother with the cache
            cached_nsamps = 0;
            cached_samples_sent = 0;
        }
        // TODO: handle case where some but not all of the samples to be cached were sent
        else if(actual_samples_sent == actual_nsamps_to_send) {
            // Since send was fully successful, copy samples that couldn't be sent this send due to limitations on packet sizing to the cache
            if(nsamps_to_cache > 0) {
                for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
                    memcpy(ch_send_buffer_info_group[ch_i].sample_cache.data(), (uint8_t*)((*send_buffer)[ch_i]) + (actual_samples_sent * _bytes_per_sample), nsamps_to_cache * _bytes_per_sample);
                }
            }
            cached_samples_sent = previous_cached_nsamps;
            cached_nsamps = nsamps_to_cache;
        } else {
            fprintf(stderr, "ERROR, more samples sent than intended. This should be impossible, contact support\n");
            // Reaching here should be impossible, these values don't matter
            cached_samples_sent = 0;
            cached_nsamps = 0;
        }

        // Returns number of samples sent + any samples added to the cache this send - samples from the cache in the previous send
        return actual_samples_sent + cached_nsamps - cached_samples_sent;
    }

    void set_samp_rate(const double rate) {
        _sample_rate = rate;
        for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
            ch_send_buffer_info_i.buffer_level_manager.set_sample_rate(rate);
        }
    }

    void update_buffer_level(const uint64_t ch, const uint64_t level, const uhd::time_spec_t & now) {
        ch_send_buffer_info_group[ch].buffer_level_manager.update_buffer_level_bias(level, now);
    }

    void enable_blocking_fc(int64_t blocking_setpoint) {
        use_blocking_fc = true;
        if(blocking_setpoint > 0.9 * _DEVICE_BUFFER_SIZE) {
            blocking_setpoint = (uint64_t) (0.9*_DEVICE_BUFFER_SIZE);
        };
        this->blocking_setpoint = blocking_setpoint;
    }

    void disable_blocking_fc() {
        use_blocking_fc = false;
    }
    
protected:
    bool use_blocking_fc = false;
    ssize_t _max_samples_per_packet;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _NUM_CHANNELS;

    // Buffer containing asynchronous messages related to underflows/overflows
    const std::shared_ptr<bounded_buffer<async_metadata_t>> _async_msg_fifo;

    // Gets the the time on the unit when a packet sent now would arrive
    virtual uhd::time_spec_t get_time_now() = 0;

    /*******************************************************************
     * converts vrt packet info into header
     * packet_buff: buffer to write vrt data to
     * if_packet_info: packet info to be used to calculate the header
     ******************************************************************/
    virtual void if_hdr_pack(uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

    // Sends a request for the buffer level from the device, returns the result of that request
    virtual int64_t get_buffer_level_from_device(const size_t ch_i) = 0;

private:
    int64_t blocking_setpoint = 0;

    //TODO: adjust this dynamically (currently everything uses 4 byte tx so it doesn't matter for now)
    const size_t _BYTES_PER_SAMPLE = 4;
    // TODO dynamically adjust send buffer size based on system RAM, number of channels, and unit buffer size
    // Desired send buffer size
    const int _DEFAULT_SEND_BUFFER_SIZE = 50000000;
    // Actual recv buffer size, not the Kernel will set the real size to be double the requested
    int _ACTUAL_SEND_BUFFER_SIZE;
    // Maximum number of packets to recv (should be able to fit in the half the real buffer)
    std::vector<int> send_sockets;
protected:
    std::vector<size_t> _channels;
private:
    // Device buffer size
    const int64_t _DEVICE_BUFFER_SIZE;

    // Desired number of samples in the tx buffer on the unit
    const ssize_t _DEVICE_TARGET_NSAMPS;

    // Number of packets per packet must be a multiple of this. Excess are cached and sent in the next send
    const size_t _DEVICE_PACKET_NSAMP_MULTIPLE;

    const double _TICK_RATE;
    // Number of samples cached between sends to account for _DEVICE_PACKET_NSAMP_MULTIPLE restriction
    size_t cached_nsamps = 0;

    double _sample_rate = 0;

    // Sequence number for next packet
    uint64_t next_sequence_number = 0;
    // Header info for each packet, the VITA (not UDP) header is the same for every channel
    std::vector<vrt::if_packet_info_t> packet_header_infos;

    // The start time of the next batch of samples in ticks
    // The FPGA requires a timestampt always be present in packets. This is used to figureout the timestamp when not specified by the user
    uhd::time_spec_t next_send_time = uhd::time_spec_t(0.0);

    //TODO move all the vectors with channel specific info here
    // Stores information about packets to send for each channel
    // Sizes of the various buffers used in send
    size_t send_buffer_info_size = 0;
    struct ch_send_buffer_info {
        const size_t _vrt_header_size;
        // Stores samples between sends, to account for limitations in number samples that can be sent at once
        std::vector<int8_t> sample_cache;
        // Stores data about the send for each packet
        std::vector<mmsghdr> msgs;
        // 0 points to header of the first packet, 1 to data, 2 to header of second packet...
        std::vector<iovec> iovecs;

        // Contains vrt header data for each packet
        std::vector<std::vector<uint32_t>> vrt_headers;

        // Stores where the samples start for each packet
        std::vector<const void*> sample_data_start_for_packet;

        // Calculates the predicted buffer level
        buffer_tracker buffer_level_manager;

        // Buffer used to store data before converting from wire format to CPU format. Unused if wire and CPU format match
        std::vector<int8_t> intermediate_send_buffer;

        /*!
        * Make a new ch_send_buffer_info
        * \param size number of packets that can be handled at once
        * \param vrt_header_size size of the vrt header
        * \param cache_size number of bytes in the sample cache
        */
        ch_send_buffer_info(const size_t size, const size_t vrt_header_size, const size_t cache_size, const int64_t device_target_nsamps, const double rate)
        : _vrt_header_size(vrt_header_size),
        sample_cache(std::vector<int8_t>(cache_size)),
        buffer_level_manager(device_target_nsamps, rate)
        {
            resize_and_clear(size);
        }

        // Resizes and clears the buffers to match packet_helper_buffer_size
        void resize_and_clear(size_t new_size) {
            msgs.resize(new_size);
            memset(msgs.data(), 0, sizeof(mmsghdr)*new_size);
            // 1 VRT header and 1 data section in every packet, plus the cached samples in the first packet
            iovecs.resize(2*new_size + 1);
            memset(iovecs.data(), 0, sizeof(iovec)*2*new_size);
            vrt_headers.resize(new_size);
            std::fill(vrt_headers.begin(), vrt_headers.end(), std::vector<uint32_t>(_vrt_header_size/sizeof(uint32_t), 0));
            sample_data_start_for_packet.resize(new_size, 0);
        }

    };
    // Group of recv info for each channels
    std::vector<ch_send_buffer_info> ch_send_buffer_info_group;

    // Whether or not a conversion is required between CPU and wire formats
    bool converter_used;

    // Pointers to the start of the send buffer for each channel
    std::vector<void*> _intermediate_send_buffer_pointers;
    // Wrapper to be use the same dataype as the regular buffer
    uhd::tx_streamer::buffs_type _intermediate_send_buffer_wrapper;

    // Converts samples between wire and cpu formats
    uhd::convert::converter::sptr _converter;

    // Expands the buffers used in the send command, does nothing if already large enough
    void expand_send_buffer_info(size_t new_size) {
        // Resizes the per channel buffers used in the send command
        if(new_size > send_buffer_info_size) {
            send_buffer_info_size = new_size;
            packet_header_infos.resize(new_size);
            for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
                ch_send_buffer_info_group[ch_i].resize_and_clear(new_size);
            }
        }
    }

    // Gets the number of samples that can be sent now (can be less than 0)
    // Also sleeps if start of burst hasn't happened yet and its a long time in the future
    int check_fc_npackets(const size_t ch_i) {
        if(BOOST_LIKELY(!use_blocking_fc)) {

            // Get the buffer level on the unit
            uhd::time_spec_t device_time = get_time_now();
            int64_t buffer_level = ch_send_buffer_info_group[ch_i].buffer_level_manager.get_buffer_level(device_time);

            int num_packets_to_send = (int) ((_DEVICE_TARGET_NSAMPS - buffer_level) / (_max_samples_per_packet));

            if(num_packets_to_send > 0 || !ch_send_buffer_info_group[ch_i].buffer_level_manager.start_of_burst_pending(device_time+1e-6)) {
                return num_packets_to_send;
            } else {
                uhd::time_spec_t sleep_time = ch_send_buffer_info_group[ch_i].buffer_level_manager.time_until_sob(device_time) - 1.0e-6;
                sleep_time.sleep_for();
                return num_packets_to_send;
            }

        } else {
            int64_t buffer_level = get_buffer_level_from_device(ch_i);
            return (int) ((blocking_setpoint - buffer_level) / (_max_samples_per_packet));
        }
    }

    UHD_INLINE size_t send_multiple_packets(
        const uhd::tx_streamer::buffs_type &sample_buffs,
        const size_t nsamps_to_send,
        const uhd::tx_metadata_t &metadata_,
        const double timeout
    ) {

        // Number of packets to send
        int num_packets = std::ceil(((double)nsamps_to_send)/_max_samples_per_packet);

        size_t samples_in_last_packet = nsamps_to_send - (_max_samples_per_packet * (num_packets - 1));

        // Expands size of buffers used to store data to be sent
        expand_send_buffer_info(num_packets);

        // Sets the start os burst time
        if(metadata_.start_of_burst) {
            for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
                ch_send_buffer_info_i.buffer_level_manager.set_start_of_burst_time(metadata_.time_spec);
            }
        }

        for(int n = 0; n < num_packets; n++) {
            packet_header_infos[n].packet_type = vrt::if_packet_info_t::PACKET_TYPE_DATA;
            packet_header_infos[n].packet_count = next_sequence_number;
            next_sequence_number = (next_sequence_number + 1) & 0xf;
            packet_header_infos[n].has_sid = false;
            packet_header_infos[n].has_sid = false;
            packet_header_infos[n].has_cid = false;
            packet_header_infos[n].has_tlr = false; // No trailer
            packet_header_infos[n].has_tsi = false; // No integer timestamp
            packet_header_infos[n].has_tsf = !metadata_.end_of_burst; // Every packet except the end of burst must have a timestamp. End of burst must have no timestamp
            if(metadata_.has_time_spec) {
                // Sets the timestamp based on what's specified by the user
                packet_header_infos[n].tsf = (metadata_.time_spec + time_spec_t::from_ticks(n * _max_samples_per_packet, _sample_rate)).to_ticks(_TICK_RATE);
            } else {
                // Sets the timestamp to follow from the previous send
                packet_header_infos[n].tsf = (next_send_time + time_spec_t::from_ticks(n * _max_samples_per_packet, _sample_rate)).to_ticks(_TICK_RATE);
            }
            packet_header_infos[n].sob = (n == 0) && metadata_.start_of_burst;
            packet_header_infos[n].eob     = metadata_.end_of_burst;
            packet_header_infos[n].fc_ack  = false; // Is not a flow control packet

            packet_header_infos[n].num_payload_bytes = _MAX_SAMPLE_BYTES_PER_PACKET;
            packet_header_infos[n].num_payload_words32 = (_MAX_SAMPLE_BYTES_PER_PACKET + 3/*round up*/)/sizeof(uint32_t);
        }

        //Set payload size info for last packet
        packet_header_infos[num_packets - 1].num_payload_bytes = samples_in_last_packet * _bytes_per_sample;
        packet_header_infos[num_packets - 1].num_payload_words32 = ((samples_in_last_packet*_bytes_per_sample) + 3/*round up*/)/sizeof(uint32_t);

        for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
            for(int n = 0; n < num_packets; n++) {
                if_hdr_pack(ch_send_buffer_info_group[ch_i].vrt_headers[n].data(), packet_header_infos[n]);
            }
        }

        for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
            for(int n = 0; n < num_packets; n++) {
                ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[n] = (uint8_t*)(sample_buffs[ch_i]) + (n * _MAX_SAMPLE_BYTES_PER_PACKET);
            }
        }

        for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
            // Sets up iovecs for the first packets
            // VRT Header
            ch_send_buffer_info_group[ch_i].iovecs[0].iov_base = ch_send_buffer_info_group[ch_i].vrt_headers[0].data();
            ch_send_buffer_info_group[ch_i].iovecs[0].iov_len = HEADER_SIZE;
            // Cached samples
            ch_send_buffer_info_group[ch_i].iovecs[1].iov_base = ch_send_buffer_info_group[ch_i].sample_cache.data();
            ch_send_buffer_info_group[ch_i].iovecs[1].iov_len = cached_nsamps * _bytes_per_sample;
            // Samples
            // iovecs.iov_base is const for all practical purposes, const_cast is used to allow it to use data from the buffer which is const
            ch_send_buffer_info_group[ch_i].iovecs[2].iov_base = const_cast<void*>(ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[0]);
            if(num_packets > 1) {
                ch_send_buffer_info_group[ch_i].iovecs[2].iov_len = _MAX_SAMPLE_BYTES_PER_PACKET - cached_nsamps;
            } else {
                ch_send_buffer_info_group[ch_i].iovecs[2].iov_len = samples_in_last_packet * _bytes_per_sample - cached_nsamps;
            }

            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_iov = &ch_send_buffer_info_group[ch_i].iovecs[0];
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_iovlen = 3;

            // Setting optional data to none
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_name = NULL;
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_namelen = 0;
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_control = NULL;
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_controllen = 0;

            // Sets up iovecs and msg for packets 1 to n -1
            for(int n = 0; n < num_packets - 1; n++) {
                // VRT Header
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n)].iov_base = ch_send_buffer_info_group[ch_i].vrt_headers[n].data();
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n)].iov_len = HEADER_SIZE;
                // Samples
                // iovecs.iov_base is const for all practical purposes, const_cast is used to allow it to use data from the buffer which is const
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n)+1].iov_base = const_cast<void*>(ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[n]);
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n)+1].iov_len = _MAX_SAMPLE_BYTES_PER_PACKET;

                ch_send_buffer_info_group[ch_i].msgs[n].msg_hdr.msg_iov = &ch_send_buffer_info_group[ch_i].iovecs[1+(2*n)];
                ch_send_buffer_info_group[ch_i].msgs[n].msg_hdr.msg_iovlen = 2;

                // Setting optional data to none
                ch_send_buffer_info_group[ch_i].msgs[n].msg_hdr.msg_name = NULL;
                ch_send_buffer_info_group[ch_i].msgs[n].msg_hdr.msg_namelen = 0;
                ch_send_buffer_info_group[ch_i].msgs[n].msg_hdr.msg_control = NULL;
                ch_send_buffer_info_group[ch_i].msgs[n].msg_hdr.msg_controllen = 0;
            }

            // Sets up iovecs and msgs for last packet
            if(num_packets > 1) {
                int n_last_packet = num_packets - 1;
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n_last_packet)].iov_base = ch_send_buffer_info_group[ch_i].vrt_headers[n_last_packet].data();
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n_last_packet)].iov_len = HEADER_SIZE;

                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n_last_packet)+1].iov_base = const_cast<void*>(ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[n_last_packet]);
                ch_send_buffer_info_group[ch_i].iovecs[1+(2*n_last_packet)+1].iov_len = samples_in_last_packet * _bytes_per_sample;

                ch_send_buffer_info_group[ch_i].msgs[n_last_packet].msg_hdr.msg_iov = &ch_send_buffer_info_group[ch_i].iovecs[1+(2*n_last_packet)];
                ch_send_buffer_info_group[ch_i].msgs[n_last_packet].msg_hdr.msg_iovlen = 2;

                ch_send_buffer_info_group[ch_i].msgs[n_last_packet].msg_hdr.msg_name = NULL;
                ch_send_buffer_info_group[ch_i].msgs[n_last_packet].msg_hdr.msg_namelen = 0;
                ch_send_buffer_info_group[ch_i].msgs[n_last_packet].msg_hdr.msg_control = NULL;
                ch_send_buffer_info_group[ch_i].msgs[n_last_packet].msg_hdr.msg_controllen = 0;
            }
        }

        // Gets the start time for use in the timeout, uses CLOCK_MONOTONIC_COARSE because it is faster and precision doesn't matter for timeouts
        struct timespec send_start_time;
        clock_gettime(CLOCK_MONOTONIC_COARSE, &send_start_time);
        int64_t send_timeout_time_ns = (send_start_time.tv_sec * 1000000000) + send_start_time.tv_nsec + (int64_t)(timeout * 1000000000);

        size_t channels_serviced = 0;
        std::vector<int> packets_sent_per_ch(_NUM_CHANNELS, 0);
        std::vector<size_t> samples_sent_per_ch(_NUM_CHANNELS, 0);

        while(channels_serviced < _NUM_CHANNELS) {

            // Sends packets for each channel
            for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
                int packets_to_send_this_sendmmsg = check_fc_npackets(ch_i);

                // Skip channel if it either is not time to send any packets yet of the desired number of packets have already been sent
                if(packets_to_send_this_sendmmsg <= 0 || packets_sent_per_ch[ch_i] == num_packets) {
                    continue;
                }

                int num_packets_alread_sent = packets_sent_per_ch[ch_i];
                int num_packets_to_send = num_packets - num_packets_alread_sent;
                packets_to_send_this_sendmmsg = std::min(packets_to_send_this_sendmmsg, num_packets_to_send);

                int num_packets_sent_this_send;
                // Check if timestamp of next packet to send is in the past
                if((int64_t)packet_header_infos[num_packets_alread_sent].tsf >= get_time_now().to_ticks(_TICK_RATE)) {
                    // If not in the past, send packets
                    num_packets_sent_this_send = sendmmsg(send_sockets[ch_i], &ch_send_buffer_info_group[ch_i].msgs[num_packets_alread_sent], packets_to_send_this_sendmmsg, MSG_CONFIRM | MSG_DONTWAIT);
                } else {
                    // If it is in the past, skip sending it, but continue as if it was sent
                    num_packets_sent_this_send = 1;
                }

                if(num_packets_sent_this_send < 0) {
                    if(errno != EAGAIN && errno != EWOULDBLOCK) {
                        std::cerr << "sendmmsg on ch " << _channels[ch_i] << "failed with error: " << std::strerror(errno) << std::endl;
                    }
                } else {
                    packets_sent_per_ch[ch_i] += num_packets_sent_this_send;
                    // Update buffer level record
                    size_t nsamps_sent;
                    if(num_packets_to_send == num_packets_sent_this_send) {
                        // This send included the last packet (which may not be the maximum length)
                        nsamps_sent = ((num_packets_sent_this_send - 1) * _max_samples_per_packet) + samples_in_last_packet;
                        channels_serviced+=1;
                    } else {
                        // Every packet except the last one will have the maximum length
                        nsamps_sent = num_packets_sent_this_send * _max_samples_per_packet;
                    }
                    // Update counter for number of samples sent this send
                    samples_sent_per_ch[ch_i] += nsamps_sent;
                    // Update buffer level count
                    ch_send_buffer_info_group[ch_i].buffer_level_manager.update(nsamps_sent);
                }
            }
            //Checks for timeout
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC_COARSE, &current_time);
            int64_t current_time_ns = (current_time.tv_sec * 1000000000) + current_time.tv_nsec;
            if(current_time_ns > send_timeout_time_ns) {
                next_sequence_number = (next_sequence_number - num_packets + packets_sent_per_ch[0]);
                break;
            }
        }

        // Acts as though the channel with the most samples sent had samples sent
        size_t samples_sent = samples_sent_per_ch[0];
        for(size_t ch_i = 1; ch_i < _NUM_CHANNELS; ch_i++) {
            samples_sent = std::min(samples_sent_per_ch[ch_i], samples_sent);
        }

        // Updates the next timestamp to follow from the end of this send
        if(metadata_.has_time_spec) {
            next_send_time = metadata_.time_spec + time_spec_t::from_ticks(samples_sent, _sample_rate);
        } else {
            next_send_time = next_send_time + time_spec_t::from_ticks(samples_sent, _sample_rate);
        }

        return samples_sent;
    }

    void send_eob_packet(const uhd::tx_metadata_t &metadata, double timeout) {
        // Create vector of dummy samples, since the FPGA cannot handle 0 sample packets
        std::vector<std::vector<int8_t>> dummy_buffs(_NUM_CHANNELS, std::vector<int8_t>(_BYTES_PER_SAMPLE * _DEVICE_PACKET_NSAMP_MULTIPLE, 0));
        std::vector<const void *> dummy_buff_ptrs;
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            dummy_buff_ptrs.push_back(dummy_buffs[n].data());
        }

        // Clear cached sob flag, to handle edge case of where user sends 0 sample sob, followed by eob
        cached_sob = false;

        uhd::tx_metadata_t eob_md = metadata;
        // Clears start of burst flag
        eob_md.start_of_burst = false;
        // Sets the eof time so buffer tracking can account for time between sob and eob
        for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
            ch_send_buffer_info_i.buffer_level_manager.set_end_of_burst_time(next_send_time);
        }
        // Sends the eob packet
        send_multiple_packets(dummy_buff_ptrs, _DEVICE_PACKET_NSAMP_MULTIPLE, eob_md, timeout);
    }

    /*!
     * Prepares the converter
     * Called as part of the constructor, only in its own function to improve readability
     * \param cpu_format datatype of samples on the host system (only sc16 and fc32)
     * \param wire_format datatype of samples in the packets (only sc16)
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
                converter_id.output_format = wire_format + "_item32_le";
            } else {
                converter_id.output_format = wire_format + "_item32_be";
            }
            converter_id.num_inputs = 1;
            converter_id.input_format = cpu_format;
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

            _converter->set_scalar(wire_max / cpu_max);
        }
    }


    /*!
     * Resizes the intermediate buffers (if needed) and converts the samples
     * Returns the wrapper (set to the data in the constructor, done as a workaround because the UHD uses a vector wrapper)
    * \param samples The samples to convert
    * \param bytes Number of bytes to convert
    * \return Vector wrapper with the converted samples
    */
    UHD_INLINE uhd::tx_streamer::buffs_type* prepare_intermediate_buffers_and_convert(const uhd::tx_streamer::buffs_type& samples, const size_t num_samples) {
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            if(ch_send_buffer_info_group[n].intermediate_send_buffer.size() < num_samples * _BYTES_PER_SAMPLE) {
                // Resizes intermediate buffer
                ch_send_buffer_info_group[n].intermediate_send_buffer.resize(num_samples * _BYTES_PER_SAMPLE);
            }
            // Updates the pointer to the intermediate buffer
            _intermediate_send_buffer_pointers[n] = ch_send_buffer_info_group[n].intermediate_send_buffer.data();
        }

        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            // TODO figure out how the converter works to optimize this, it might be possible to do all at once
            // Converts the samples
            _converter->conv(samples[n], _intermediate_send_buffer_pointers[n], num_samples);
        }

        return &_intermediate_send_buffer_wrapper;
    }

    // Utility function to identify where randomg slowdowns are
    bool delay_check_start_time_set = false;
    uhd::time_spec_t last_delay_check_time;
    inline void check_for_long_delay(int flag_id) {
        if(delay_check_start_time_set) {
            delay_check_start_time_set = true;
            uhd::time_spec_t current_time = uhd::get_system_time();
            if(current_time.get_real_secs() - last_delay_check_time.get_real_secs() > 0.02) {
                printf("Long delay at %i\n", flag_id);
            }
            last_delay_check_time = current_time;
        } else {
            last_delay_check_time = uhd::get_system_time();
            delay_check_start_time_set = true;
        }
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

class send_packet_streamer_mmsg : public send_packet_handler_mmsg, public tx_streamer
{
public:
    send_packet_streamer_mmsg(const std::vector<size_t>& channels, ssize_t max_samples_per_packet, const int64_t device_buffer_size, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, ssize_t device_packet_nsamp_multiple, double tick_rate, const std::shared_ptr<bounded_buffer<async_metadata_t>> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian):
    sph::send_packet_handler_mmsg(channels, max_samples_per_packet, device_buffer_size, dst_ips, dst_ports, device_target_nsamps, device_packet_nsamp_multiple, tick_rate, async_msg_fifo, cpu_format, wire_format, wire_little_endian)
    {
    }

    size_t get_num_channels(void) const{
        return _NUM_CHANNELS;
    }

    size_t get_max_num_samps(void) const{
        return _max_samples_per_packet;
    }
    
    // Asynchronously receive messages notifying of overflow/underflows
    bool recv_async_msg(
        uhd::async_metadata_t &async_metadata, double timeout = 0.1
    ){
        boost::this_thread::disable_interruption di; //disable because the wait can throw
        return _async_msg_fifo->pop_with_timed_wait(async_metadata, timeout);
    }

    // Asynchronously send messages notifying of overflow/underflows
    bool push_async_msg( uhd::async_metadata_t &async_metadata ){
		return _async_msg_fifo->push_with_pop_on_full(async_metadata);
    }

    // Makes sure the correct enable_blocking_fc is used instead of the one from tx_streamer
    void enable_blocking_fc(uint64_t blocking_setpoint) {
        // TODO: change tx_streamer to use int64_t instead of uint64_t
        send_packet_handler_mmsg::enable_blocking_fc((int64_t)blocking_setpoint);
    }

    // Makes sure the correct enable_blocking_fc is used instead of the one from tx_streamer
    void disable_blocking_fc() {
        send_packet_handler_mmsg::disable_blocking_fc();
    }
};

} // namespace sph
} // namespace transport
} // namespace uhd

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_MMSG_HPP */
