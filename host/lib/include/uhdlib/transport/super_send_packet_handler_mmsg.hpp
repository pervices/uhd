//
// Copyright 2023-2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <uhd/convert.hpp>
#include <uhd/exception.hpp>
#include <uhd/stream.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <sys/socket.h>

#include <uhdlib/usrp/common/clock_sync.hpp>
#include <uhd/transport/buffer_tracker.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <uhd/utils/log.hpp>
#include <uhdlib/utils/performance_mode.hpp>
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
class send_packet_handler_mmsg {
// Declare constants first so they are initialized before constructor
private:
    // Cache line size
    // Assume it is 64, which is the case for virtually all AMD64 systems
    static constexpr uint_fast8_t CACHE_LINE_SIZE = 64;
    static constexpr size_t _bytes_per_sample = 4;
    // Size of the vrt header in bytes
    static constexpr size_t HEADER_SIZE = 12;
public:
    /*!
     * Make a new packet handler for send
     * \param buffer_size size of the buffer on the unit
     */
    send_packet_handler_mmsg(const std::vector<size_t>& channels, ssize_t max_samples_per_packet, const int64_t device_buffer_size, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, ssize_t device_packet_nsamp_multiple, double tick_rate, const std::shared_ptr<bounded_buffer<async_metadata_t>> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<uhd::usrp::clock_sync_shared_info> clock_sync_info_owner);

    ~send_packet_handler_mmsg(void);

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

        size_t previous_nsamps_in_cache = nsamps_in_cache;

        // FPGAs can sometimes only receive multiples of a set number of samples
        size_t actual_nsamps_to_send = (((nsamps_in_cache + nsamps_to_send) / _DEVICE_PACKET_NSAMP_MULTIPLE) * _DEVICE_PACKET_NSAMP_MULTIPLE);
        size_t desired_nsamps_to_cache = nsamps_to_send + nsamps_in_cache - actual_nsamps_to_send;

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

        // Lets the user know if the last burst dropped samples due to packet length multiple requirements
        if(dropped_nsamps_in_cache) {
            UHD_LOGGER_WARNING("SUPER_SEND_PACKET_HANDLER_MMSG") << "bursts must be a multiple of " << _DEVICE_PACKET_NSAMP_MULTIPLE << " samples. Dropping " << dropped_nsamps_in_cache << " samples to comply";
            dropped_nsamps_in_cache = 0;
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

        // Actual number of samples to cache
        size_t actual_nsamples_to_cache;
        // Number of samples from the cache that were sent
        size_t cached_samples_sent;
        // NUmber of samples from that cache that are to be kept for the next run that were present from the previous run
        size_t cached_samples_to_retain;

        // Copies samples that won't fit as a multiple of _DEVICE_PACKET_NSAMP_MULTIPLE to the cache
        if(actual_samples_sent == 0) {
            // No samples sent, therefore none should be added to the buffer
            actual_nsamples_to_cache = 0;
            // No samples sent, therefore no cached samples were consumed
            cached_samples_sent = 0;
            // No samples sent, therefore all samples in cache kept
            cached_samples_to_retain = previous_nsamps_in_cache;

        } else if(actual_samples_sent < previous_nsamps_in_cache) {
            actual_nsamples_to_cache = 0;
            cached_samples_sent = actual_samples_sent;
            cached_samples_to_retain = previous_nsamps_in_cache - cached_samples_sent;

            // If fewer samples were sent than were in the cache move the remaining samples to front of the cache
            for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
                memmove(ch_send_buffer_info_group[ch_i].sample_cache.data(), ch_send_buffer_info_group[ch_i].sample_cache.data() + actual_samples_sent, cached_samples_to_retain * _bytes_per_sample);
            }
        } else if(actual_samples_sent < actual_nsamps_to_send) {
            // If not the samples meant to actually be sent were sent, clear the cache and do not cache any samples
            // The sample cache is meant to handle the case where the send was successful, but the number of samples the user requested isn't a multiple of the required amount
            // Since in this case the send didn't send all the intended samples anyway, we don't need to bother with the cache
            actual_nsamples_to_cache = 0;
            cached_samples_sent = previous_nsamps_in_cache;
            cached_samples_to_retain = 0;
        }
        else if(actual_samples_sent == actual_nsamps_to_send) {
            actual_nsamples_to_cache = desired_nsamps_to_cache;
            cached_samples_sent = previous_nsamps_in_cache;
            cached_samples_to_retain = 0;
            // Since send was fully successful, copy samples that couldn't be sent this send due to limitations on packet sizing to the cache
            if(desired_nsamps_to_cache > 0) {
                for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
                    memcpy(ch_send_buffer_info_group[ch_i].sample_cache.data(), (uint8_t*)((*send_buffer)[ch_i]) + ((actual_samples_sent - cached_samples_sent) * _bytes_per_sample), actual_nsamples_to_cache * _bytes_per_sample);
                }
            }
        } else {
            fprintf(stderr, "ERROR, more samples sent than intended. This should be impossible, contact support\n");
            // Reaching here should be impossible, these values don't matter
            actual_nsamples_to_cache = 0;
            cached_samples_sent = 0;
            cached_samples_to_retain = 0;
        }

        // Update number of samples in cache count
        nsamps_in_cache = previous_nsamps_in_cache - cached_samples_sent + actual_nsamples_to_cache;

        // Return number of samples actually sent
        return actual_samples_sent - cached_samples_sent + actual_nsamples_to_cache;
    }

    void set_samp_rate(const double rate);
    void enable_blocking_fc(int64_t blocking_setpoint);
    void disable_blocking_fc();

protected:
    bool use_blocking_fc = false;
    ssize_t _max_samples_per_packet;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _NUM_CHANNELS;

    // Buffer containing asynchronous messages related to underflows/overflows
    const std::shared_ptr<bounded_buffer<async_metadata_t>> _async_msg_fifo;

    // Gets the the time on the unit when a packet sent now would arrive
    uhd::time_spec_t get_device_time();

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
    size_t nsamps_in_cache = 0;
    // Number of cached_samples dropped during the last EOB, resets after printing warning to user
    size_t dropped_nsamps_in_cache = 0;

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
        ch_send_buffer_info(const size_t size, const size_t vrt_header_size, const size_t cache_size, const int64_t device_target_nsamps, const double rate);

        // Resizes and clears the buffers to match packet_helper_buffer_size
        void resize_and_clear(size_t new_size);
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
    void expand_send_buffer_info(size_t new_size);

    // A smart pointer can have inconsistent access times but we need it to maintain ownership of the info to ensure it is not destructed
    // To solve this problem, we will put the smart pointer on it's own cache line (shown here as a pointer to a smart pointer) for ownership while using a raw pointer for actual operations

    // Pointer to a smart pointer with ownership to where the info required to calculate the device time is stored
    static constexpr size_t clock_sync_shared_info_size = (size_t) ceil(sizeof(std::shared_ptr<uhd::usrp::clock_sync_shared_info>) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
    std::shared_ptr<uhd::usrp::clock_sync_shared_info>* _clock_sync_info_owner;
    // Raw pointer to the above
    uhd::usrp::clock_sync_shared_info* _clock_sync_info;

    // Gets the number of samples that can be sent now (can be less than 0)
    int check_fc_npackets(const size_t ch_i);

    UHD_INLINE size_t send_multiple_packets(
        const uhd::tx_streamer::buffs_type &sample_buffs,
        const size_t nsamps_to_send,
        const uhd::tx_metadata_t &metadata_,
        const double timeout,
        // TODO: split sending the eob packets into their own function to be less spaghetti
        // Call this function for sending eob packet (which only contains dummy samples)
        const bool is_eob_send = false
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
            packet_header_infos[n].has_tsf = true; // Always include a fractional timestamp (in ticks of _TICK_RATE)
            if(metadata_.has_time_spec) {
                // Sets the timestamp based on what's specified by the user
                packet_header_infos[n].tsf = (metadata_.time_spec + time_spec_t::from_ticks(n * _max_samples_per_packet - nsamps_in_cache, _sample_rate)).to_ticks(_TICK_RATE);
            } else {
                // Sets the timestamp to follow from the previous send
                packet_header_infos[n].tsf = (next_send_time + time_spec_t::from_ticks(n * _max_samples_per_packet - nsamps_in_cache, _sample_rate)).to_ticks(_TICK_RATE);
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

        // Figures out where in the user provided buffer each packet should get data from
        for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
            // Get data from the start of the buffer for the first packet
            // Impact of the cached samples is handled later by making adding cache to the iovec before the buffer
            ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[0] = (uint8_t*)(sample_buffs[ch_i]);
            // For every other packet get data from (buffer start) + (packet_number * packet data length), the subtract samples
            for(int n = 1; n < num_packets; n++) {
                ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[n] = (uint8_t*)(sample_buffs[ch_i]) + (n * _MAX_SAMPLE_BYTES_PER_PACKET) - (nsamps_in_cache * _bytes_per_sample);
            }
        }

        for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
            // Sets up iovecs for the first packets
            // VRT Header
            ch_send_buffer_info_group[ch_i].iovecs[0].iov_base = ch_send_buffer_info_group[ch_i].vrt_headers[0].data();
            ch_send_buffer_info_group[ch_i].iovecs[0].iov_len = HEADER_SIZE;
            // Cached samples
            ch_send_buffer_info_group[ch_i].iovecs[1].iov_base = ch_send_buffer_info_group[ch_i].sample_cache.data();
            ch_send_buffer_info_group[ch_i].iovecs[1].iov_len = nsamps_in_cache * _bytes_per_sample;
            // Samples
            // iovecs.iov_base is const for all practical purposes, const_cast is used to allow it to use data from the buffer which is const
            ch_send_buffer_info_group[ch_i].iovecs[2].iov_base = const_cast<void*>(ch_send_buffer_info_group[ch_i].sample_data_start_for_packet[0]);
            if(num_packets > 1) {
                ch_send_buffer_info_group[ch_i].iovecs[2].iov_len = _MAX_SAMPLE_BYTES_PER_PACKET - (nsamps_in_cache * _bytes_per_sample);
            } else {
                ch_send_buffer_info_group[ch_i].iovecs[2].iov_len = (samples_in_last_packet - nsamps_in_cache) * _bytes_per_sample;
            }

            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_iov = &ch_send_buffer_info_group[ch_i].iovecs[0];
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_iovlen = 3;

            // Setting optional data to none
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_name = NULL;
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_namelen = 0;
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_control = NULL;
            ch_send_buffer_info_group[ch_i].msgs[0].msg_hdr.msg_controllen = 0;

            // Sets up iovecs and msg for packets 1 to n -1
            for(int n = 1; n < num_packets - 1; n++) {
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

                // Always send 1 packet if eob (only contains dummy samples so they can be sent regardless of buffer level)
                if(is_eob_send) {
                    packets_to_send_this_sendmmsg = 1;
                }

                // Skip channel if it either is not time to send any packets yet of the desired number of packets have already been sent
                if((packets_to_send_this_sendmmsg <= 0 || packets_sent_per_ch[ch_i] == num_packets)) {
                    continue;
                }

                int num_packets_alread_sent = packets_sent_per_ch[ch_i];
                int num_packets_to_send = num_packets - num_packets_alread_sent;
                packets_to_send_this_sendmmsg = std::min(packets_to_send_this_sendmmsg, num_packets_to_send);

                int num_packets_sent_this_send;
                // Check if timestamp of next packet to send is in the past
                if((int64_t)packet_header_infos[num_packets_alread_sent].tsf >= get_device_time().to_ticks(_TICK_RATE) || packet_header_infos[num_packets_alread_sent].sob || packet_header_infos[num_packets_alread_sent].eob || !packet_header_infos[num_packets_alread_sent].has_tsf || use_blocking_fc) {
                    // If not in the past, send packets
                    // Ignore the check for in the past if using use_blocking_fc since the buffer won't overflow because of latency in get buffer level and trigger streaming which uses it will often have timestamps in the past
                    // TODO: remove !use_blocking_fc workaround once the FPGA can handle packets without timestamps
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
                    if(!is_eob_send) {
                        ch_send_buffer_info_group[ch_i].buffer_level_manager.update(nsamps_sent);
                    }
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

        if(metadata_.start_of_burst && is_eob_send) {
            printf("ERROR sob and eob at the same time\n");
        }

        // If no packets sent remove the sob time for this send from the list of start of bursts
        if(metadata_.start_of_burst && !samples_sent) {
            for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
                ch_send_buffer_info_i.buffer_level_manager.pop_back_start_of_burst_time();
            }
            // Add sob to cache to be used on the next attempt
            cached_sob = true;
            sob_time_cache = metadata_.time_spec;
        }

        // NOTE: samples_sent here will be non 0 because of the dummy samples
        if(is_eob_send && !samples_sent) {
            for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
                ch_send_buffer_info_i.buffer_level_manager.pop_back_end_of_burst_time();
            }
        }

        if(is_eob_send) {
            // The samples in an eob are unused, they only exist because the FPGA can't handle 0 samples per packet
            return 0;
        } else {
            return samples_sent;
        }
    }

    void send_eob_packet(const uhd::tx_metadata_t &metadata, double timeout);

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
                // item32 results in entire 32 bit words being converted to little endian
                // i.e. _item32_le means Q LSB, Q MSB, I LSB, I MSB
                // we want _item32_le means I LSB, I MSB, Q LSB, Q MSB
                // We want 16 bit halves to be little endian, which chdr provides
                // NOTE: chdr is a legacy data format for old Ettus stuff
                // If it ever gets removes create an identical implementation named _item_16_le
                converter_id.output_format = wire_format + "_chdr";
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

    int get_mtu(int socket_fd, std::string ip);
};

class send_packet_streamer_mmsg : public send_packet_handler_mmsg, public tx_streamer {
public:
    send_packet_streamer_mmsg(const std::vector<size_t>& channels, ssize_t max_samples_per_packet, const int64_t device_buffer_size, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, ssize_t device_packet_nsamp_multiple, double tick_rate, const std::shared_ptr<bounded_buffer<async_metadata_t>> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<uhd::usrp::clock_sync_shared_info> clock_sync_info);

    size_t get_num_channels(void) const override {
        return _NUM_CHANNELS;
    }

    size_t get_max_num_samps(void) const override {
        return _max_samples_per_packet;
    }

    // Asynchronously receive messages notifying of overflow/underflows
    bool recv_async_msg(uhd::async_metadata_t &async_metadata, double timeout = 0.1) override;

    // Asynchronously send messages notifying of overflow/underflows
    bool push_async_msg( uhd::async_metadata_t &async_metadata);

    // Makes sure the correct enable_blocking_fc is used instead of the one from tx_streamer
    void enable_blocking_fc(uint64_t blocking_setpoint) override;

    // Makes sure the correct enable_blocking_fc is used instead of the one from tx_streamer
    void disable_blocking_fc() override;

    void post_output_action(const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t) override;
};
} // namespace sph
} // namespace transport
} // namespace uhd
