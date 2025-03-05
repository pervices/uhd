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
#include <uhd/transport/async_recv_manager.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhdlib/utils/network_config.hpp>
#include <uhdlib/utils/performance_mode.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include <uhdlib/utils/system_time.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>

#include <error.h>
#include <uhd/exception.hpp>
#include <sys/ioctl.h>
#include <net/if.h>

#include <sys/mman.h>
#include <fcntl.h>

#include <immintrin.h>

// Manages sending streaming commands
#include <uhdlib/usrp/common/stream_cmd_issuer.hpp>

#define MIN_MTU 9000

namespace uhd { namespace transport { namespace sph {

    // Socket priority for rx sockets
    // Experimentally verified that this needs to be at least 6
    const int RX_SO_PRIORITY = 6;

/***********************************************************************
 * Super receive packet handler
 *
 * A receive packet handler represents a group of channels.
 * The channel group shares a common sample rate.
 * All channels are received in unison in recv().
 **********************************************************************/
class recv_packet_handler_mmsg
{
public:

    /*!
     * Make a new packet handler for receive
     * \param recv_sockets sockets to receive data packets on. Must be bound to the desired IP/port already. This close will handle all other setup and closing the socket
     * \param dst_ip destination IP address of the packets to be received
     * \param max_sample_bytes_per_packet the number of transport channels
     * \param header_size the number of transport channels
     * \param cpu_format datatype of samples on the host system
     * \param wire_format datatype of samples in the packets
     * \param wire_little_endian true if the device is configured to send little endian data. If cpu_format == wire_format and wire_little_endian no converter is required, boosting performance
     * \param device_total_rx_channels Total number of rx channels on the device, used to determine how many threads to use for receiving
     */
    recv_packet_handler_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuers)
    :
    _NUM_CHANNELS(recv_sockets.size()),
    _MAX_SAMPLE_BYTES_PER_PACKET(max_sample_bytes_per_packet),
    _HEADER_SIZE(header_size),
    _TRAILER_SIZE(trailer_size),
    _stream_cmd_issuers(cmd_issuers),
    _recv_sockets(recv_sockets),
    _previous_buffer_writes_count(_NUM_CHANNELS, 0),
    _num_cached_samples(_NUM_CHANNELS, 0),
    _sample_cache(_NUM_CHANNELS, std::vector<uint8_t>(_MAX_SAMPLE_BYTES_PER_PACKET, 0))
    {
        UHD_LOG_INFO("UHD", "A1");
        if (wire_format=="sc16") {
            _BYTES_PER_SAMPLE = 4;
        } else if (wire_format=="sc12") {
            _BYTES_PER_SAMPLE = 3;
        } else {
            throw uhd::runtime_error( "Unsupported wire format:" + wire_format);
        }

        // Performs a check (and if applicable warning message) for potential source of performance issues
        check_high_order_alloc_disable();
        UHD_LOG_INFO("UHD", "A10");

        // Checks if preemption is disabled/voluntary and warns user if it is not
        check_pre_empt();
        UHD_LOG_INFO("UHD", "A20");

        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            check_rx_ring_buffer_size(dst_ip[n]);
        }
        UHD_LOG_INFO("UHD", "A30");

        // Performs socket setup
        // Sockets passed to this constructor must already be bound
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {

            // Set socket to non-blocking
            // For unknown reasons having this set helps performance, even though it shouldn't make a difference if recvmmsg is called with MSG_DONTWAIT
            int flags = fcntl(_recv_sockets[n],F_GETFL);
            flags = (flags | O_NONBLOCK);
            if(fcntl(_recv_sockets[n], F_SETFL, flags) < 0)
            {
                throw uhd::runtime_error( "Failed to set socket to non-blocking. Performance may be affected" );
            }


            // Sets the recv buffer size
            setsockopt(_recv_sockets[n], SOL_SOCKET, SO_RCVBUF, &DEFAULT_RECV_BUFFER_SIZE, sizeof(DEFAULT_RECV_BUFFER_SIZE));

            // Checks the recv buffer size
            // Actual recv buffer size, the Kernel will set the real size to be double the requested
            int actual_recv_buffer_size = 0;
            socklen_t opt_len = sizeof(actual_recv_buffer_size);
            getsockopt(_recv_sockets[n], SOL_SOCKET, SO_RCVBUF, &actual_recv_buffer_size, &opt_len);

            // NOTE: The kernel will set the actual size to be double the requested. So the expected amount is double the requested
            if(actual_recv_buffer_size < 2*DEFAULT_RECV_BUFFER_SIZE) {
                fprintf(stderr, "Unable to set recv buffer size. Performance may be affected\nTarget size %i\nActual size %i\nPlease run \"sudo sysctl -w net.core.rmem_max=%i\"\n", DEFAULT_RECV_BUFFER_SIZE, actual_recv_buffer_size/2, DEFAULT_RECV_BUFFER_SIZE);
                throw uhd::system_error("Unable to set recv socket size");
            }

            // Verify the interface can handle large packets
            int mtu = get_mtu(_recv_sockets[n], dst_ip[n].c_str());
            if(mtu < MIN_MTU) {
                fprintf(stderr, "MTU of interface associated with %s is to small. %i required, current value is %i", dst_ip[n].c_str(), MIN_MTU, mtu);
                throw uhd::system_error("MTU size to small");
            }

            // Set socket priority
            int set_priority_ret = setsockopt(_recv_sockets[n], SOL_SOCKET, SO_PRIORITY, &RX_SO_PRIORITY, sizeof(RX_SO_PRIORITY));
            if(set_priority_ret) {
                fprintf(stderr, "Attempting to set rx socket priority failed with error code: %s", strerror(errno));
            }

            // Sets the duration to busy poll/read (in us) after a recv call
            // Documentation says this only applies to blocking requests, experimentally this still helps with recvmmsg MSG_DONTWAIT
            const int busy_poll_time = 1000;
            int set_busy_poll_ret = setsockopt(_recv_sockets[n], SOL_SOCKET, SO_BUSY_POLL, &busy_poll_time, sizeof(set_busy_poll_ret));
            if(set_priority_ret) {
                fprintf(stderr, "Attempting to set rx busy read priority failed with error code: %s", strerror(errno));
            }
        }
        UHD_LOG_INFO("UHD", "A100");

        setup_converter(cpu_format, wire_format, wire_little_endian);
        UHD_LOG_INFO("UHD", "A110");

        // Check if the governor is set to performance mode, warns the user if it is not
        check_if_only_using_governor();
        UHD_LOG_INFO("UHD", "A120");

        // Create manager for receive threads and access to buffer recv data
        size_t page_size = getpagesize();
        // Create manager for threads that receive data to buffers using placement new to avoid false sharing
        // Put the manager on it's own memory page. Theoretically placing it's own cache line is sufficient, but experimentally padding to full pages is required to avoid interactions with other threads
        size_t recv_manager_size = (size_t) ceil(sizeof(async_recv_manager) / (double)page_size) * page_size;
        recv_manager = (async_recv_manager*) aligned_alloc(page_size, recv_manager_size);

        // Prevent the class from being moved to a huge page, causes latency spikes
        if( madvise(recv_manager, recv_manager_size, MADV_NOHUGEPAGE) < 0 ) {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER_MMSG", "Error while calling madvise MADV_NOHUGEPAGE for interal buffer. Error: " + std::string(strerror(errno)));
        }
        UHD_LOG_INFO("UHD", "A400");

        new (recv_manager) async_recv_manager(device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet, device_total_rx_channels);
        UHD_LOG_INFO("UHD", "A500");
    }

    ~recv_packet_handler_mmsg(void)
    {
        // recv_manager must be deleted before closing sockets
        // Destructor must be manually called when using placement new
        recv_manager->~async_recv_manager();
        free(recv_manager);
        for(size_t n = 0; n < _recv_sockets.size(); n++) {
            int r = close(_recv_sockets[n]);
            if(r) {
                fprintf(stderr, "close failed on data receive socket with: %s\nThe program may not have closed cleanly\n", strerror(errno));
            }
        }

        if(_overflow_occured && _suboptimal_spb) {
            UHD_LOGGER_WARNING("RECV_PACKET_HANDLER_MMSG") << "An overflow occured during a run where a subopitmal number of samples were requested from recv. To reduce the chance of an overflow in the future ensure nsamps_per_buff is multiple of " << _MAX_SAMPLE_BYTES_PER_PACKET / _BYTES_PER_SAMPLE;
        }
    }

    UHD_INLINE size_t recv(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)
    {
        // A suboptimal number of samples per call is anything that is not a multiple of the packet length
        // Sets a flag to provide advice to the user in the event of an overflow
        _suboptimal_spb |= ((nsamps_per_buff * _BYTES_PER_SAMPLE) % _MAX_SAMPLE_BYTES_PER_PACKET);

        // Clears the metadata struct
        // Reponsible for setting error_code to none, has_time_spec to false and eob to false
        metadata.reset();

        // Vector of variables used to improve cache locality
        struct packet_info{
            // Buffer to store copy of Vita header of a packet
            // A copy is created instead of using a pointer to avoid issues with the header being overwriten while being unpacked
            std::vector<uint8_t> packet_hdr;
            // Pointer to the start of samples in the packet
            uint8_t* packet_samples;
            // Length of the packet received (usually does not include the trailers)
            uint_fast32_t packet_length;

            packet_info(const size_t header_size) :
            packet_hdr(std::vector<uint8_t>(header_size, 0)),
            packet_samples(nullptr),
            packet_length(0)
            {

            };
        };

        std::vector<packet_info> packet_infos(_NUM_CHANNELS, packet_info(_HEADER_SIZE));

        // Place to store metadata of Vita packets
        // Not included in packet_info becuase packet_info's memory are accessed togther within a channel, vita_md is better to have it be adjacent for different channels
        std::vector<vrt::if_packet_info_t> vita_md(_NUM_CHANNELS);

        // The channel with the latest packet
        uint64_t latest_packet = 0;

        bool oflow_message_printed = false;

        size_t samples_received = 0;

        // Withdraw samples from the cache
        for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
            if(_num_cached_samples[ch]) {
                size_t cached_samples_to_use = std::min(_num_cached_samples[ch], nsamps_per_buff);
                // Copies samples from the cache to the user requested buffer
                convert_samples(buffs[ch], _sample_cache[ch].data(), cached_samples_to_use);

                // Move extra cached samples to the start of the buffer
                _num_cached_samples[ch] -= cached_samples_to_use;
                memmove(_sample_cache[ch].data(), _sample_cache[ch].data() + (cached_samples_to_use * _BYTES_PER_SAMPLE), _num_cached_samples[ch] * _BYTES_PER_SAMPLE);

                // Record that samples have been received, setting this for each is fine since they should be equal at this time
                samples_received = cached_samples_to_use;

                // Generate a timestamp to simulate what the timestamp of the cached samples based off of the timestamp in the last packet (other the previous synthetic timestamp
                metadata.has_time_spec = true;
                // Shift cached timestamp by number of samples withdrawn from the cache
                tsf_cache+=cached_samples_to_use;
                metadata.time_spec = time_spec_t::from_ticks(tsf_cache, _sample_rate);

                // Set EOB flag if there is a cached eob flag and there are no extra samples in the buffer
                metadata.end_of_burst = eob_cached && !_num_cached_samples[ch];
            }
        }

        // Clear eob from cache since it has been applied
        eob_cached = false;

        time_spec_t recv_start_time = get_system_time();

        // Limit for how many time realignment will be attempted before giving up
        const size_t max_realignment_attempts = 100;
        size_t realignment_attempts = 0;

        // Main receive loop
        while(samples_received < nsamps_per_buff) [[likely]] {
            bool overflow_detected = false;
            bool realignment_required = false;

            // Stores buffer_write_count from when the packet was obtained
            std::vector<int_fast64_t> initial_buffer_writes_count(_NUM_CHANNELS);

            size_t ch = 0;
            // While not all channels have been obtained and timeout has not been reached
            while(ch < _NUM_CHANNELS && recv_start_time + timeout > get_system_time()) {
                initial_buffer_writes_count[ch] = recv_manager->get_buffer_write_count(ch);
                // if (buffer_write_count has increased since the last recv || the next packet is not the first packet of the buffer) && buffer_write_count is even
                if((initial_buffer_writes_count[ch] > _previous_buffer_writes_count[ch] || !recv_manager->is_first_packet_of_buffer(ch)) && !(initial_buffer_writes_count[ch] & 1)) {
                    // Move onto the next channel since this one is ready
                    ch++;
                } else {
                    // Do nothing
                    // _mm_pause (which marks this as a polling loop) might help, but it appears to make performance worse
                }
            }

            _mm_mfence();

            // Check if timeout occured
            // TODO: refactor to reduce branching
            if(ch < _NUM_CHANNELS) [[unlikely]] {
                if(samples_received) {
                    // Does not set timeout error when any samples were received
                    return samples_received;
                } else {
                    // Set timeout if no other error occured and no samples received and no other error code present
                    if(metadata.error_code == rx_metadata_t::ERROR_CODE_NONE) {
                        metadata.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
                    }
                    return 0;
                }
            }

            // Flag that indicates if the packet was overwritten mid read
            bool mid_header_read_header_overwrite = false;

            std::atomic_thread_fence(std::memory_order_consume);

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Gets info for this packet
                memcpy(packet_infos[ch].packet_hdr.data(), recv_manager->get_next_packet_vita_header(ch), _HEADER_SIZE);

                packet_infos[ch].packet_samples = recv_manager->get_next_packet_samples(ch);
                packet_infos[ch].packet_length = recv_manager->get_next_packet_length(ch);
                // Maximum size the packet length field in Vita packet could be ( + _TRAILER_SIZE since we drop the trailer)
                vita_md[ch].num_packet_words32 = (packet_infos[ch].packet_length + _TRAILER_SIZE) / sizeof(uint32_t);

                // Check for incorrect packet
                if(packet_infos[ch].packet_length < _HEADER_SIZE) [[unlikely]] {
                    throw std::runtime_error("Received sample packet smaller than header size");
                }

                int_fast64_t post_header_copied_buffer_write_count = recv_manager->get_buffer_write_count(ch);
                std::atomic_thread_fence(std::memory_order_consume);
                // If buffer_write_count changed while getting header info
                if(post_header_copied_buffer_write_count != initial_buffer_writes_count[ch]) {
                    mid_header_read_header_overwrite = true;
                    // Change the location to get the next packet to the start of the buffer, since this buffer is newly modified
                    // Droped everything in all buffers between the packet originally meant to be read the start of this buffer, which also helps catch up after overflows
                    recv_manager->reset_buffer_read_head(ch);
                }
            }

            // Restart loop since buffers may have been modified when headers were being processed
            if(mid_header_read_header_overwrite) {
                continue;
            }

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Extract Vita metadata
                if_hdr_unpack((uint32_t*) packet_infos[ch].packet_hdr.data(), vita_md[ch]);

                // TODO: enable this once eob flag is properly implement in packets && cache it in the eve
                // Currently Crimson will always have eob and Cyan will never have
                // metadata.end_of_burst |= vita_md[ch].eob

                // Finds and records the sequence number and timestamp of whichever channel's next packet is last
                // Used for realignment, normally they will be the same for all packets
                if(latest_packet < vita_md[ch].tsf) {
                    latest_packet = vita_md[ch].tsf;
                }

                // Set the flag for realignment required if there is a mismatch in timestamps between packets
                realignment_required = vita_md[ch].tsf != vita_md[0].tsf || realignment_required;

                // Detect and warn user of overflow error
                if(vita_md[ch].packet_count != (SEQUENCE_NUMBER_MASK & (previous_sequence_number + 1))  && vita_md[ch].tsf != 0) [[unlikely]] {
                    metadata.error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
                    _overflow_occured = true;
                    overflow_detected = true;
                }
            }

            if(overflow_detected && !oflow_message_printed) [[unlikely]] {
                print_overflow_message();

                // Flag to prevent printing the message once per recv call
                oflow_message_printed = true;
            }

            if(realignment_required) [[unlikely]] {
                if(realignment_attempts >= max_realignment_attempts) {
                    if(!align_message_printed) {
                        UHD_LOGGER_ERROR("STREAMER") << "Failed to re-align channels after overflow";
                    }
                    align_message_printed = true;
                    // Override overflow error with alignment error to indicate that UHD was unable to fully recover from the overflow
                    metadata.error_code = rx_metadata_t::ERROR_CODE_ALIGNMENT;
                } else {
                    for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                        if(vita_md[ch].tsf != latest_packet) {
                            // Drop this packet to allow the channel to catch up
                            recv_manager->advance_packet(ch);
                        }
                    }
                    realignment_attempts++;
                    // Start a new iteration of the check to see if the packets are aligned now
                    continue;
                }
            } else {
                // Reset realignment attempt counter
                realignment_attempts = 0;
            }

            size_t packet_sample_bytes = vita_md[0].num_payload_bytes;
            size_t samples_to_consume = 0;
            bool mid_header_read_data_overwrite = false;
            // Number of samples in the packet that don't fit in the user's buffer and need to be cached until the next recv
            std::vector<size_t> samples_to_cache(_NUM_CHANNELS, 0);

            // Copies sample data from the provider buffer to the user buffer
            // NOTE: do not update variables stored between runs in this loop, since the results will need to be discarded if data was overwritten
            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Error checking for if there is a mismatch in packet lengths
                if(packet_sample_bytes != vita_md[ch].num_payload_bytes) [[unlikely]] {
                    packet_sample_bytes = std::min(packet_sample_bytes, vita_md[ch].num_payload_bytes);
                    UHD_LOGGER_ERROR("STREAMER") << "Mismatch in sample count between packets";

                    // Something is wrong with the packets if there is a mismatch in size and no other error has occured
                    if(metadata.error_code == rx_metadata_t::ERROR_CODE_NONE) {
                        metadata.error_code = rx_metadata_t::ERROR_CODE_BAD_PACKET;
                    }
                }

                size_t samples_in_packet = vita_md[ch].num_payload_bytes / _BYTES_PER_SAMPLE;
                // Number of samples in the packet that fit in the user's buffer
                samples_to_consume = std::min(samples_in_packet, nsamps_per_buff - samples_received);
                samples_to_cache[ch] = samples_in_packet - samples_to_consume;
                // Copies data from provider buffer to the user's buffer,
                convert_samples(buffs[ch], packet_infos[ch].packet_samples, samples_to_consume);

                // Not actually unlikely, flagged as unlikely since it is false when all samples per recv call is most optimal
                if(samples_to_cache[ch]) [[unlikely]] {
                    // Copy extra samples from the packet to the cache
                    memcpy(_sample_cache[ch].data(), packet_infos[ch].packet_samples + (samples_to_consume * _BYTES_PER_SAMPLE), samples_to_cache[ch] * _BYTES_PER_SAMPLE);
                }

                int_fast64_t post_data_copied_buffer_write_count = recv_manager->get_buffer_write_count(ch);
                std::atomic_thread_fence(std::memory_order_consume);
                // If buffer_write_count changed while copying data
                if(post_data_copied_buffer_write_count != initial_buffer_writes_count[ch]) {
                    mid_header_read_data_overwrite = true;
                    // Change the location to get the next packet to the start of the buffer, since this buffer is newly modified
                    // Droped everything in all buffers between the packet originally meant to be read the start of this buffer, which also helps catch up after overflows
                    recv_manager->reset_buffer_read_head(ch);
                }
            }

            // Restart recv loop since the packets was overwritten while copying data from the provider buffer
            if(mid_header_read_data_overwrite) {
                continue;
            }

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Update number of cached samples
                _num_cached_samples[ch] = samples_to_cache[ch];
                if(samples_to_cache[ch]) {
                    // Remove eob flag and record it in the cache to apply to cached samples
                    eob_cached = metadata.end_of_burst;
                    metadata.end_of_burst = false;
                }

                // Moves to the next packet
                recv_manager->advance_packet(ch);
            }

            // Set the timepec to that of the first packet received if not already set from the cache
            // They should be equal to the only the first channel's is used
            if(!metadata.has_time_spec) {
                metadata.has_time_spec = true;
                // No need to check for has_tsf since our FPGAs always include tsf
                metadata.time_spec = time_spec_t::from_ticks(vita_md[0].tsf, _sample_rate);
            }

            // Update the sequence number
            previous_sequence_number = vita_md[0].packet_count;

            // Update tsf cache to most recent (this packet)
            tsf_cache = vita_md[0].tsf;

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                _previous_buffer_writes_count[ch] = initial_buffer_writes_count[ch];
            }

            // Record how many samples have been copied to the buffer, will be the same for all channels
            samples_received += samples_to_consume;

            // Exit loop if user only wants one packet
            // Not actually unlikely, but performance matters more when false
            if(one_packet) [[unlikely]] {
                break;
            }

        }

        return samples_received;
    }

    // Set the rate of samples per second
    void set_sample_rate(const double rate)
    {
        _sample_rate = rate;
    }

    void issue_stream_cmd(const stream_cmd_t& stream_cmd)
    {
        if (_NUM_CHANNELS > 1 and stream_cmd.stream_now
            and stream_cmd.stream_mode != stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS) {
            throw uhd::runtime_error(
                "Invalid recv stream command - stream now on multiple channels in a "
                "single streamer will fail to time align.");
        }

        for (size_t chan_i = 0; chan_i < _NUM_CHANNELS; chan_i++) {
            _stream_cmd_issuers[chan_i].issue_stream_command(stream_cmd);
        }
    }

protected:
    size_t _NUM_CHANNELS;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _BYTES_PER_SAMPLE;

    virtual void if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

private:

    // Desired recv buffer size
    static constexpr int DEFAULT_RECV_BUFFER_SIZE = 500000000;
    // Maximum number of packets to recv (should be able to fit in the half the real buffer)
    const size_t _HEADER_SIZE;
    // Trailer is not needed for anything so receive will discard it
    const size_t _TRAILER_SIZE;

    // Sends stream commands the device, manages the corresponding sockets
    std::vector<uhd::usrp::stream_cmd_issuer> _stream_cmd_issuers;

    std::vector<int> _recv_sockets;
    // Maximum sequence number
    static constexpr size_t SEQUENCE_NUMBER_MASK = 0xf;

    size_t previous_sequence_number = 0;

    // Value of buffer_write_count during the last recv call
    std::vector<int_fast64_t> _previous_buffer_writes_count;

    // Converts samples between wire and cpu formats
    uhd::convert::converter::sptr _converter;

    // Sample rate in samples per second
    double _sample_rate = 0;

    async_recv_manager* recv_manager;

    // Cache of samples from packets that are leftover and stored until the next packet
    // _sample_cache in wire format
    std::vector<size_t> _num_cached_samples;
    std::vector<std::vector<uint8_t>> _sample_cache;

    // Most recent packet's timestamp, used when generating a synthetic timestamp for cached samples
    // The tick rate for tsf is in samples
    uint64_t tsf_cache = 0;

    // EOB was detected and should be applied to the cached samples
    uint_fast8_t eob_cached = false;

    // Flag for if an overflow occured, used to decide if advice should be printed to the user
    uint_fast8_t _overflow_occured = false;
    // An spb was requested that was not a multiple of packet length
    uint_fast8_t _suboptimal_spb = false;

    // Only print these messages once per recv call
    bool align_message_printed = false;

    /*!
     * Prepares the converter.
     * The converter is still useful when converting data to the same time since it uses SIMD function as a faster memcpy
     * Called as part of the constructor, only in its own function to improve readability
     * \param cpu_format datatype of samples on the host system (only sc16 and fc32)
     * \param wire_format datatype of samples in the packets (only sc16 or sc12)
     * \param wire_little_endian data format in packets is little endian
     */
    UHD_INLINE void setup_converter(const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian) {

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

    /**
     * Copies samples from src to dst using SIMD, converts between data formats if applicable.
     * @param dst the destination to copy to. Must be at least of dst sample size * num_samples
     * @param src the source to copy to. Must be at least src sample size * num_samples
     * @param num_samples the number of samples to copy
     */
    UHD_INLINE void convert_samples(void* dst, void* src, size_t num_samples) {
        // TODO: investigate if this be optimized to reduce branching
        _converter->conv(src, dst, num_samples);
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
                        if(errno == ENOSPC) {
                            throw uhd::value_error("Address buffer to small");
                        } else {
                            throw uhd::runtime_error("Unexpected error in inet_ntop: " + std::string(strerror(errno)));
                        }
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

    void print_overflow_message() {
        // Warn user that an overflow occured
        UHD_LOG_FASTPATH("D");
    }

    /*
     * Check if higher order allocation is enabled.
     * Informs the user if it is disabled. Older UHD versions could benefit on some CPUs but it worsens performance on others.
     * Let's the user know they can re-enable it since they likely only disabled it due to previous warnings.
     */
    void check_high_order_alloc_disable() {
        std::string path = "/proc/sys/net/core/high_order_alloc_disable";

        FILE *file;

        file = fopen(path.c_str(), "r");

        if(file == NULL) {
            // Unable to open file, skip check
            // The likely cause is that this is running on a very old kernel (such as RedHat 8's 4.18.0-553.33.1.el8_10.x86_64)
            // Therefore don't warn the user since there is no action to take
            return;
        }

        int value = fgetc(file);

        if(value == -1) {
            UHD_LOG_INFO("RECV_PACKET_HANDLER", "Read " + path + " failed with error code:" + std::string(strerror(errno)) + ". Unable to check if high order allocation enabled.\nUHD used to benefit from having higher order allocation disabled but that is no longer the case. You may restore default higher order allocation setting (disabled) if you changed it at the requested of UHD.\n");
        } else if(value != '0') {
            UHD_LOG_INFO("RECV_PACKET_HANDLER", "High order allocation disabled. UHD no longer benefits from this being disabled. You may renable it. Run \"sudo sysctl -w net.core.high_order_alloc_disable=0\" to enable it.");
        }
    }

    /*
     * Checks if preemption is set to full.
     * Preemption can cause occasional brief latency spikes that cause overflows at high sample rates
     */
    void check_pre_empt() {
        std::string path = "/sys/kernel/debug/sched/preempt";

        FILE *file;

        file = fopen(path.c_str(), "r");

        if(file == NULL) {
            if(errno == EACCES) {
                UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Insufficient permission to check preemption setting. Check " + path + " to manually check it's current setting. It must be set to none or voluntary for optimal performance.\nTo allow this check to work successfully either run this program with sudo or give this user read access to " + path);
                return;
            } else if (errno == ENOENT) {
                // Do nothing
                // If the file does not exist assume that the kernel is to old to have this feature and therefore skip the warning message
                return;
            } else {
                UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Preemption check failed with error code: " + std::string(strerror(errno)) + "\nCheck " + path + " to manually check it's current setting. It must be set to none or voluntary for optimal performance.");
                return;
            }
        }

        char buffer[25];
        char* r = fgets(buffer, 25, file);
        std::string value;
        if(r != nullptr) {
            value = std::string(buffer);
        } else {
            value = "";
        }

        if(value.find("(none)") == std::string::npos && value.find("(voluntary)") == std::string::npos) {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Preemption is currently enabled, this may cause infrequent performance issues. Run \"echo voluntary > " + path + "\" as root. It must be run as root user, sudo will not work.");
        }
    }

    /**
     * Checks if the rx ring buffer for the device with the specified ip is set to it's maximum and prints a warning ot the user if it isn't.
     */
    void check_rx_ring_buffer_size(std::string ip) {
        try {
            std::string dev = get_dev_from_ipv4(ip);

            uint32_t current_size = get_rx_ring_buffer_size(dev);
            uint32_t max_size = get_rx_ring_buffer_max_size(dev);

            if(current_size < max_size) {
                UHD_LOG_WARNING("RECV_PACKET_HANDLER", "The RX ring buffer size (" + std::to_string(current_size) + ") is not set to the maximum (" + std::to_string(max_size) + ") for interface " + dev + ". This may impact performance. Run \"sudo ethtool -G " + dev + " rx " + std::to_string(max_size) + "\" to fix it.");
            }
        } catch(...) {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Unable to check ring buffer size for the ethernet device used by " + ip + ". Find the interface used by " + ip + " then run ethtool -g <dev>.\nIf the value of \"RX:\" under \"Current hardware settings:\" is less than the value of \"RX:\" under \"Pre-set maximums:\" run \"sudo ethtool -G <dev> rx <maximum>\". This may impact performance.");
        }
    }

};

class recv_packet_streamer_mmsg : public recv_packet_handler_mmsg, public rx_streamer
{
public:
    recv_packet_streamer_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuers)
    : recv_packet_handler_mmsg(recv_sockets, dst_ip, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian, device_total_rx_channels, cmd_issuers)
    {
    }

    //Consider merging recv_packet_streamer_mmsg and recv_packet_handler_mmsg
    //This is here to implement a virtual function from rx_streamer
    UHD_INLINE size_t recv(const rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet) override
    {
        // Set flag for if the user ever requested a subopitmal number of samples per buffer
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

    // Issues the stream command
    UHD_INLINE void issue_stream_cmd(const stream_cmd_t& stream_cmd) {
        recv_packet_handler_mmsg::issue_stream_cmd(stream_cmd);
    }
};

}}} // namespace uhd::transport::sph

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP */
