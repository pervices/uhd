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

#include <immintrin.h>

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
    recv_packet_handler_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels)
    : recv_packet_handler(max_sample_bytes_per_packet + header_size),
    page_size(getpagesize()),
    _NUM_CHANNELS(recv_sockets.size()),
    _MAX_SAMPLE_BYTES_PER_PACKET(max_sample_bytes_per_packet),
    // TODO: select based on number of channels
    _optimized_recv((_NUM_CHANNELS != 1) ? &uhd::transport::sph::recv_packet_handler_mmsg::multi_ch_recv : &uhd::transport::sph::recv_packet_handler_mmsg::recv_single_ch_sequential),
    _HEADER_SIZE(header_size),
    _TRAILER_SIZE(trailer_size),
    _recv_sockets(recv_sockets),
    _previous_buffer_writes_count(_NUM_CHANNELS, 0),
    _intermediate_recv_buffer_pointers(_NUM_CHANNELS),
    _intermediate_recv_buffer_wrapper(_intermediate_recv_buffer_pointers.data(), _NUM_CHANNELS),
    _num_cached_samples(_NUM_CHANNELS, 0),
    _sample_cache(_NUM_CHANNELS, std::vector<uint8_t>(_MAX_SAMPLE_BYTES_PER_PACKET, 0))
    {
        if (wire_format=="sc16") {
            _BYTES_PER_SAMPLE = 4;
        } else if (wire_format=="sc12") {
            _BYTES_PER_SAMPLE = 3;
        } else {
            throw uhd::runtime_error( "Unsupported wire format:" + wire_format);
        }

        // Performs a check (and if applicable warning message) for potential source of performance issues
        check_high_order_alloc_disable();

        // Performs socket setup
        // Sockets passed to this constructor must already be bound
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            // Sets the recv buffer size
            setsockopt(_recv_sockets[n], SOL_SOCKET, SO_RCVBUF, &_DEFAULT_RECV_BUFFER_SIZE, sizeof(_DEFAULT_RECV_BUFFER_SIZE));

            // Checks the recv buffer size
            // Actual recv buffer size, the Kernel will set the real size to be double the requested
            // TODO: change _ACTUAL_RECV_BUFFER_SIZE to local variable once recv_single_ch_sequential is removed
            socklen_t opt_len = sizeof(_ACTUAL_RECV_BUFFER_SIZE);
            getsockopt(_recv_sockets[n], SOL_SOCKET, SO_RCVBUF, &_ACTUAL_RECV_BUFFER_SIZE, &opt_len);

            // NOTE: The kernel will set the actual size to be double the requested. So the expected amount is double the requested
            if(_ACTUAL_RECV_BUFFER_SIZE < 2*_DEFAULT_RECV_BUFFER_SIZE) {
                fprintf(stderr, "Unable to set recv buffer size. Performance may be affected\nTarget size %i\nActual size %i\nPlease run \"sudo sysctl -w net.core.rmem_max=%i\"\n", _DEFAULT_RECV_BUFFER_SIZE, _ACTUAL_RECV_BUFFER_SIZE/2, _DEFAULT_RECV_BUFFER_SIZE);
                throw uhd::system_error("Unable to set recv socket size");
            }

            int mtu = get_mtu(_recv_sockets[n], dst_ip[n].c_str());
            if(mtu < MIN_MTU) {
                fprintf(stderr, "MTU of interface associated with %s is to small. %i required, current value is %i", dst_ip[n].c_str(), MIN_MTU, mtu);
                throw uhd::system_error("MTU size to small");
            }

            int set_priority_ret = setsockopt(_recv_sockets[n], SOL_SOCKET, SO_PRIORITY, &RX_SO_PRIORITY, sizeof(RX_SO_PRIORITY));
            if(set_priority_ret) {
                fprintf(stderr, "Attempting to set rx socket priority failed with error code: %s", strerror(errno));
            }

            // recvmmsg should attempt to recv at most the amount to fill 1/_NUM_CHANNELS of the socket buffer
            _MAX_PACKETS_TO_RECV = (int)((_ACTUAL_RECV_BUFFER_SIZE/(_NUM_CHANNELS + 1))/(_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET + 42));
        }

        ch_recv_buffer_info_i = {
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
        memset(ch_recv_buffer_info_i.msgs.data(), 0, sizeof(mmsghdr)*_num_header_buffers);
        // Contains data about where to store received data
        // Alternating between pointer to header, pointer to data
        memset(ch_recv_buffer_info_i.iovecs.data(), 0, sizeof(iovec)*ch_recv_buffer_info_i.iovecs.size());

        setup_converter(cpu_format, wire_format, wire_little_endian);

        // Check if any core is not set to performance mode, used to decide if an info message should be printed if overflows occur
        _using_performance_governor = true;
        std::vector<std::string> governors = uhd::get_performance_governors();
        if(governors.size() != 0) {
            _governor_known = true;
            for(auto& g : governors) {
                if(g.find("performance") == std::string::npos) {
                    _using_performance_governor = false;
                    break;
                }
            }
        } else {
            _governor_known = false;
            _using_performance_governor = false;
        }

        // Create manager for receive threads and access to buffer recv data
        size_t cache_line_size = sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
        if(cache_line_size == 0) {
            UHD_LOGGER_ERROR("RECV_PACKET_HANDLER_MMSG") << "Unable to get cache line size, assuming it is 64";
            cache_line_size = 64;
        }
        // With 1 channel the old method is used which doesn't use the manager
        if(_NUM_CHANNELS != 1) {
            // Create manager for threads that receive data to buffers using placement new to avoid false sharing
            size_t recv_manager_size = (size_t) ceil(sizeof(async_recv_manager) / (double)cache_line_size) * cache_line_size;
            recv_manager = (async_recv_manager*) aligned_alloc(cache_line_size, recv_manager_size);
            new (recv_manager) async_recv_manager(device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet, device_total_rx_channels);
        }
    }

    ~recv_packet_handler_mmsg(void)
    {
        if(_NUM_CHANNELS != 1) {
            // recv_manager must be deleted before closing sockets
            // Destructor must be manually called when using placement new
            recv_manager->~async_recv_manager();
            free(recv_manager);
        }
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

    bool first_run = true;

    UHD_INLINE size_t recv(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)
    {
        // A suboptimal number of samples per call is anything that is not a multiple of the packet length
        _suboptimal_spb |= ((nsamps_per_buff * _BYTES_PER_SAMPLE) % _MAX_SAMPLE_BYTES_PER_PACKET);
        return (this->*_optimized_recv)(buffs, nsamps_per_buff, metadata, timeout, one_packet);
    }

    // Function used to receive data for multiple channels
    size_t multi_ch_recv(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)
    {
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

            // if(first_run) {
            //     std::vector<int> random(1000000, 1);
            //     time_spec_t bust_end_time = uhd::get_system_time() + 100;
            //     while(bust_end_time > uhd::get_system_time()) {
            //         for(size_t n = 0; n < 1000000; n++) {
            //             random[n] = random[n] * rand();
            //         }
            //     }
            //     first_run = false;
            // }

            size_t ch = 0;
            // While not all channels have been obtained and timeout has not been reached
            while(ch < _NUM_CHANNELS && recv_start_time + timeout > get_system_time()) {
                initial_buffer_writes_count[ch] = recv_manager->get_buffer_write_count(ch);
                std::atomic_thread_fence(std::memory_order_consume);
                // if (buffer_write_count has increased since the last recv || the next packet is not the first packet of the buffer) && buffer_write_count is even
                if((initial_buffer_writes_count[ch] > _previous_buffer_writes_count[ch] || !recv_manager->is_first_packet_of_buffer(ch)) && !(initial_buffer_writes_count[ch] & 1)) {
                    // Move onto the next channel since this one is ready
                    ch++;
                } else {
                    // Lets CPU know this is in a spin loop
                    // Helps performance so the branch predictor doesn't get killed by the loop
                    // usleep(1);
                    _mm_pause();
                }
            }

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
                if(vita_md[ch].packet_count != (sequence_number_mask & (previous_sequence_number + 1))  && vita_md[ch].tsf != 0) [[unlikely]] {
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

    // TODO: replace with single_ch_recv_threaded once it is fast enough
    size_t recv_single_ch_sequential(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)
    {
        // This function is not only used for single channel mode only
        constexpr size_t ch = 0;
        // Clears the metadata struct, theoretically not required but included to make sure mistakes don't cause non-deterministic behaviour
        metadata.reset();

        size_t bytes_per_buff = nsamps_per_buff * _BYTES_PER_SAMPLE;

        // If no converter is required data will be written directly into buffs, otherwise it is written to an intermediate buffer
        const uhd::rx_streamer::buffs_type *recv_buffer = (converter_required) ? prepare_intermediate_buffers(bytes_per_buff) : &buffs;

        // Number of bytes copied from the cache (should be the same for every channel)
        size_t cached_bytes_to_copy = 0;
        // Number of samples to copy, this will be reduced by the number cached samples copied
        size_t bytes_to_recv = bytes_per_buff;

        // Copies the data from the sample cache
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

        // Returns the number of samples requested, if there were enough samples in the cache
        if(!bytes_to_recv || eob_cached) {
            metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;

            // Sets the end of burst flag if cached, the clears it
            metadata.end_of_burst = eob_cached;
            eob_cached = false;

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
        metadata.time_spec = time_spec_t::from_ticks(ch_recv_buffer_info_i.vrt_metadata[0].tsf - /* Simulate what timestamp would'bve been if it was for the start of the cached samples */cached_bytes_to_copy, _sample_rate);
        previous_timestamp = metadata.time_spec;

        // Check for overflow errors and flushed packets to keep buffers aligned
        size_t aligned_bytes = align_buffs(metadata.error_code) + cached_bytes_to_copy;


        metadata.end_of_burst = detect_end_of_burst();

        size_t final_nsamps = aligned_bytes/_BYTES_PER_SAMPLE;

        if(converter_required) {
            old_convert_samples(buffs, final_nsamps);
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
    const size_t page_size;
    size_t _NUM_CHANNELS;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _BYTES_PER_SAMPLE;

    virtual void if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

private:
    // Used to select between variants of the receive function optimized for a single channel o
    typedef size_t (uhd::transport::sph::recv_packet_handler_mmsg::*optimized_recv_type) (const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet);

    optimized_recv_type _optimized_recv;

    // Desired recv buffer size
    const int _DEFAULT_RECV_BUFFER_SIZE = 500000000;
    // TODO: remove once recv_single_ch_sequential is removed
    // Actual recv buffer size, not the Kernel will set the real size to be double the requested
    int _ACTUAL_RECV_BUFFER_SIZE;
    // TODO: remove once recv_single_ch_sequential is removed
    // Maximum number of packets to recv (should be able to fit in the half the real buffer)
    int _MAX_PACKETS_TO_RECV;
    // Maximum number of packets to recv (should be able to fit in the half the real buffer)
    size_t _HEADER_SIZE;
    // Trailer is not needed for anything so receive will discard it
    size_t _TRAILER_SIZE;
    // TODO: remove once recv_single_ch_sequential is removed
    size_t _num_header_buffers = 32;
    std::vector<int> _recv_sockets;
    // Maximum sequence number
    const size_t sequence_number_mask = 0xf;
    // TODO: see if using this is missing in new functions, if not needed mark as to be removed once recv_single_ch_sequential is gone

    // Number of samples in the previous recv. Used for simulating timestamp for cached samples
    time_spec_t previous_timestamp = time_spec_t(0.0);
    size_t previous_num_samples = 0;

    size_t previous_sequence_number = 0;

    // Value of buffer_write_count during the last recv call
    std::vector<int_fast64_t> _previous_buffer_writes_count;

    // TODO: remove once recv_single_ch_sequential is removed
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
    // Group of recv info (used in 1 channel mode)
    ch_recv_buffer_info ch_recv_buffer_info_i;

    // Whether or not a conversion is required between CPU and wire formats (convert still may be used anyway since it is a fast way of copying)
    bool converter_required;

    // TODO: remove once recv_single_ch_sequential is removed
    // Pointers to the start of the recv buffer for each channel
    std::vector<void*> _intermediate_recv_buffer_pointers;
    // TODO: remove once recv_single_ch_sequential is removed
    // Wrapper to be use the same dataype as the regular buffer
    uhd::rx_streamer::buffs_type _intermediate_recv_buffer_wrapper;

    // Converts samples between wire and cpu formats
    uhd::convert::converter::sptr _converter;

    // Sample rate in samples per second
    double _sample_rate = 0;

    // Stores whether or not the CPU governor is set to performance mode
    // NOTE: getting this is done at the start, but the warning related to it only prints during streaming, assumes the governor does not change while the program is running
    bool _using_performance_governor;
    bool _governor_known;
    // The warning for using non performance governor has already been printed
    bool _performance_warning_printed = false;

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
        converter_required = !(cpu_format == wire_format && wire_little_endian);

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

    // TODO: remove once recv_single_ch_sequential is removed
    // Resizes the intermediate buffers (if needed) and updates the
    // Returns the wrapper (set to the data in the constructor, done as a workaround because the UHD uses a vector wrapper)
    UHD_INLINE uhd::rx_streamer::buffs_type* prepare_intermediate_buffers(size_t bytes) {
        if(ch_recv_buffer_info_i.intermediate_recv_buffer.size() < bytes) {
            // Resizes intermediate buffer
            ch_recv_buffer_info_i.intermediate_recv_buffer.resize(bytes);
        }
        // Updates the pointer to the intermediate buffer
        _intermediate_recv_buffer_pointers[0] = ch_recv_buffer_info_i.intermediate_recv_buffer.data();

        return &_intermediate_recv_buffer_wrapper;
    }

    /**
     * Copies samples from src to dst using SIMD, converts between data formats if applicable.
     * @param dst the destination to copy to. Must be at least of dst sample size * num_samples
     * @param src the source to copy to. Must be at least src sample size * num_samples
     * @param num_samples the number of samples to copy
     */
    UHD_INLINE void convert_samples(const ref_vector<void*> dst, void* src, size_t num_samples) {
        // TODO: investigate if this be optimized to reduce branching
        _converter->conv(src, dst, num_samples);
    }

    // TODO: remove once recv_single_ch_sequential is removed
    // Copies samples from _intermediate_recv_buffer_pointers to user_buffer
    UHD_INLINE void old_convert_samples(const uhd::rx_streamer::buffs_type& user_buffer_ptrs, size_t num_samples) {
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            // TODO figure out how the converter works to optimize this, it might be possible to do all at once
            const ref_vector<void*> user_buffer_ch(user_buffer_ptrs[n]);
            // Converts the samples
            _converter->conv(_intermediate_recv_buffer_pointers[n], user_buffer_ch, num_samples);
        }
    }

    // TODO: remove once recv_single_ch_sequential is removed
    bool detect_end_of_burst() {
        //TODO: fix FPGA and re-enable this function. At time of writing Crimson will always have eob flag and Cyan will never have it
        return false;
        // bool end_of_burst_received = false;
        // if(ch_recv_buffer_info_i.num_headers_used > 0) {
        //     // Set end of burst if received on any channel
        //     if(ch_recv_buffer_info_i.vrt_metadata[ch_recv_buffer_info_i.num_headers_used-1].eob) {
        //         end_of_burst_received = true;
        //     }
        // }
        //
        // bool should_cache_eob = end_of_burst_received;
        // if(end_of_burst_received) {
            // if(ch_recv_buffer_info_i.sample_cache_used == 0) {
            //     should_cache_eob = false;
            // }
        // }
        //
        // // EOB received, but there as samples in the cache so it should be saved until the next receive
        // if(should_cache_eob && end_of_burst_received) {
        //     eob_cached = true;
        //     return false;
        // // EOB received and there are no samples in the cache so clear EOB cache and set metadata flag
        // } else if(end_of_burst_received) {
        //     eob_cached = false;
        //     return true;
        // } else {
        // // Clear EOB cache, should be unreachable because
        //     eob_cached = false;
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

    // TODO: remove once recv_single_ch_sequential is removed
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
        // This function is not only used for single channel mode only
        constexpr size_t ch = 0;

        size_t nbytes_to_recv = buffer_length_bytes - sample_buffer_offset;

        // Pointers for where to write samples to from each packet using scatter gather
        std::vector<void*> samples_sg_dst;

        // Clears number of headers (which is also a count of number of packets received
        ch_recv_buffer_info_i.num_headers_used = 0;
        // Resets the count for amount of data in the cache
        ch_recv_buffer_info_i.sample_cache_used = 0;

        //Fills the pointer to where in the buffer to write samples from the packet to
        for(size_t p = sample_buffer_offset; p < buffer_length_bytes; p += _MAX_SAMPLE_BYTES_PER_PACKET) {
            samples_sg_dst.push_back(p+(uint8_t*)(sample_buffers[ch]));
        }

        size_t num_packets_to_recv = samples_sg_dst.size();

        // Adds more room to store headers if required
        if(num_packets_to_recv > _num_header_buffers) {
            _num_header_buffers = num_packets_to_recv;
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

        // Amount of data in the last packet copied directly to buffer
        // Amount of data stored in the cache betwen recv
        size_t expected_excess_data_in_last_packet = num_packets_to_recv * _MAX_SAMPLE_BYTES_PER_PACKET - nbytes_to_recv;
        size_t expected_data_in_last_packet = _MAX_SAMPLE_BYTES_PER_PACKET - expected_excess_data_in_last_packet;

        for (size_t n = 0; n < num_packets_to_recv - 1; n++) {
            // Location to write header data to
            ch_recv_buffer_info_i.iovecs[2*n].iov_base = 0;
            ch_recv_buffer_info_i.iovecs[2*n].iov_base = ch_recv_buffer_info_i.headers[n].data();
            ch_recv_buffer_info_i.iovecs[2*n].iov_len = _HEADER_SIZE;
            ch_recv_buffer_info_i.iovecs[2*n+1].iov_base = 0;
            // Location to write sample data to
            ch_recv_buffer_info_i.iovecs[2*n+1].iov_base = samples_sg_dst[n];
            ch_recv_buffer_info_i.iovecs[2*n+1].iov_len = _MAX_SAMPLE_BYTES_PER_PACKET;
            ch_recv_buffer_info_i.msgs[n].msg_hdr.msg_iov = &ch_recv_buffer_info_i.iovecs[2*n];
            ch_recv_buffer_info_i.msgs[n].msg_hdr.msg_iovlen = 2;
        }


        size_t n_last_packet = num_packets_to_recv - 1;
        // Location to write header data to
        ch_recv_buffer_info_i.iovecs[2*n_last_packet].iov_base =ch_recv_buffer_info_i.headers[n_last_packet].data();
        ch_recv_buffer_info_i.iovecs[2*n_last_packet].iov_len = _HEADER_SIZE;
        // Location to write sample data to
        ch_recv_buffer_info_i.iovecs[2*n_last_packet+1].iov_base = samples_sg_dst[n_last_packet];
        ch_recv_buffer_info_i.iovecs[2*n_last_packet+1].iov_len = expected_data_in_last_packet;
        // Location to write samples that don't fit in sample_buffer to
        ch_recv_buffer_info_i.iovecs[2*n_last_packet+2].iov_base = ch_recv_buffer_info_i.sample_cache.data();
        ch_recv_buffer_info_i.iovecs[2*n_last_packet+2].iov_len = expected_excess_data_in_last_packet;
        ch_recv_buffer_info_i.msgs[n_last_packet].msg_hdr.msg_iov = &ch_recv_buffer_info_i.iovecs[2*n_last_packet];
        ch_recv_buffer_info_i.msgs[n_last_packet].msg_hdr.msg_iovlen = 3;

        // Gets the start time for use in the timeout, uses CLOCK_MONOTONIC_COARSE because it is faster and precision doesn't matter for timeouts
        struct timespec recv_start_time;
        clock_gettime(CLOCK_MONOTONIC_COARSE, &recv_start_time);
        int64_t recv_timeout_time_ns = (recv_start_time.tv_sec * 1000000000) + recv_start_time.tv_nsec + (int64_t)(timeout * 1000000000);

        // Flag to indicate if a timeout occured. Note: timeout should only be reported if no data was received
        bool timeout_occured = false;
        size_t num_channels_serviced = 0;
        while(num_channels_serviced < _NUM_CHANNELS) {
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC_COARSE, &current_time);
            int64_t current_time_ns = (current_time.tv_sec * 1000000000) + current_time.tv_nsec;
            if(current_time_ns > recv_timeout_time_ns) {
                timeout_occured = true;
                break;
            }

            // Skip this channel if it has already received enough packets
            if(ch_recv_buffer_info_i.num_headers_used >= num_packets_to_recv) {
                continue;
            }
            // Receive packets system call
            int num_packets_received_this_recv = recvmmsg(_recv_sockets[ch], &ch_recv_buffer_info_i.msgs[ch_recv_buffer_info_i.num_headers_used], std::min((int)(num_packets_to_recv - ch_recv_buffer_info_i.num_headers_used), _MAX_PACKETS_TO_RECV), MSG_DONTWAIT, 0);

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

        extract_vrt_metadata();

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

        if(timeout_occured) {
            if(ch_recv_buffer_info_i.num_headers_used == 0) {
                return rx_metadata_t::ERROR_CODE_TIMEOUT;
            }
            // Only return timeout if one of the channels received no data
            return rx_metadata_t::ERROR_CODE_NONE;
        } else {
            return rx_metadata_t::ERROR_CODE_NONE;
        }
    }

    // TODO: remove once recv_single_ch_sequential is removed
    /*******************************************************************
     * extract_vrt_metadata:
     * extracts metadata fromthe vrt headers in ch_recv_buffer_info.headers and stores in ch_recv_buffer_info.vrt_metadata
     ******************************************************************/
    UHD_INLINE void extract_vrt_metadata() {
        for(size_t packet_i = 0; packet_i < ch_recv_buffer_info_i.num_headers_used; packet_i++) {
            // Number of 32 bit words per vrt packet
            // will be compared against packet length field
            ch_recv_buffer_info_i.vrt_metadata[packet_i].num_packet_words32 = (_HEADER_SIZE + _MAX_SAMPLE_BYTES_PER_PACKET +_TRAILER_SIZE)/sizeof(uint32_t);
            // First word of the packet
            const uint32_t* vrt_hdr = (uint32_t*) ch_recv_buffer_info_i.headers[packet_i].data();
            //ifpi.has_tsf = true;
            if_hdr_unpack(vrt_hdr, ch_recv_buffer_info_i.vrt_metadata[packet_i]);
        }
    }

    // TODO: remove once recv_single_ch_sequential is removed
    /*******************************************************************
     * align_buffs:
     * Checks for sequence number or timestamp errors and drops samples to keep channels aligned
     * error_code where to store the error code
     * return: Number of aligned bytes
     ******************************************************************/
    UHD_INLINE uint64_t align_buffs(uhd::rx_metadata_t::error_code_t& error_code) {

        bool oflow_error = false;

        uint64_t aligned_bytes = 0;

        // Only print these messages once per recv call
        bool oflow_message_printed = false;

        // Checks for overflows
        // Each channel should end up with the same number of aligned bytes so its fine to reset the counter each channel which will end up using the last one
        for(size_t header_i = 0; header_i < ch_recv_buffer_info_i.num_headers_used; header_i++) {
            //assume vrt_metadata.hst_tsf is true
            // Checks if sequence number is correct, ignore check if timestamp is 0
            if((ch_recv_buffer_info_i.vrt_metadata[header_i].packet_count != (sequence_number_mask & (ch_recv_buffer_info_i.previous_sequence_number + 1)))  && (ch_recv_buffer_info_i.vrt_metadata[header_i].tsf != 0)) {
                oflow_error = true;
                if(!oflow_message_printed) {
                    // Only print the message once per recv call
                    oflow_message_printed = true;

                    //UHD_LOG_FASTPATH("D" + std::to_string(ch_recv_buffer_info_i.vrt_metadata[header_i].tsf) + "\n");
                    UHD_LOG_FASTPATH("D");

                    if(!_performance_warning_printed) {
                        _performance_warning_printed = true;
                        if(!_governor_known) {
                            UHD_LOG_FASTPATH("\nRecv overflow detected, ensure the CPU governor is set to performance. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                        }
                        else if(!_using_performance_governor) {
                            UHD_LOG_FASTPATH("\nRecv overflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
                        }
                    }
                }
            }
            ch_recv_buffer_info_i.previous_sequence_number = ch_recv_buffer_info_i.vrt_metadata[header_i].packet_count;
            aligned_bytes += ch_recv_buffer_info_i.data_bytes_from_packet[header_i];
        }

        // Record if an overflow error occured and no other errors occured
        if(error_code == uhd::rx_metadata_t::ERROR_CODE_NONE && oflow_error) {
            error_code = uhd::rx_metadata_t::ERROR_CODE_OVERFLOW;
            _overflow_occured = true;
        }

        return aligned_bytes;
    }

    void print_overflow_message() {
        // Warn user that an overflow occured
        UHD_LOG_FASTPATH("D");
        if(!_using_performance_governor && !_performance_warning_printed) {
            UHD_LOG_FASTPATH("\nRecv overflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
            _performance_warning_printed = true;
        }
        if(!_performance_warning_printed) {
            _performance_warning_printed = true;
            if(!_governor_known) {
                UHD_LOG_FASTPATH("\nRecv overflow detected, ensure the CPU governor is set to performance. Using governors other than performance can cause spikes in latency which can cause overflows\n");
            }
            else if(!_using_performance_governor) {
                UHD_LOG_FASTPATH("\nRecv overflow detected while not using performance cpu governor. Using governors other than performance can cause spikes in latency which can cause overflows\n");
            }
        }
    }

    /*
     * Check if higher order allocation is enabled.
     * I have experimentally verified that this can cause issues and should be disabled on: Linux wave 6.6.7-arch1-1 #1 SMP PREEMPT_DYNAMIC Thu, 14 Dec 2023 03:45:42 +0000 x86_64 GNU/Linux. The issue is likely due to frequent de and re allocation since the socket's receive buffers (controlled by SO_RCVBUF) are dynamically sized. If a feature is introduced to make those buffers a fixed size then that should be tested.
     * Documentation: https://docs.kernel.org/admin-guide/sysctl/net.html#high-order-alloc-disable
     */
    void check_high_order_alloc_disable() {
        std::string path = "/proc/sys/net/core/high_order_alloc_disable";

        FILE *file;

        file = fopen(path.c_str(), "r");

        if(file == NULL) {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Open " + path + " failed with error code:" + std::string(strerror(errno)) + ". Unable to check if high order allocation enabled. Having it enabled may cause performance issues. Run \"sudo sysctl -w net.core.high_order_alloc_disable=1\" to disable it.");
        }

        int value = fgetc(file);

        if(value == -1) {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Read " + path + " failed with error code:" + std::string(strerror(errno)) + ".Unable to check if high order allocation enabled. Having it enabled may cause performance issues. Run \"sudo sysctl -w net.core.high_order_alloc_disable=1\" to disable it.");
        } else if(value != '1') {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "High order allocation enabled, this may cause performance issues. Run \"sudo sysctl -w net.core.high_order_alloc_disable=1\" to disable it.");
        }
    }

};

class recv_packet_streamer_mmsg : public recv_packet_handler_mmsg, public rx_streamer
{
public:
    recv_packet_streamer_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels)
    : recv_packet_handler_mmsg(recv_sockets, dst_ip, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian, device_total_rx_channels)
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
};

}}} // namespace uhd::transport::sph

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_MMSG_HPP */
