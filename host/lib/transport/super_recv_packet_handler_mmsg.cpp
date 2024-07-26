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
    : recv_packet_handler(max_sample_bytes_per_packet + header_size), _NUM_CHANNELS(recv_sockets.size()), _MAX_SAMPLE_BYTES_PER_PACKET(max_sample_bytes_per_packet),
    _HEADER_SIZE(header_size),
    _TRAILER_SIZE(trailer_size),
    _recv_sockets(recv_sockets),
    _num_cached_samples(_NUM_CHANNELS, 0),
    _sample_cache(_NUM_CHANNELS, std::vector<uint8_t>(_MAX_SAMPLE_BYTES_PER_PACKET))
    {
        if (wire_format=="sc16") {
            _BYTES_PER_SAMPLE = 4;
        } else if (wire_format=="sc12") {
            _BYTES_PER_SAMPLE = 3;
        } else {
            throw uhd::runtime_error( "Unsupported wire format:" + wire_format);
        }

        // Performs socket setup
        // Sockets passed to this constructor must already be bound
        for(size_t n = 0; n < _NUM_CHANNELS; n++) {
            // Sets the recv buffer size
            setsockopt(_recv_sockets[n], SOL_SOCKET, SO_RCVBUF, &_DEFAULT_RECV_BUFFER_SIZE, sizeof(_DEFAULT_RECV_BUFFER_SIZE));

            // Checks the recv buffer size
            // Actual recv buffer size, the Kernel will set the real size to be double the requested
            int actual_recv_buffer_size;
            socklen_t opt_len = sizeof(actual_recv_buffer_size);
            getsockopt(_recv_sockets[n], SOL_SOCKET, SO_RCVBUF, &actual_recv_buffer_size, &opt_len);

            // NOTE: The kernel will set the actual size to be double the requested. So the expected amount is double the requested
            if(actual_recv_buffer_size < 2*_DEFAULT_RECV_BUFFER_SIZE) {
                fprintf(stderr, "Unable to set recv buffer size. Performance may be affected\nTarget size %i\nActual size %i\nPlease run \"sudo sysctl -w net.core.rmem_max=%i\"\n", _DEFAULT_RECV_BUFFER_SIZE, actual_recv_buffer_size/2, _DEFAULT_RECV_BUFFER_SIZE);
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
        }

        setup_converter(cpu_format, wire_format, wire_little_endian);

        // Check if any core is not set to performance mode, used to decide if an info message should be printed if overflows occur
        _using_performance_governor = true;
        std::vector<std::string> governors = uhd::get_performance_governors();
        if(governors.size() != 0) {
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
            UHD_LOGGER_ERROR("PACKET_HANDLER_MMSG") << "Unable to get cache line size, assuming it is 64";
            cache_line_size = 64;
        }
        // Create manager for threads that receive data to buffers using placement new to avoid false sharing
        size_t recv_manager_size = (size_t) ceil(sizeof(async_recv_manager) / (double)cache_line_size) * cache_line_size;
        recv_manager = (async_recv_manager*) aligned_alloc(cache_line_size, recv_manager_size);
        new (recv_manager) async_recv_manager(device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet, device_total_rx_channels);
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
    }

    // Function used to receive data
    size_t recv(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)
    {
        // Clears the metadata struct
        // Reponsible for setting error_code to none, has_time_spec to false and eob to false
        metadata.reset();

        // Metadata of Vita packets
        std::vector<uint8_t*> packets(_NUM_CHANNELS);
        std::vector<vrt::if_packet_info_t> vita_md(_NUM_CHANNELS);
        std::vector<uint64_t> packet_tsfs(_NUM_CHANNELS);

        // Sequence number and timestamp on the latest packet
        // Should be equal for all packets, used during realignment
        size_t latest_sequence_number = 0;
        uint64_t latest_packet = 0;

        bool align_message_printed = false;

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

        time_spec_t recv_start_time = get_system_time();

        // Limit for how many time realignment will be attempted before giving up
        const size_t max_realignment_attempts = 100;
        size_t realignment_attempts = 0;

        // Flag used to ignore the timeout on the first check for data
        uint_fast8_t first_loop = true;

        // Main receive loop
        while(samples_received < nsamps_per_buff && (recv_start_time + timeout > get_system_time() || first_loop)) {
            first_loop = false;

            bool all_ready = true;
            bool underflow_detected = false;
            bool realignment_required = false;

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                packets[ch] = recv_manager->get_next_packet(ch);

                if(packets[ch] == nullptr) {
                    all_ready = false;
                    break;
                }

                uint32_t packet_length = recv_manager->get_next_packet_length(ch);
                if(packet_length < _HEADER_SIZE) {
                    throw std::runtime_error("Received sample packet smaller than header size");
                }

                // Maximum size the packet length field in Vita packet could be ( + _TRAILER_SIZE since we drop the trailer)
                vita_md[ch].num_packet_words32 = (packet_length + _TRAILER_SIZE) / sizeof(uint32_t);

                // Extract Vita metadata
                if_hdr_unpack((uint32_t*) packets[ch], vita_md[ch]);

                // TODO: enable this once eob flag is properly implement in packets && cache it in the eve
                // Currently Crimson will always have eob and Cyan will never have
                // metadata.end_of_burst |= vita_md[ch].eob

                // Finds and records the sequence number and timestamp of whichever channel's next packet is last
                // Used for realignment, normally they will be the same for all packets
                packet_tsfs[ch] = vita_md[ch].tsf;
                if(latest_packet < packet_tsfs[ch]) {
                    latest_packet = packet_tsfs[ch];
                    latest_sequence_number = vita_md[ch].packet_count;
                }

                // Set the flag for realignment required if there is a mismatch in timestamps between packets
                realignment_required = packet_tsfs[ch] != packet_tsfs[0] || realignment_required;

                // Detect and warn user of overflow error
                if(vita_md[ch].packet_count != (sequence_number_mask & (previous_sequence_number + 1))  && vita_md[ch].tsf != 0) {
                    metadata.error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
                    underflow_detected = true;
                }
            }

            // Not all channels have data ready
            if(!all_ready) {
                // TODO remove after debugging done
                usleep(1);
                continue;
            }

            // Advance (skip) packets that are behind the latest packet so that they catch up after a few iterations of this
            if(underflow_detected) {
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

            if(realignment_required) {
                if(realignment_attempts >= max_realignment_attempts) {
                    if(!align_message_printed) {
                        UHD_LOGGER_ERROR("STREAMER") << "Failed to re-align channels after overflow";
                    }
                    align_message_printed = true;
                    // Override overflow error with alignment error to indicate that UHD was unable to fully recover from the overflow
                    metadata.error_code = rx_metadata_t::ERROR_CODE_ALIGNMENT;
                } else {
                    for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                        if(packet_tsfs[ch] != latest_packet) {
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

            // Update the sequence number
            previous_sequence_number = latest_sequence_number;

            // Update tsf cache to most recent (this packet)
            tsf_cache = vita_md[0].tsf;

            // Set the timepec to that of the first packet received if not already set from the cache
            // They should be equal to the only the first channel's is used
            if(!metadata.has_time_spec) {
                metadata.has_time_spec = true;
                // No need to check for has_tsf since our FPGAs always include tsf
                metadata.time_spec = time_spec_t::from_ticks(vita_md[0].tsf, _sample_rate);
            }

            size_t packet_sample_bytes = vita_md[0].num_payload_bytes;
            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Error checking for if there is a mismatch in packet lengths
                if(packet_sample_bytes != vita_md[ch].num_payload_bytes) {
                    packet_sample_bytes = std::min(packet_sample_bytes, vita_md[ch].num_payload_bytes);
                    UHD_LOGGER_ERROR("STREAMER") << "Mismatch in sample count between packets";

                    // Something is wrong with the packets if there is a mismatch in size and no other error has occured
                    if(metadata.error_code == rx_metadata_t::ERROR_CODE_NONE) {
                        metadata.error_code = rx_metadata_t::ERROR_CODE_BAD_PACKET;
                    }
                }

                size_t samples_in_packet = vita_md[ch].num_payload_bytes / _BYTES_PER_SAMPLE;
                // Number of samples in the packet that fit in the user's buffer
                size_t samples_to_consume = std::min(samples_in_packet, nsamps_per_buff - samples_received);
                // Number of samples in the packet that don't fit in the user's buffer and need to be cached until the next recv
                size_t samples_to_cache = samples_in_packet - samples_to_consume;
                // Copies data from provider buffer to the user's buffer,
                convert_samples(buffs[ch], packets[ch] + _HEADER_SIZE, samples_to_consume);

                if(samples_to_cache) {
                    // Copy extra samples from the packet to the cache
                    memcpy(_sample_cache[ch].data(), packets[ch] + _HEADER_SIZE + (samples_to_consume * _BYTES_PER_SAMPLE), samples_to_cache * _BYTES_PER_SAMPLE);
                    eob_cached = metadata.end_of_burst;
                    metadata.end_of_burst = false;
                }
                _num_cached_samples[ch] = samples_to_cache;

                // Moves to the next packet
                recv_manager->advance_packet(ch);

            }

            samples_received += packet_sample_bytes / _BYTES_PER_SAMPLE;

            // Exit loop if user only wants one packet
            if(one_packet) {
                break;
            }

        }

        // Return a timeout error only if no samples were received
        if(samples_received == 0 && metadata.error_code == rx_metadata_t::ERROR_CODE_NONE) {
            metadata.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
        }

        return samples_received;
    }

    // Set the rate of samples per second
    void set_sample_rate(const double rate)
    {
        _sample_rate = rate;
    }

protected:
    size_t _NUM_CHANNELS;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _BYTES_PER_SAMPLE;

    virtual void if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) = 0;

private:
    // Desired recv buffer size
    const int _DEFAULT_RECV_BUFFER_SIZE = 500000000;
    // Maximum number of packets to recv (should be able to fit in the half the real buffer)
    size_t _HEADER_SIZE;
    // Trailer is not needed for anything so receive will discard it
    size_t _TRAILER_SIZE;
    std::vector<int> _recv_sockets;
    // Maximum sequence number
    const size_t sequence_number_mask = 0xf;

    // Number of samples in the previous recv. Used for simulating timestamp for cached samples
    time_spec_t previous_timestamp = time_spec_t(0.0);
    size_t previous_num_samples = 0;

    size_t previous_sequence_number = 0;

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

    // EOB was detected and should be applied to the caced samples
    uint_fast8_t eob_cached = false;

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
    inline void convert_samples(const ref_vector<void*> dst, void* src, size_t num_samples) {
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
