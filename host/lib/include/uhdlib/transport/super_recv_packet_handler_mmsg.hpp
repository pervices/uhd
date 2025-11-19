//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2023-2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <uhd/convert.hpp>
#include <uhd/exception.hpp>
#include <uhd/stream.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhdlib/utils/system_time.hpp>

// Manages sending streaming commands
#include <uhdlib/usrp/common/stream_cmd_issuer.hpp>
#include <string>
#ifdef HAVE_LIBURING
    #include <uhd/transport/io_uring_recv_manager.hpp>
#else
    #include <uhdlib/transport/user_recv_manager.hpp>
#endif

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
    recv_packet_handler_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuers);

    ~recv_packet_handler_mmsg(void);

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

        // Place to store metadata of Vita packets
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

            // Contains the location and length of the next packet for each channel
            std::vector<async_packet_info> next_packet(_NUM_CHANNELS);

            // Bit field of the channels not ready yet
            // Bit 0 = 1 when channel 0 hasn't a samples, bit 1 = 0 when channel 1 has received a packet
            size_t ch_not_ready = (((size_t) 1) << _NUM_CHANNELS) - 1;
            // While not all channels have been obtained and timeout has not been reached
            do {
                for(size_t ch = 0; ch < _NUM_CHANNELS; ch ++) {
                    // Mask blocking everything in ch_not_ready except the channel being checked
                    size_t ch_not_ready_mask = ((size_t) 1) << ch;
                    // If this channel doesn't have data
                    if(ch_not_ready & ch_not_ready_mask) {
                        recv_manager->get_next_async_packet_info(ch, &next_packet[ch]);

                        // If length is non 0, and vita_header and samples are not nullptr
                        // All of these should be true or none
                        // All are checked to make behaviour more predictable if there is an earlier bug
                        if(next_packet[ch].length && next_packet[ch].vita_header && next_packet[ch].samples) {
                            ch_not_ready = ch_not_ready & ~ch_not_ready_mask;
                        } else {
                            // Do nothing
                            // _mm_pause (which marks this as a polling loop) might help, but it appears to make performance worse
                        }
                    }
                }
            } while(ch_not_ready && recv_start_time + timeout > get_system_time());

            // Check if timeout occured (Any channels not ready)
            if(ch_not_ready) [[unlikely]] {
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

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Maximum size the packet length field in Vita packet could be ( + _TRAILER_SIZE since we drop the trailer)
                vita_md[ch].num_packet_words32 = (next_packet[ch].length + _TRAILER_SIZE) / sizeof(uint32_t);

                // Check if the packet is smaller than the header size, which should be impossible
                if(next_packet[ch].length < (int64_t) _HEADER_SIZE) [[unlikely]] {
                    std::string message = "Packet length of " + std::to_string(next_packet[ch].length) + " on channel " + std::to_string(ch) + " less than the expected Vita header size of " + std::to_string(_HEADER_SIZE);
                    UHD_LOG_ERROR("RECV_PACKET_HANDLER", message);
                    throw std::runtime_error(message);
                }
            }

            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {

                // Extract Vita metadata
                if_hdr_unpack((uint32_t*) next_packet[ch].vita_header, vita_md[ch]);

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
                    std::string msg = "OVERFLOW. Packet count: " + std::to_string(vita_md[ch].packet_count);
                    UHD_LOG_WARNING("RECV_PACKET_HANDLER", msg);
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
            // Number of samples to copy to return to the user in this packet
            size_t samples_to_consume = 0;
            // Number of samples in the packet that don't fit in the user's buffer and need to be cached until the next recv
            std::vector<size_t> samples_to_cache(_NUM_CHANNELS, 0);

            // Flag to decide if the packet length error should be printed
            bool print_packet_length_error = false;

            // Copies sample data from the provider buffer to the user buffer
            // NOTE: do not update variables stored between runs in this loop, since the results will need to be discarded if data was overwritten
            for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                // Error checking for if there is a mismatch in packet lengths
                if(packet_sample_bytes != vita_md[ch].num_payload_bytes) [[unlikely]] {
                    packet_sample_bytes = std::min(packet_sample_bytes, vita_md[ch].num_payload_bytes);

                    // Something is wrong with the packets if there is a mismatch in size and no other error has occured
                    if(metadata.error_code == rx_metadata_t::ERROR_CODE_NONE) {
                        metadata.error_code = rx_metadata_t::ERROR_CODE_BAD_PACKET;
                        print_packet_length_error = true;
                    }
                }

                size_t samples_in_packet = vita_md[ch].num_payload_bytes / _BYTES_PER_SAMPLE;
                // Number of samples in the packet that fit in the user's buffer
                samples_to_consume = std::min(samples_in_packet, nsamps_per_buff - samples_received);
                samples_to_cache[ch] = samples_in_packet - samples_to_consume;
                // Copies data from provider buffer to the user's buffer,
                convert_samples((void*) (((uint8_t*)buffs[ch]) + (samples_received * _CPU_BYTES_PER_SAMPLE)), next_packet[ch].samples, samples_to_consume);

                // Not actually unlikely, flagged as unlikely since it is false when all samples per recv call is most optimal
                if(samples_to_cache[ch]) [[unlikely]] {
                    // Copy extra samples from the packet to the cache
                    memcpy(_sample_cache[ch].data(), next_packet[ch].samples + (samples_to_consume * _BYTES_PER_SAMPLE), samples_to_cache[ch] * _BYTES_PER_SAMPLE);
                }
            }
            if(print_packet_length_error) [[unlikely]] {
                std::string message = "Mismatch in sample count between packets:";
                for(size_t ch = 0; ch < _NUM_CHANNELS; ch++) {
                    message += "\n\t" + std::to_string(vita_md[ch].num_payload_bytes / _BYTES_PER_SAMPLE);
                }
                UHD_LOG_ERROR("STREAMER", message);
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
    void set_sample_rate(const double rate);
    void issue_stream_cmd(const stream_cmd_t& stream_cmd);

protected:
    size_t _NUM_CHANNELS;
    size_t _MAX_SAMPLE_BYTES_PER_PACKET;
    size_t _BYTES_PER_SAMPLE;
    size_t _CPU_BYTES_PER_SAMPLE;

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
            _CPU_BYTES_PER_SAMPLE = 8;
        } else if("sc16" == cpu_format) {
            cpu_max = 0x7fff;
            _CPU_BYTES_PER_SAMPLE = 4;
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

    int get_mtu(int socket_fd, std::string ip);

    // Low priority thread to print D to indicate and overflow
    std::thread overflow_messenger;
    // No synchronization is used for the count to avoid any side effects in the main recv loop
    uint64_t oflows_to_print = 0;
    bool stop_overflow_loop = false;

    UHD_INLINE void print_overflow_message() {
        // Warn user that an overflow occured
        oflows_to_print++;
    }

    static void send_overflow_messages_loop(recv_packet_handler_mmsg* self);

    /*
     * Check if higher order allocation is enabled.
     * Informs the user if it is disabled. Older UHD versions could benefit on some CPUs but it worsens performance on others.
     * Let's the user know they can re-enable it since they likely only disabled it due to previous warnings.
     */
    void check_high_order_alloc_disable();

    /*
     * Checks if preemption is set to full.
     * Preemption can cause occasional brief latency spikes that cause overflows at high sample rates
     */
    void check_pre_empt();

    /**
     * Checks if the rx ring buffer for the device with the specified ip is set to it's maximum and prints a warning ot the user if it isn't.
     */
    void check_rx_ring_buffer_size(std::string ip);
};

class recv_packet_streamer_mmsg : public recv_packet_handler_mmsg, public rx_streamer
{
public:
    recv_packet_streamer_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuers);

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

    UHD_INLINE size_t get_num_channels(void) const override {
        return _NUM_CHANNELS;
    }

    // Gets the maximum number of samples per packet
    UHD_INLINE size_t get_max_num_samps(void) const override {
        return _MAX_SAMPLE_BYTES_PER_PACKET/_BYTES_PER_SAMPLE;
    }

    // Issues the stream command
    UHD_INLINE void issue_stream_cmd(const stream_cmd_t& stream_cmd) override{
        recv_packet_handler_mmsg::issue_stream_cmd(stream_cmd);
    }

    void post_input_action(const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t) override;
};
}}}
