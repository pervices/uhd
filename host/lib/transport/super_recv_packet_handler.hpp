//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_HPP
#define INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_HPP

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
#include <boost/dynamic_bitset.hpp>
#include <boost/format.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace uhd { namespace transport { namespace sph {

UHD_INLINE uint32_t get_context_code(
    const uint32_t* vrt_hdr, const vrt::if_packet_info_t& if_packet_info)
{
    // extract the context word (we dont know the endianness so mirror the bytes)
    uint32_t word0 = vrt_hdr[if_packet_info.num_header_words32]
                     | uhd::byteswap(vrt_hdr[if_packet_info.num_header_words32]);
    return word0 & 0xff;
}

typedef std::function<void(void)> handle_overflow_type;
static inline void handle_overflow_nop(void) {}

/***********************************************************************
 * Super receive packet handler
 *
 * A receive packet handler represents a group of channels.
 * The channel group shares a common sample rate.
 * All channels are received in unison in recv().
 **********************************************************************/
class recv_packet_handler
{
public:
    typedef std::function<managed_recv_buffer::sptr(double, int*)> get_buff_type;
    typedef std::function<void(const size_t)> handle_flowctrl_type;
    typedef std::function<void(const uint32_t*)> handle_flowctrl_ack_type;
    typedef std::function<void(const stream_cmd_t&)> issue_stream_cmd_type;
    typedef void (*vrt_unpacker_type)(const uint32_t*, vrt::if_packet_info_t&);
    // typedef std::function<void(const uint32_t *, vrt::if_packet_info_t &)>
    // vrt_unpacker_type;

    /*!
     * Make a new packet handler for receive
     * \param size the number of transport channels
     */
    recv_packet_handler(const size_t size = 1)
        : _queue_error_for_next_call(false), _buffers_infos_index(0)
    {
#ifdef ERROR_INJECT_DROPPED_PACKETS
        recvd_packets = 0;
#endif

        this->resize(size);
        set_alignment_failure_threshold(1000);
    }

    ~recv_packet_handler(void)
    {
        /* NOP */
    }

    //! Resize the number of transport channels
    void resize(const size_t size)
    {
        if (this->size() == size)
            return;
        _props.resize(size);
        // re-initialize all buffers infos by re-creating the vector
        _buffers_infos = std::vector<buffers_info_type>(4, buffers_info_type(size));
    }

    //! Get the channel width of this handler
    size_t size(void) const
    {
        return _props.size();
    }

    //! Setup the vrt unpacker function and offset
    void set_vrt_unpacker(
        const vrt_unpacker_type& vrt_unpacker, const size_t header_offset_words32 = 0)
    {
        _vrt_unpacker          = vrt_unpacker;
        _header_offset_words32 = header_offset_words32;
    }

    /*!
     * Set the threshold for alignment failure.
     * How many packets throw out before giving up?
     * \param threshold number of packets per channel
     */
    void set_alignment_failure_threshold(const size_t threshold)
    {
        _alignment_failure_threshold = threshold * this->size();
    }

    //! Set the rate of ticks per second
    void set_tick_rate(const double rate)
    {
        _tick_rate = rate;
    }

    //! Set the rate of samples per second
    void set_samp_rate(const double rate)
    {
        _samp_rate = rate;
    }

    /*!
     * Set the function to get a managed buffer.
     * \param xport_chan which transport channel
     * \param get_buff the getter function
     */
    void set_xport_chan_get_buff(
        const size_t xport_chan, const get_buff_type& get_buff, const bool flush = true)
    {
        if (flush) {
            int error_code = 0;
            while (get_buff(0.0, &error_code)) {
            };
        }
        _props.at(xport_chan).get_buff = get_buff;
    }

    /*!
     * Flush all transports in the streamer:
     * The packet payload is discarded.
     */
    void flush_all(const double timeout = 0.0)
    {
        _flush_all(timeout);
        return;
    }

    /*!
     * Set the function to handle flow control
     * \param xport_chan which transport channel
     * \param handle_flowctrl the callback function
     */
    void set_xport_handle_flowctrl(const size_t xport_chan,
        const handle_flowctrl_type& handle_flowctrl,
        const size_t update_window,
        const bool do_init = false)
    {
        _props.at(xport_chan).handle_flowctrl = handle_flowctrl;
        // we need the window size to be within the 0xfff (max 12 bit seq)
        _props.at(xport_chan).fc_update_window = std::min<size_t>(update_window, 0xfff);
        if (do_init)
            handle_flowctrl(0);
    }

    void set_xport_handle_flowctrl_ack(
        const size_t xport_chan, const handle_flowctrl_ack_type& handle_flowctrl_ack)
    {
        _props.at(xport_chan).handle_flowctrl_ack = handle_flowctrl_ack;
    }

    //! Set the conversion routine for all channels
    void set_converter(const uhd::convert::id_type& id)
    {
        _num_outputs = id.num_outputs;
        _converter   = uhd::convert::get_converter(id)();
        this->set_scale_factor(1 / 32767.); // update after setting converter
        _bytes_per_otw_item = uhd::convert::get_bytes_per_item(id.input_format);
        _bytes_per_cpu_item = uhd::convert::get_bytes_per_item(id.output_format);
    }

    //! Set the transport channel's overflow handler
    void set_overflow_handler(
        const size_t xport_chan, const handle_overflow_type& handle_overflow)
    {
        _props.at(xport_chan).handle_overflow = handle_overflow;
    }

    //! Set the scale factor used in float conversion
    void set_scale_factor(const double scale_factor)
    {
        _converter->set_scalar(scale_factor);
    }

    //! Set the callback to issue stream commands
    void set_issue_stream_cmd(
        const size_t xport_chan, const issue_stream_cmd_type& issue_stream_cmd)
    {
        _props.at(xport_chan).issue_stream_cmd = issue_stream_cmd;
    }

    //! Overload call to issue stream commands
    void issue_stream_cmd(const stream_cmd_t& stream_cmd)
    {
        if (size() > 1 and stream_cmd.stream_now
            and stream_cmd.stream_mode != stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS) {
            throw uhd::runtime_error(
                "Invalid recv stream command - stream now on multiple channels in a "
                "single streamer will fail to time align.");
        }

        for (size_t i = 0; i < _props.size(); i++) {
            if (_props[i].issue_stream_cmd)
                _props[i].issue_stream_cmd(stream_cmd);
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
        // handle metadata queued from a previous receive
        if (_queue_error_for_next_call) {
            _queue_error_for_next_call = false;
            metadata                   = _queue_metadata;
            // We want to allow a full buffer recv to be cut short by a timeout,
            // but do not want to generate an inline timeout message packet.
            if (_queue_metadata.error_code != rx_metadata_t::ERROR_CODE_TIMEOUT)
                return 0;
        }

        size_t accum_num_samps =
            recv_one_packet(buffs, nsamps_per_buff, metadata, timeout);

        if (one_packet or metadata.end_of_burst) {
            return accum_num_samps;
        }

        // first recv had an error code set, return immediately
        if (metadata.error_code != rx_metadata_t::ERROR_CODE_NONE) {
            return accum_num_samps;
        }

        // loop until buffer is filled or error code
        while (accum_num_samps < nsamps_per_buff) {
            size_t num_samps = recv_one_packet(buffs,
                nsamps_per_buff - accum_num_samps,
                _queue_metadata,
                timeout,
                accum_num_samps * _bytes_per_cpu_item);

            metadata.end_of_burst = _queue_metadata.end_of_burst;

            // metadata had an error code set, store for next call and return
            if (_queue_metadata.error_code != rx_metadata_t::ERROR_CODE_NONE) {
                _queue_error_for_next_call = true;
                break;
            }

            accum_num_samps += num_samps;

            // return immediately if end of burst
            if (_queue_metadata.end_of_burst) {
                break;
            }
        }
        return accum_num_samps;
    }

protected:
    double _samp_rate;

private:
    vrt_unpacker_type _vrt_unpacker;
    size_t _header_offset_words32;
    double _tick_rate;
    bool _queue_error_for_next_call;
    size_t _alignment_failure_threshold;
    rx_metadata_t _queue_metadata;
    struct xport_chan_props_type
    {
        xport_chan_props_type(void)
            : packet_count(0), handle_overflow(&handle_overflow_nop), fc_update_window(0)
        {
        }
        get_buff_type get_buff;
        issue_stream_cmd_type issue_stream_cmd;
        size_t packet_count;
        handle_overflow_type handle_overflow;
        handle_flowctrl_type handle_flowctrl;
        handle_flowctrl_ack_type handle_flowctrl_ack;
        size_t fc_update_window;
    };
    std::vector<xport_chan_props_type> _props;
    size_t _num_outputs;
    size_t _bytes_per_otw_item; // used in conversion
    size_t _bytes_per_cpu_item; // used in conversion
    uhd::convert::converter::sptr _converter; // used in conversion

    //! information stored for a received buffer
    struct per_buffer_info_type
    {
        void reset()
        {
            buff.reset();
            vrt_hdr   = nullptr;
            time      = 0;
            copy_buff = nullptr;
        }
        managed_recv_buffer::sptr buff;
        const uint32_t* vrt_hdr;
        vrt::if_packet_info_t ifpi;
        uint64_t time;
        const char* copy_buff;
    };

    //! information stored for a set of aligned buffers
    struct buffers_info_type : std::vector<per_buffer_info_type>
    {
        buffers_info_type(const size_t size)
            : std::vector<per_buffer_info_type>(size)
            , indexes_todo(size, true)
            , alignment_time(0)
            , alignment_time_valid(false)
            , data_bytes_to_copy(0)
            , fragment_offset_in_samps(0)
        { /* NOP */
        }
        void reset()
        {
            indexes_todo.set();
            alignment_time           = 0;
            alignment_time_valid     = false;
            data_bytes_to_copy       = 0;
            fragment_offset_in_samps = 0;
            metadata.reset();
            for (size_t i = 0; i < size(); i++)
                at(i).reset();
        }
        boost::dynamic_bitset<> indexes_todo; // used in alignment logic
        uint64_t alignment_time; // used in alignment logic
        bool alignment_time_valid; // used in alignment logic
        size_t data_bytes_to_copy; // keeps track of state
        size_t fragment_offset_in_samps; // keeps track of state
        rx_metadata_t metadata; // packet description
    };

    //! a circular queue of buffer infos
    std::vector<buffers_info_type> _buffers_infos;
    size_t _buffers_infos_index;
    buffers_info_type& get_curr_buffer_info(void)
    {
        return _buffers_infos[_buffers_infos_index];
    }
    buffers_info_type& get_prev_buffer_info(void)
    {
        return _buffers_infos[(_buffers_infos_index + 3) % 4];
    }
    buffers_info_type& get_next_buffer_info(void)
    {
        return _buffers_infos[(_buffers_infos_index + 1) % 4];
    }
    void increment_buffer_info(void)
    {
        _buffers_infos_index = (_buffers_infos_index + 1) % 4;
    }

    //! possible return options for the packet receiver
    enum packet_type {
        PACKET_IF_DATA,
        PACKET_TIMESTAMP_ERROR,
        PACKET_INLINE_MESSAGE,
        PACKET_TIMEOUT_ERROR,
        PACKET_SEQUENCE_ERROR,
        PACKET_EINTR
    };

#ifdef ERROR_INJECT_DROPPED_PACKETS
    int recvd_packets;
#endif

    /*******************************************************************
     * Get and process a single packet from the transport:
     * Receive a single packet at the given index.
     * Extract all the relevant info and store.
     * Check the info to determine the return code.
     ******************************************************************/
    UHD_INLINE packet_type get_and_process_single_packet(const size_t index,
        per_buffer_info_type& prev_buffer_info,
        per_buffer_info_type& curr_buffer_info,
        double timeout)
    {
        managed_recv_buffer::sptr& buff = curr_buffer_info.buff;
        per_buffer_info_type& info      = curr_buffer_info;
        while (1) {
            // get a single packet from the transport layer

            int get_buff_error_code = 0;
            buff = _props[index].get_buff(timeout, &get_buff_error_code);
            if (buff.get() == nullptr) {
                if(get_buff_error_code == EINTR) {
                    return PACKET_EINTR;
                } else {
                    return PACKET_TIMEOUT_ERROR;
                }
            } else {
            }

#ifdef ERROR_INJECT_DROPPED_PACKETS
            if (++recvd_packets > 1000) {
                recvd_packets = 0;
                buff.reset();
                buff = _props[index].get_buff(timeout);
                if (buff.get() == nullptr)
                    return PACKET_TIMEOUT_ERROR;
            }
#endif

            // bounds check before extract
            const size_t num_packet_words32 = buff->size() / sizeof(uint32_t);
            if (num_packet_words32 <= _header_offset_words32) {
                throw std::runtime_error("recv buffer smaller than vrt packet offset");
            }

            // extract packet info
            info.ifpi                    = {};
            info.ifpi.num_packet_words32 = num_packet_words32 - _header_offset_words32;
            info.vrt_hdr = buff->cast<const uint32_t*>() + _header_offset_words32;
            _vrt_unpacker(info.vrt_hdr, info.ifpi);
            info.time      = info.ifpi.tsf; // assumes has_tsf is true
            info.copy_buff = reinterpret_cast<const char*>(
                info.vrt_hdr + info.ifpi.num_header_words32);

            // handle flow control
            if (_props[index].handle_flowctrl) {
                if ((info.ifpi.packet_count % _props[index].fc_update_window) == 0) {
                    _props[index].handle_flowctrl(info.ifpi.packet_count);
                }
            }

            // handle flow control ack
            if (info.ifpi.fc_ack) {
                if (_props[index].handle_flowctrl_ack) {
                    _props[index].handle_flowctrl_ack(
                        reinterpret_cast<const uint32_t*>(info.copy_buff));
                }
                // Process the next packet
                buff.reset();
                info.copy_buff = nullptr;
                continue;
            }

            break;
        }

        //--------------------------------------------------------------
        //-- Determine return conditions:
        //-- The order of these checks is HOLY.
        //--------------------------------------------------------------

        // 1) check for inline IF message packets
        if (info.ifpi.packet_type != vrt::if_packet_info_t::PACKET_TYPE_DATA) {
            return PACKET_INLINE_MESSAGE;
        }

// 2) check for sequence errors
#ifndef SRPH_DONT_CHECK_SEQUENCE
        const size_t seq_mask =
            (info.ifpi.link_type == vrt::if_packet_info_t::LINK_TYPE_NONE) ? 0xf : 0xfff;
        const size_t expected_packet_count = _props[index].packet_count;
        _props[index].packet_count         = (info.ifpi.packet_count + 1) & seq_mask;
        // Skip checking sequence number if timestamp is 0
        if (expected_packet_count != info.ifpi.packet_count && !(info.time == 0 && info.ifpi.packet_count == 0)) {
            // UHD_LOGGER_INFO("STREAMER") << "expected: " << expected_packet_count << "
            // got: " << info.ifpi.packet_count;
            if (_props[index].handle_flowctrl) {
                // Always update flow control in this case, because we don't
                // know which packet was dropped and what state the upstream
                // flow control is in.
                _props[index].handle_flowctrl(info.ifpi.packet_count);
            }
            return PACKET_SEQUENCE_ERROR;
        }
#endif

        // 3) check for out of order timestamps
        // Skip check if timestamp and sequence number are 0 (timestamp and sequence number get reset when streaming on a trigger)
        if (info.ifpi.has_tsf && prev_buffer_info.time > info.time && !(info.time == 0 && info.ifpi.packet_count == 0)) {
            return PACKET_TIMESTAMP_ERROR;
        }

        // 4) otherwise the packet is normal!
        return PACKET_IF_DATA;
    }

    void _flush_all(double timeout)
    {
        get_prev_buffer_info().reset();
        get_curr_buffer_info().reset();
        get_next_buffer_info().reset();

        for (size_t i = 0; i < _props.size(); i++) {
            per_buffer_info_type prev_buffer_info, curr_buffer_info;
            prev_buffer_info.reset();
            curr_buffer_info.reset();
            while (true) {
                // receive a single packet from the transport
                try {
                    // call into get_and_process_single_packet()
                    // to make sure flow control is handled
                    if (get_and_process_single_packet(
                            i, prev_buffer_info, curr_buffer_info, timeout)
                        == PACKET_TIMEOUT_ERROR)
                        break;
                } catch (...) {
                }
                curr_buffer_info.buff.reset(); // Let my buffer go!
                prev_buffer_info = curr_buffer_info;
                curr_buffer_info.reset();
            }
        }
    }

    /*******************************************************************
     * Alignment check:
     * Check the received packet for alignment and mark accordingly.
     ******************************************************************/
    UHD_INLINE void alignment_check(const size_t index, buffers_info_type& info)
    {
        // if alignment time was not valid or if the sequence id is newer:
        //  use this index's time as the alignment time
        //  reset the indexes list and remove this index
        if (not info.alignment_time_valid or info[index].time > info.alignment_time) {
            info.alignment_time_valid = true;
            info.alignment_time       = info[index].time;
            info.indexes_todo.set();
            info.indexes_todo.reset(index);
            // release the other buffers
            for (size_t i = 0; i < info.size(); i++) {
                if (i != index) {
                    info[i].reset();
                }
            }
            info.data_bytes_to_copy = info[index].ifpi.num_payload_bytes;
            // reset start_of_burst and end_of_burst states
            info.metadata.start_of_burst = info[index].ifpi.sob;
            info.metadata.end_of_burst   = info[index].ifpi.eob;
        }

        // if the sequence id matches:
        //  remove this index from the list and continue
        else if (info[index].time == info.alignment_time) {
            info.indexes_todo.reset(index);
            // All channels should have sob set at the same time, so only
            // set start_of burst if all channels have sob set.
            info.metadata.start_of_burst &= info[index].ifpi.sob;
            // If any channel indicates eob, no more data will be received for
            // that channel so set end_of_burst for any eob.
            info.metadata.end_of_burst |= info[index].ifpi.eob;
        } else {
            // Not going to use this buffer, so release it
            info[index].reset();
        }

        // if the sequence id is older:
        //  continue with the same index to try again
        // else if (info[index].time < info.alignment_time)...
    }

    /*******************************************************************
     * Get aligned buffers:
     * Iterate through each index and try to accumulate aligned buffers.
     * Handle all of the edge cases like inline messages and errors.
     * The logic will throw out older packets until it finds a match.
     ******************************************************************/
    size_t oflow_count_this_line = 0;
    UHD_INLINE void get_aligned_buffs(double timeout)
    {
        get_prev_buffer_info()
            .reset(); // no longer need the previous info - reset it for future use

        increment_buffer_info(); // increment to next buffer

        buffers_info_type& prev_info = get_prev_buffer_info();
        buffers_info_type& curr_info = get_curr_buffer_info();
        buffers_info_type& next_info = get_next_buffer_info();

        curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;

        // Loop until we get a message of an aligned set of buffers:
        // - Receive a single packet and extract its info.
        // - Handle the packet type yielded by the receive.
        // - Check the timestamps for alignment conditions.
        size_t iterations = 0;
        while (curr_info.indexes_todo.any()) {
            // get the index to process for this iteration
            const size_t index = curr_info.indexes_todo.find_first();
            packet_type packet;

            // receive a single packet from the transport
            try {
                packet = get_and_process_single_packet(
                    index, prev_info[index], curr_info[index], timeout);
            }

            // handle the case where a bad header exists
            catch (const uhd::value_error& e) {
                UHD_LOGGER_ERROR("STREAMER")
                    << boost::format(
                           "The receive packet handler caught a value exception.\n%s")
                           % e.what();
                std::swap(curr_info, next_info); // save progress from curr -> next
                curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_BAD_PACKET;
                return;
            }

            switch (packet) {
                case PACKET_IF_DATA:
                    alignment_check(index, curr_info);
                    break;

                case PACKET_TIMESTAMP_ERROR:
                    // If the user changes the device time while streaming or without
                    // flushing, we can receive a packet that comes before the previous
                    // packet in time. This could cause the alignment logic to discard
                    // future received packets. Therefore, when this occurs, we reset the
                    // info to restart from scratch.
                    if (curr_info.alignment_time_valid
                        and curr_info.alignment_time != curr_info[index].time) {
                        curr_info.alignment_time_valid = false;
                    }
                    alignment_check(index, curr_info);
                    break;

                case PACKET_INLINE_MESSAGE:
                    std::swap(curr_info, next_info); // save progress from curr -> next
                    curr_info.metadata.has_time_spec = next_info[index].ifpi.has_tsf;
                    curr_info.metadata.time_spec =
                        time_spec_t::from_ticks(next_info[index].time, _tick_rate);
                    curr_info.metadata.error_code =
                        rx_metadata_t::error_code_t(get_context_code(
                            next_info[index].vrt_hdr, next_info[index].ifpi));
                    if (curr_info.metadata.error_code
                        == rx_metadata_t::ERROR_CODE_OVERFLOW) {
                        // Not sending flow control would cause timeouts due to source
                        // flow control locking up. Send first as the overrun handler may
                        // flush the receive buffers which could contain packets with
                        // sequence numbers after this packet's sequence number!
                        if (_props[index].handle_flowctrl) {
                            _props[index].handle_flowctrl(
                                next_info[index].ifpi.packet_count);
                        }

                        rx_metadata_t metadata = curr_info.metadata;
                        _props[index].handle_overflow();
                        curr_info.metadata = metadata;
                        UHD_LOG_FASTPATH("O");
                    }
                    next_info[index].buff.reset(); // No data, so release the buffer
                    next_info[index].copy_buff = nullptr;

                    return;

                case PACKET_TIMEOUT_ERROR:
                    std::swap(curr_info, next_info); // save progress from curr -> next
                    if (_props[index].handle_flowctrl) {
                        _props[index].handle_flowctrl(next_info[index].ifpi.packet_count);
                    }
                    curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
                    return;

                case PACKET_EINTR:
                    std::swap(curr_info, next_info); // save progress from curr -> next
                    if (_props[index].handle_flowctrl) {
                        _props[index].handle_flowctrl(next_info[index].ifpi.packet_count);
                    }
                    curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_EINTR;
                    return;

                case PACKET_SEQUENCE_ERROR:
                    alignment_check(index, curr_info);
                    std::swap(curr_info, next_info); // save progress from curr -> next
                    curr_info.metadata.has_time_spec = prev_info.metadata.has_time_spec;
                    curr_info.metadata.time_spec =
                        prev_info.metadata.time_spec
                        + time_spec_t::from_ticks(
                            prev_info[index].ifpi.num_payload_words32 * sizeof(uint32_t)
                                / _bytes_per_otw_item,
                            _samp_rate);
                    curr_info.metadata.out_of_sequence = true;
                    curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
                    if(oflow_count_this_line < 25) {
                        UHD_LOG_FASTPATH("D");
                        oflow_count_this_line++;
                    } else {
                        UHD_LOG_FASTPATH("D\n");
                        oflow_count_this_line = 0;
                    }
                    return;
            }

            // too many iterations: detect alignment failure
            if (iterations++ > _alignment_failure_threshold) {
                UHD_LOGGER_ERROR("STREAMER")
                    << boost::format(
                           "The receive packet handler failed to time-align packets.\n"
                           "%u received packets were processed by the handler.\n"
                           "However, a timestamp match could not be determined.\n")
                           % iterations
                    << std::endl;
                std::swap(curr_info, next_info); // save progress from curr -> next
                curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_ALIGNMENT;
                _props[index].handle_overflow();
                return;
            }
        }

        // set the metadata from the buffer information at index zero
        curr_info.metadata.has_time_spec = curr_info[0].ifpi.has_tsf;
        curr_info.metadata.time_spec =
            time_spec_t::from_ticks(curr_info[0].time, _tick_rate);
        curr_info.metadata.more_fragments  = false;
        curr_info.metadata.fragment_offset = 0;
        curr_info.metadata.error_code      = rx_metadata_t::ERROR_CODE_NONE;
    }

    /*******************************************************************
     * Receive a single packet on all channels
     * Handles fragmentation, messages, errors, and copy-conversion.
     * When no fragments are available, call the get aligned buffers.
     * Then copy-convert available data into the user's IO buffers.
     ******************************************************************/
    UHD_INLINE size_t recv_one_packet(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const size_t buffer_offset_bytes = 0)
    {
        // get the next buffer if the current one has expired
        if (get_curr_buffer_info().data_bytes_to_copy == 0) {
            // perform receive with alignment logic
            get_aligned_buffs(timeout);
        }

        buffers_info_type& info = get_curr_buffer_info();
        metadata                = info.metadata;

        // interpolate the time spec (useful when this is a fragment)
        metadata.time_spec +=
            time_spec_t::from_ticks(info.fragment_offset_in_samps, _samp_rate);

        // extract the number of samples available to copy
        const size_t nsamps_available = info.data_bytes_to_copy / _bytes_per_otw_item;
        const size_t nsamps_to_copy =
            std::min(nsamps_per_buff * _num_outputs, nsamps_available);
        const size_t bytes_to_copy              = nsamps_to_copy * _bytes_per_otw_item;
        const size_t nsamps_to_copy_per_io_buff = nsamps_to_copy / _num_outputs;

        // setup the data to share with converter threads
        _convert_nsamps              = nsamps_to_copy_per_io_buff;
        _convert_buffs               = &buffs;
        _convert_buffer_offset_bytes = buffer_offset_bytes;
        _convert_bytes_to_copy       = bytes_to_copy;

        // perform N channels of conversion
        for (size_t i = 0; i < this->size(); i++) {
            convert_to_out_buff(i);
        }

        // update the copy buffer's availability
        info.data_bytes_to_copy -= bytes_to_copy;

        // setup the fragment flags and offset
        metadata.more_fragments  = info.data_bytes_to_copy != 0;
        metadata.fragment_offset = info.fragment_offset_in_samps;
        info.fragment_offset_in_samps += nsamps_to_copy; // set for next call

        return nsamps_to_copy_per_io_buff;
    }

    /*! Run the conversion from the internal buffers to the user's output
     *  buffer.
     *
     * - Calls the converter
     * - Releases internal data buffers
     * - Updates read/write pointers
     */
    inline void convert_to_out_buff(const size_t index)
    {
        // shortcut references to local data structures
        buffers_info_type& buff_info         = get_curr_buffer_info();
        per_buffer_info_type& info           = buff_info[index];
        const rx_streamer::buffs_type& buffs = *_convert_buffs;

        // fill IO buffs with pointers into the output buffer
        void* io_buffs[4 /*max interleave*/];
        for (size_t i = 0; i < _num_outputs; i++) {
            char* b     = reinterpret_cast<char*>(buffs[index * _num_outputs + i]);
            io_buffs[i] = b + _convert_buffer_offset_bytes;
        }
        const ref_vector<void*> out_buffs(io_buffs, _num_outputs);

        // perform the conversion operation
        _converter->conv(info.copy_buff, out_buffs, _convert_nsamps);

        // advance the pointer for the source buffer
        info.copy_buff += _convert_bytes_to_copy;

        // release the buffer if fully consumed
        if (buff_info.data_bytes_to_copy == _convert_bytes_to_copy) {
            info.buff.reset(); // effectively a release
        }
    }

    //! Shared variables for the worker threads
    size_t _convert_nsamps;
    const rx_streamer::buffs_type* _convert_buffs;
    size_t _convert_buffer_offset_bytes;
    size_t _convert_bytes_to_copy;
};

class recv_packet_streamer : public recv_packet_handler, public rx_streamer
{
public:
    recv_packet_streamer(const size_t max_num_samps)
    {
        _max_num_samps = max_num_samps;
    }

    size_t get_num_channels(void) const override
    {
        return this->size();
    }

    size_t get_max_num_samps(void) const override
    {
        return _max_num_samps;
    }

    size_t recv(const rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet) override
    {
        return recv_packet_handler::recv(
            buffs, nsamps_per_buff, metadata, timeout, one_packet);
    }

    void issue_stream_cmd(const stream_cmd_t& stream_cmd) override
    {
        return recv_packet_handler::issue_stream_cmd(stream_cmd);
    }

    // Post an action to the input edge.
    void post_input_action(
        const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t) override
    {
        throw uhd::not_implemented_error("post_input_action is not implemented here!");
    }

private:
    size_t _max_num_samps;
};

}}} // namespace uhd::transport::sph

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_HPP */
