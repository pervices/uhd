//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_HPP
#define INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_HPP

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

#ifdef UHD_TXRX_DEBUG_PRINTS
// Included for debugging
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <map>
#include <fstream>
#endif

//#define FLOW_CONTROL_DEBUG

namespace uhd {
namespace transport {
namespace sph {

#define MAX_CHANNELS 64

/***********************************************************************
 * Super send packet handler
 *
 * A send packet handler represents a group of channels.
 * The channel group shares a common sample rate.
 * All channels are sent in unison in send().
 **********************************************************************/

// static bool time_comp(std::chrono::duration <double, std::micro> a, std::chrono::duration <double, std::micro> b) {
//     return (a.count() > b.count());
// }

class send_packet_handler{
public:
    typedef std::function<managed_send_buffer::sptr(double)> get_buff_type;
    typedef std::function<void(size_t)> update_fc_send_count_type;
    typedef std::function<bool(double)> check_flow_control_type;
    typedef std::function<void(void)> post_send_cb_type;
    typedef std::function<bool(uhd::async_metadata_t &, const double)> async_receiver_type;
    typedef void(*vrt_packer_type)(uint32_t *, vrt::if_packet_info_t &);
    //typedef std::function<void(uint32_t *, vrt::if_packet_info_t &)> vrt_packer_type;

    /*!
     * Make a new packet handler for send
     * \param buffer_size size of the buffer on the unit
     */
    send_packet_handler(const size_t buffer_size):
        _buffer_size(buffer_size),
        _next_packet_seq(0),
        _cached_metadata(false)

    {
        this->channel_per_conversion_thread = 8;
        this->set_enable_trailer(true);
        this->samps_per_buffer = 1;
        // this->resize(size);
        multi_msb_buffs.resize(1);
        this->end_time = std::chrono::high_resolution_clock::now();
        this->start_time = std::chrono::high_resolution_clock::now();
    }

    ~send_packet_handler(void){
        // Destroy the multi-threaded convert_to_in_buff
        if (this->size() != 0) {
            std::unique_lock<std::mutex> guard(this->conversion_mutex);
            this->conversion_terminate = true;
            for (size_t i = 0; i < this->size(); i++) {
                this->conversion_ready[i] = true;
            }
            guard.unlock();
            this->conversion_cv.notify_all();

            for (size_t i = 1; i < this->conversion_threads.size(); i++) {
                this->conversion_threads[i].join();
            }
            this->multi_msb_buffs.clear();
        }

        // std::chrono::duration <double, std::micro> sum {0};
        // int i = 0;
        // std::cout << "First 400 sample send times:\n";
        // for (auto send_time : elapsed) {
        //     sum += send_time;
        //     if (i < 400) {
        //         std::cout << send_time.count() << "  ---  ";
        //     }
        //     i++;
        // }
        // std::cout << "\n\n\n";
        // std::cout << "Minimum Times sorted:\n";
        // std::sort(elapsed.begin(), elapsed.end());
        // i = 0;
        // for (auto send_time : elapsed) {
        //     if (i < 1000) {
        //         std::cout << send_time.count() << "  ---  ";
        //     } else {
        //         break;
        //     }
        //     i++;
        // }
        // std::cout << "\n\n\n";
        // std::cout << "Maximum Times sorted:\n";
        // std::sort(elapsed.begin(), elapsed.end(), time_comp);
        // i = 0;
        // for (auto send_time : elapsed) {
        //     if (i < 1000) {
        //         std::cout << send_time.count() << "  ---  ";
        //     } else {
        //         break;
        //     }
        //     i++;
        // }
        // std::cout << "\n";
        // std::cout << "Average elapsed time: " << (sum/elapsed.size()).count() << "\n";
        /* NOP */
    }

    //! Resize the number of transport channels
    void resize(const size_t size){
        if (this->size() == size) return;

        // Handle the multi-threaded convert_to_in_buff
        //   Destroy current threads
        if (this->size() != 0) {
            std::unique_lock<std::mutex> guard(this->conversion_mutex);
            this->conversion_terminate = true;
            for (size_t i = 0; i < this->size(); i++) {
                this->conversion_ready[i] = true;
            }
            guard.unlock();
            this->conversion_cv.notify_all();

            for (size_t i = 1; i < this->conversion_threads.size(); i++) {
                this->conversion_threads[i].join();
            }
        }

        // Decide which indices the conversion threads will handle
        if (channel_per_conversion_thread < size) {
            this->conversion_threads.resize(std::ceil(((double)size)/channel_per_conversion_thread));
        } else {
            this->conversion_threads.resize(1);
        }

        thread_indices.resize(this->conversion_threads.size());
        for (size_t i = 0; i < thread_indices.size(); i++) {
            for (size_t j = 0; j < channel_per_conversion_thread; j++) {
                size_t thread_index = (i*channel_per_conversion_thread)+j;
                if (thread_index >= size) {
                    break;
                }
                thread_indices[i].push_back(thread_index);
            }
        }

        // Assign synchronization defaults
        this->conversion_done.resize(size);
        this->conversion_ready.resize(size);
        this->conversion_terminate = false;

        // Create new threads
        for (size_t i = 1; i < this->conversion_threads.size(); i++) {
            this->conversion_threads[i] = std::thread(&send_packet_handler::send_multiple_packets_threaded, this, thread_indices[i]);
        }

        for (size_t i = 0; i < size; i++) {
            this->conversion_done[i] = false;
            this->conversion_ready[i] = false;
        }

        multi_msb_buffs.resize(size);
        _props.resize(size);
        static const uint64_t zero = 0;
        _zero_buffs.resize(size, &zero);
    }

    //! Get the channel width of this handler
    size_t size(void) const{
        return _props.size();
    }

    //! Setup the vrt packer function and offset
    void set_vrt_packer(const vrt_packer_type &vrt_packer, const size_t header_offset_words32 = 0){
        _vrt_packer = vrt_packer;
        _header_offset_words32 = header_offset_words32;
    }

    //! Set the stream ID for a specific channel (or no SID)
    void set_xport_chan_sid(const size_t xport_chan, const bool has_sid, const uint32_t sid = 0){
        _props.at(xport_chan).has_sid = has_sid;
        _props.at(xport_chan).sid = sid;
    }

    void set_enable_trailer(const bool enable)
    {
        _has_tlr = enable;
    }

    //! Set the rate of ticks per second
    void set_tick_rate(const double rate){
        _tick_rate = rate;
    }

    //! Set the rate of samples per second
    void set_samp_rate(const double rate){
        _samp_rate = rate;
    }

    /*!
     * Set the function to get a managed buffer.
     * \param xport_chan which transport channel
     * \param get_buff the getter function
     */
    void set_xport_chan_get_buff(const size_t xport_chan, const get_buff_type &get_buff){
        _props.at(xport_chan).get_buff = get_buff;
    }

    void set_xport_chan_update_fc_send_size(const size_t xport_chan, const update_fc_send_count_type &update_fc_send_count){
        _props.at(xport_chan).update_fc_send_count = update_fc_send_count;
    }

    void set_xport_chan_check_flow_control(const size_t xport_chan, const check_flow_control_type &check_flow_control){
        _props.at(xport_chan).check_flow_control = check_flow_control;
    }
    /*!
     * Set the callback function for post-send.
     * \param xport_chan which transport channel
     * \param cb post-send callback
     */
    void set_xport_chan_post_send_cb(const size_t xport_chan, const post_send_cb_type &cb){
        _props.at(xport_chan).go_postal = cb;
    }

    //! Set the conversion routine for all channels
    void set_converter(const uhd::convert::id_type &id){
        _num_inputs = id.num_inputs;
        _converter = uhd::convert::get_converter(id)();
        this->set_scale_factor(32767.); //update after setting converter
        _bytes_per_otw_item = uhd::convert::get_bytes_per_item(id.output_format);
        _bytes_per_cpu_item = uhd::convert::get_bytes_per_item(id.input_format);
    }

    /*!
     * Set the maximum number of samples per host packet.
     * Ex: A USRP1 in dual channel mode would be half.
     * \param num_samps the maximum samples in a packet
     */
    void set_max_samples_per_packet(const size_t num_samps){
        _max_samples_per_packet = num_samps;
    }

    //! Set the scale factor used in float conversion
    void set_scale_factor(const double scale_factor){
        _converter->set_scalar(scale_factor);
    }

    //xxx:DMCL Function for getting number of samples to send
    // Used for predictive modeling in Per Vices
    size_t get_nsamps(){
        return _convert_nsamps;
    }

    //! Set the callback to get async messages
    void set_async_receiver(const async_receiver_type &async_receiver)
    {
        _async_receiver = async_receiver;
    }

    //! Overload call to get async metadata
    bool recv_async_msg(
        uhd::async_metadata_t &async_metadata, double timeout = 0.1
    ){
        if (_async_receiver) return _async_receiver(async_metadata, timeout);
        std::this_thread::sleep_for(std::chrono::microseconds(long(timeout*1e6)));
        return false;
    }

    // Sends an end of burst packet with 1 sample (since Crimson/Cyan can't handle 0 length packets)
    void send_eob_packet(const uhd::tx_streamer::buffs_type &buffs, vrt::if_packet_info_t if_packet_info, const double timeout) {
        if_packet_info.eob = true;
        if_packet_info.sob = false;
        // EOB cannot have timestamps
        if_packet_info.has_tsf = false;
        static const uint64_t zero = 0;
        _zero_buffs.resize(buffs.size(), &zero);
        // Prepared the packets
        send_one_packet(_zero_buffs, 1, if_packet_info, timeout, 0);
        this->samps_per_buffer = 1;
        // Sends the packets
        send_multiple_packets();
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
    ){
        uhd::tx_metadata_t metadata = metadata_;
        //translate the metadata to vrt if packet info
        vrt::if_packet_info_t if_packet_info;
        if_packet_info.packet_type = vrt::if_packet_info_t::PACKET_TYPE_DATA;
        //if_packet_info.has_sid = false; //set per channel
        if_packet_info.has_cid = false;
        if_packet_info.has_tlr = _has_tlr;
        if_packet_info.has_tsi = false;
        if_packet_info.has_tsf = metadata.has_time_spec;
        if_packet_info.tsf     = metadata.time_spec.to_ticks(_tick_rate);
        if_packet_info.sob     = metadata.start_of_burst;
        // End of burst should be an empty packet after all the data packets
        // = false makes all the data packets get sent without EOB set, eob_requested tracks if an empty EOB packet should be sent at the end
        bool eob_requested = metadata.end_of_burst;
        if_packet_info.eob     = false;
        if_packet_info.fc_ack  = false; //This is a data packet

        const size_t samp_rate_per_ch = metadata.aggregate_samp_rate/this->size();

        /*
         * Metadata is cached when we get a send requesting a start of burst with no samples.
         * It is applied here on the next call to send() that actually has samples to send.
         */
        if (_cached_metadata && nsamps_per_buff != 0)
        {
            // If the new metada has a time_spec, do not use the cached time_spec.
            if (!metadata.has_time_spec)
            {
                if_packet_info.has_tsf = _metadata_cache.has_time_spec;
                if_packet_info.tsf     = _metadata_cache.time_spec.to_ticks(_tick_rate);
            }
            if_packet_info.sob     = _metadata_cache.start_of_burst;
            // EOB packets need to be empty, eob_requested is used to track if an empty EOB packet should be sent after sneding all data packets
            // if_packet_info.sob should be left false until its time to send that final empty packet
            eob_requested = _metadata_cache.end_of_burst;
            _cached_metadata = false;
        }

        if (nsamps_per_buff <= _max_samples_per_packet){

            //TODO remove this code when sample counts of zero are supported by hardware
            #ifndef SSPH_DONT_PAD_TO_ONE
                static const uint64_t zero = 0;
                _zero_buffs.resize(buffs.size(), &zero);

                if (nsamps_per_buff == 0)
                {
                    // if this is a start of a burst and there are no samples
                    if (metadata.start_of_burst)
                    {
                        // cache metadata and apply on the next send()
                        _metadata_cache = metadata;
                        _cached_metadata = true;
                        return 0;
                    } else if (eob_requested)
                    {
                        send_eob_packet(buffs, if_packet_info, timeout);
                        return 0;
                    }
                    else
                    {
                        // send requests with no samples are handled here (such as end of burst)
                        send_one_packet(_zero_buffs, 1, if_packet_info, timeout, 0);
                        this->samps_per_buffer = 1;
                        send_multiple_packets();
                        return 0;
                    }
                }
            #endif

            size_t nsamps_sent = send_one_packet(buffs, nsamps_per_buff, if_packet_info, timeout, 0);
            this->samps_per_buffer = nsamps_per_buff;
            send_multiple_packets();
            if(eob_requested) {
                send_eob_packet(buffs, if_packet_info, timeout);
            }
#ifdef UHD_TXRX_DEBUG_PRINTS
			dbg_print_send(nsamps_per_buff, nsamps_sent, metadata, timeout);
#endif
			return nsamps_sent;
        }

        size_t total_num_samps_sent = 0;
        size_t prev_total_num_samps_sent = 0;

        //false until final fragment
        if_packet_info.eob = false;

        const size_t num_fragments = (nsamps_per_buff-1)/_max_samples_per_packet;
        const size_t final_length = ((nsamps_per_buff-1)%_max_samples_per_packet)+1;
        const double fc_buff_size_limit_percentage = (samp_rate_per_ch > 10000*_buffer_size) ? 0.05 :
                                                     (samp_rate_per_ch > 10000*_buffer_size/2) ? 0.10 : 0.15;
        const size_t flow_control_limit = _buffer_size*fc_buff_size_limit_percentage;
        const size_t flow_control_passes = ceil(nsamps_per_buff/flow_control_limit);

        //loop through the following fragment indexes
        size_t i = 0;
        
        while ( i < num_fragments) {

            //send a fragment with the helper function
            const size_t num_samps_sent = send_one_packet(buffs, _max_samples_per_packet, if_packet_info, timeout, total_num_samps_sent*_bytes_per_cpu_item);
            total_num_samps_sent += num_samps_sent;
            if (num_samps_sent == 0) {
                return total_num_samps_sent;
            }

            //setup metadata for the next fragment
            const time_spec_t time_spec = metadata.time_spec + time_spec_t::from_ticks(total_num_samps_sent, _samp_rate);
            if(i > 0) {
                metadata.start_of_burst = false;
            }

            if_packet_info.tsf = time_spec.to_ticks(_tick_rate);
            if_packet_info.sob = false;
            
            if ((flow_control_passes > 1 && i > 0 && i%flow_control_passes == 0)
                || (i == num_fragments-1))
            {
                this->samps_per_buffer = total_num_samps_sent - prev_total_num_samps_sent;
                prev_total_num_samps_sent = total_num_samps_sent;
                send_multiple_packets();
            }
            i++;

        }
        
        // Cyan/Crimson requires that end of burst be in an empty packet.
        // if final_length sends the final samples from this call (if applicable), then if metatdata.end_of_burst sends the empty end of burst packet
        size_t nsamps_sent = 0;
        if(final_length) {
            //send the final fragment with the helper function
             nsamps_sent = total_num_samps_sent + send_one_packet(buffs, final_length, if_packet_info, timeout, total_num_samps_sent * _bytes_per_cpu_item);

            this->samps_per_buffer = nsamps_sent - total_num_samps_sent;
            total_num_samps_sent += nsamps_sent;

            send_multiple_packets();
        }
        if(eob_requested) {
            send_eob_packet(buffs, if_packet_info, timeout);
        }

#ifdef UHD_TXRX_DEBUG_PRINTS
		dbg_print_send(nsamps_per_buff, nsamps_sent, metadata, timeout);
#endif
		return nsamps_sent;
    }

protected:
    double _tick_rate, _samp_rate;
    struct xport_chan_props_type{
        xport_chan_props_type(void):has_sid(false),sid(0){}
        get_buff_type get_buff;
        update_fc_send_count_type update_fc_send_count;
        check_flow_control_type check_flow_control;
        post_send_cb_type go_postal;
        bool has_sid;
        uint32_t sid;
        managed_send_buffer::sptr buff;
    };
    std::vector<xport_chan_props_type> _props;

private:

    // apparatus for multi-threaded execution of convert_to_in_buff
    std::vector<std::thread> conversion_threads;
    std::mutex conversion_mutex;
    std::condition_variable conversion_cv;
    std::vector<bool> conversion_ready;
    std::vector<bool> conversion_done;
    std::vector< std::vector<size_t> > thread_indices;
    bool conversion_terminate;
    size_t channel_per_conversion_thread;
    std::vector< std::chrono::duration<double, std::micro> > elapsed;
    size_t samps_per_buffer;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> last_send = std::chrono::high_resolution_clock::now();
    size_t _buffer_size;

    vrt_packer_type _vrt_packer;
    size_t _header_offset_words32;

    // This structure will hold a vector of buffers of data to be sent to a single socket
    // using sendmmsg system call.
    struct multi_msb_type {
        std::vector<const void *> data_buffs;
        std::vector<size_t> data_buff_length;
        std::vector<void *> vrt_headers;
        std::vector<size_t> vrt_header_length;
        int sock_fd;
    };
    std::vector<multi_msb_type> multi_msb_buffs;

    size_t _num_inputs;
    size_t _bytes_per_otw_item; //used in conversion
    size_t _bytes_per_cpu_item; //used in conversion
    uhd::convert::converter::sptr _converter; //used in conversion
    size_t _max_samples_per_packet;
    std::vector<const void *> _zero_buffs;
    size_t _next_packet_seq;
    bool _has_tlr;
    async_receiver_type _async_receiver;
    bool _cached_metadata;
    uhd::tx_metadata_t _metadata_cache;

#ifdef UHD_TXRX_DEBUG_PRINTS
    struct dbg_send_stat_t {
        dbg_send_stat_t(long wc, size_t nspb, size_t nss, uhd::tx_metadata_t md, double to, double rate):
            wallclock(wc), nsamps_per_buff(nspb), nsamps_sent(nss), metadata(md), timeout(to), samp_rate(rate)
        {}
        long wallclock;
        size_t nsamps_per_buff;
        size_t nsamps_sent;
        uhd::tx_metadata_t metadata;
        double timeout;
        double samp_rate;
        // Create a formatted print line for all the info gathered in this struct.
        std::string print_line() {
            boost::format fmt("send,%ld,%f,%i,%i,%s,%s,%s,%ld");
            fmt % wallclock;
            fmt % timeout % (int)nsamps_per_buff % (int) nsamps_sent;
            fmt % (metadata.start_of_burst ? "true":"false") % (metadata.end_of_burst ? "true":"false");
            fmt % (metadata.has_time_spec ? "true":"false") % metadata.time_spec.to_ticks(samp_rate);
            return fmt.str();
        }
    };

    void dbg_print_send(size_t nsamps_per_buff, size_t nsamps_sent,
            const uhd::tx_metadata_t &metadata, const double timeout,
            bool dbg_print_directly = true)
    {
        dbg_send_stat_t data(boost::get_system_time().time_of_day().total_microseconds(),
            nsamps_per_buff,
            nsamps_sent,
            metadata,
            timeout,
            _samp_rate
        );
        if(dbg_print_directly){
            dbg_print_err(data.print_line());
        }
    }
    void dbg_print_err(std::string msg) {
        msg = "super_send_packet_handler," + msg;
        fprintf(stderr, "%s\n", msg.c_str());
    }


#endif
    /*******************************************************************
     * Send multiple packets at once:
     ******************************************************************/
    UHD_INLINE size_t send_multiple_packets() {
        //perform N channels of conversion
        // Wake up the worker threads (send_multiple_packets_threaded) and wait for their completion
        if (this->conversion_threads.size() > 1) {
            std::unique_lock<std::mutex> guard(this->conversion_mutex);
            for (size_t i = 0; i < this->size(); i++) {
                conversion_done[i] = false;
                conversion_ready[i] = true;
            }
            guard.unlock();
            conversion_cv.notify_all();
        }

        send_multiple_packets_sequential(this->thread_indices[0]);

        // Wait for worker threads to finish their work
        if (this->conversion_threads.size() > 1) {
            for (size_t i = thread_indices[1].front(); i < this->size(); i++) {
                while (!conversion_done[i]) {
                    // Sleep for 10 us intervals while checking whether the worker threads are done
                    // TODO: verify that the sleep duration is efficient.
                    std::this_thread::sleep_for(std::chrono::nanoseconds(1000));
                }
            }
        }

        // Clears the buffer of packets that were sent
        for (auto &multi_msb : this->multi_msb_buffs) {
            multi_msb.data_buffs.clear();
            multi_msb.data_buff_length.clear();
            multi_msb.vrt_headers.clear();
            multi_msb.vrt_header_length.clear();
        }
        return 0;
    }

    UHD_INLINE size_t send_multiple_packets_threaded(const std::vector<size_t> channels) {
        const double timeout = 0;
        while (true) {
            // Wait until the controlling thread gives the green light
            std::unique_lock<std::mutex> guard(conversion_mutex);
            conversion_cv.wait(guard, [this, channels]{return this->conversion_ready[channels[0]] == true;});

            if (conversion_terminate) {
                break;
            }

            // data structure to record which channels we have sent the data for
            // char channels_serviced[MAX_CHANNELS];
            // memset(channels_serviced, 0, MAX_CHANNELS*sizeof(char));
            std::array<char, MAX_CHANNELS> channels_serviced;
            for (auto &chan : channels_serviced) {
                chan = 0;
            }
            size_t total_channels_to_service = channels.size();
            size_t total_channels_serviced = 0;
            

            while(total_channels_serviced < total_channels_to_service) {
                for (const auto & chan: channels) {
                    if (channels_serviced[chan] == 0) {
                        if (!(_props.at(chan).check_flow_control(timeout))) {
                            // The time to send for this channel has not reached.
                            continue;
                        }
                        // It's time to send for this channel, mark it as serviced.
                        channels_serviced[chan] = 1;
                        total_channels_serviced++;
                    } else {
                        // We've already sent the data for this channel; move on.
                        continue;
                    }
                    const auto multi_msb = multi_msb_buffs.at(chan);
                    int number_of_messages = multi_msb.data_buffs.size();
                    mmsghdr msg[number_of_messages];
                    // In case we want to use scatter/gather capability and
                    // send Vita header and data in two separate buffers.
                    iovec iov[2*number_of_messages];

                    int i = 0;
                    for (auto data_buff : multi_msb.data_buffs) {
                        // In case of scatter/gather, the VRT header should be in the first iov
                        // and the data should be in the second iov
                        if (_converter->bypass_conversion_and_use_scatter_gather()) {
                            iov[(i*2)].iov_base = ((void *)multi_msb.vrt_headers.at(i));
                            iov[(i*2)].iov_len = multi_msb.vrt_header_length.at(i);
                            iov[(i*2)+1].iov_base = ((void *)data_buff);
                            iov[(i*2)+1].iov_len = multi_msb.data_buff_length.at(i);
                            msg[i].msg_hdr.msg_iov = &iov[i*2];
                            msg[i].msg_hdr.msg_iovlen = 2;
                        } else {
                            iov[i].iov_base = ((void*)data_buff);
                            iov[i].iov_len = multi_msb.data_buff_length.at(i);
                            msg[i].msg_hdr.msg_iov = &iov[i];
                            msg[i].msg_hdr.msg_iovlen = 1;
                        }
                        msg[i].msg_hdr.msg_name = NULL;
                        msg[i].msg_hdr.msg_namelen = 0;
                        msg[i].msg_hdr.msg_control = NULL;
                        msg[i].msg_hdr.msg_controllen = 0;

                        i++;
                    }
                    
                    int retval = sendmmsg(multi_msb.sock_fd, msg, number_of_messages, 0);
                    
                    if (retval == -1) {
                        std::cout << "XXX: chan " << chan << " sendmmsg failed : " << errno << " : " <<  std::strerror(errno) << "\n";
                        std::cout << "XXX: Must implement retry code!\n";
                    }

                    // for (auto buff : multi_msb.buffs) {
                    //     // Efectively a release
                    //     buff.reset();
                    // }
                    _props.at(chan).update_fc_send_count(this->samps_per_buffer);

                    // Notify the calling thread that we're finished with our work.
                    this->conversion_ready[chan] = false;
                    this->conversion_done[chan] = true;
                }
            }
        }

        return 0;
    }

    UHD_INLINE size_t send_multiple_packets_sequential(const std::vector<size_t> channels) {
        const double timeout = 0.1;

        // data structure to record which channels we have sent the data for
        // char channels_serviced[MAX_CHANNELS];
        // memset(channels_serviced, 0, MAX_CHANNELS*sizeof(char));
        std::array<char, MAX_CHANNELS> channels_serviced;
        for (auto &chan : channels_serviced) {
            chan = 0;
        }
        size_t total_channels_to_service = channels.size();
        size_t total_channels_serviced = 0;
        
        while (total_channels_serviced < total_channels_to_service) {
            for (const auto & chan: channels) {
                if (channels_serviced[chan] == 0) {
                    if (!(_props.at(chan).check_flow_control(timeout))) {
//  #ifdef FLOW_CONTROL_DEBUG
//                          std::cout << "Not time to send yet" << std::endl;
//  #endif
                        // The time to send for this channel has not reached.
                        continue;
                    } else {
#ifdef FLOW_CONTROL_DEBUG
                        std::cout << "Time to send" << std::endl;
#endif
                    }
                                        
                    // It's time to send for this channel, mark it as serviced.
                    channels_serviced[chan] = 1;
                    total_channels_serviced++;
                } else {
                    // We've already sent the data for this channel; move on.
                    continue;
                }
                
                const auto multi_msb = multi_msb_buffs.at(chan);
                int number_of_messages = multi_msb.data_buffs.size();
                mmsghdr msg[number_of_messages];
                // In case we want to use scatter/gather capability and
                // send Vita header and data in two separate buffers.
                iovec iov[2*number_of_messages];

                int i = 0;
                for (auto data_buff : multi_msb.data_buffs) {
                    // In case of scatter/gather, the VRT header should be in the first iov
                    // and the data should be in the second iov
                    if (_converter->bypass_conversion_and_use_scatter_gather()) {
                        iov[(i*2)].iov_base = (void *)multi_msb.vrt_headers.at(i);
                        iov[(i*2)].iov_len = multi_msb.vrt_header_length.at(i);
                        iov[(i*2)+1].iov_base = (void *)data_buff;
                        iov[(i*2)+1].iov_len = multi_msb.data_buff_length.at(i);
                        msg[i].msg_hdr.msg_iov = &iov[i*2];
                        msg[i].msg_hdr.msg_iovlen = 2;
                    } else {
                        iov[i].iov_base = (void *)data_buff;
                        iov[i].iov_len = multi_msb.data_buff_length.at(i);
                        msg[i].msg_hdr.msg_iov = &iov[i];
                        msg[i].msg_hdr.msg_iovlen = 1;
                    }
                    msg[i].msg_hdr.msg_name = NULL;
                    msg[i].msg_hdr.msg_namelen = 0;
                    msg[i].msg_hdr.msg_control = NULL;
                    msg[i].msg_hdr.msg_controllen = 0;

                    i++;
                }

                int retval = sendmmsg(multi_msb.sock_fd, msg, number_of_messages, 0);
                
                if (retval == -1) {
                    std::cout << "XXX: chan " << chan << " sendmmsg failed : " << errno << " : " <<  std::strerror(errno) << "\n";
                    std::cout << "XXX: Must implement retry code!\n";
                }

                // for (auto buff : multi_msb.buffs) {
                //     // Efectively a release
                //     buff.reset();
                // }
                _props.at(chan).update_fc_send_count(this->samps_per_buffer);
            }
        }

        return 0;
    }

    /*******************************************************************
     * Probably used to send a single packet
     * Doesn't actually send anything now
     * Probably involves in preparing packets for the function that is actually a part of the send process (send_multiple_packets)
     * TODO: refactor the send process to be easier to follow
     ******************************************************************/
    UHD_INLINE size_t send_one_packet(
        const uhd::tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        vrt::if_packet_info_t &if_packet_info,
        const double timeout,
        const size_t buffer_offset_bytes = 0
    ){
        //load the rest of the if_packet_info in here
        if_packet_info.num_payload_bytes = nsamps_per_buff*_num_inputs*_bytes_per_otw_item;
        if_packet_info.num_payload_words32 = (if_packet_info.num_payload_bytes + 3/*round up*/)/sizeof(uint32_t);
        if_packet_info.packet_count = _next_packet_seq;

        //xxx:DMCL Update class member here so that the get_buffs routine knows
        // how many samples we are sending.
        // Used for predictive modeling in Per Vices
        _convert_nsamps = nsamps_per_buff;

        // get a buffer for each channel or timeout
        for (xport_chan_props_type& props : _props) {
            if (not props.buff)
                props.buff = props.get_buff(timeout);
            if (not props.buff)
                return 0; // timeout
        }
        

        //setup the data to share with converter threads
        _convert_buffs = &buffs;
        _convert_buffer_offset_bytes = buffer_offset_bytes;
        _convert_if_packet_info = &if_packet_info;

        //perform N channels of conversion
        for (size_t i = 0; i < this->size(); i++) {
            convert_to_in_buff(i);
        }
        
        _next_packet_seq++; //increment sequence after commits
        
        return nsamps_per_buff;
    }

    /*! Run the conversion from the user's buffers to the internal buffer.
     *
     * - Calls the converter
     * - Releases internal data buffers
     * - Updates read/write pointers
     */
    UHD_INLINE void convert_to_in_buff(const size_t index)
    {

        //shortcut references to local data structures
        managed_send_buffer::sptr &buff = _props[index].buff;
        vrt::if_packet_info_t if_packet_info = *_convert_if_packet_info;
        const tx_streamer::buffs_type &buffs = *_convert_buffs;

        //fill IO buffs with pointers into the output buffer
        const void *io_buffs[4/*max interleave*/];
        for (size_t i = 0; i < _num_inputs; i++){
            const char *b = reinterpret_cast<const char *>(buffs[index*_num_inputs + i]);
            io_buffs[i] = b + _convert_buffer_offset_bytes;
        }
        const ref_vector<const void *> in_buffs(io_buffs, _num_inputs);

        //pack metadata into a vrt header
        uint32_t *otw_mem = buff->cast<uint32_t *>() + _header_offset_words32;
        if_packet_info.has_sid = _props[index].has_sid;
        if_packet_info.sid = _props[index].sid;

        _vrt_packer(otw_mem, if_packet_info);
        uint32_t *vrt_header = otw_mem;
        otw_mem += if_packet_info.num_header_words32;

        if (_converter->bypass_conversion_and_use_scatter_gather()) {
            // Add buffer to the array to be sent using sendmmsg
            multi_msb_buffs[index].data_buffs.push_back(reinterpret_cast<const void *>(io_buffs[0]));
            multi_msb_buffs[index].data_buff_length.push_back(if_packet_info.num_payload_words32*sizeof(uint32_t));
            multi_msb_buffs[index].sock_fd = buff->get_socket();;
            multi_msb_buffs[index].vrt_headers.push_back(vrt_header);
            multi_msb_buffs[index].vrt_header_length.push_back(if_packet_info.num_header_words32*sizeof(uint32_t));
        } else {
            const size_t num_vita_words32 = _header_offset_words32+if_packet_info.num_packet_words32;
            //perform the conversion operation
            _converter->conv(in_buffs, otw_mem, _convert_nsamps);
            multi_msb_buffs[index].data_buffs.push_back(buff->cast<const void *>());
            multi_msb_buffs[index].data_buff_length.push_back(num_vita_words32*sizeof(uint32_t));
            multi_msb_buffs[index].sock_fd = buff->get_socket();

            //commit the samples to the zero-copy interface

            buff->commit(num_vita_words32*sizeof(uint32_t));
        }

        buff->release();
        buff.reset(); //effectively a release
        
        if (_props[index].go_postal)
        {
            _props[index].go_postal();
        }
    }

    //! Shared variables for the worker threads
    size_t _convert_nsamps;
    const tx_streamer::buffs_type *_convert_buffs;
    size_t _convert_buffer_offset_bytes;
    vrt::if_packet_info_t *_convert_if_packet_info;

};

class send_packet_streamer : public send_packet_handler, public tx_streamer{
public:
    send_packet_streamer(const size_t max_num_samps, const size_t buffer_size = 0):
    sph::send_packet_handler(buffer_size)
    {
        try {
            assert(buffer_size > 0);
        } catch (...) {
            std::cerr << "Buffer size not set in send_packet_streamer. Your device is not fully supported with this version of UHD." << std::endl;
        }
        _max_num_samps = max_num_samps;
        this->set_max_samples_per_packet(_max_num_samps);
    }

    size_t get_num_channels(void) const{
        return this->size();
    }

    size_t get_max_num_samps(void) const{
        return _max_num_samps;
    }

    size_t send(
        const tx_streamer::buffs_type &buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t &metadata,
        const double timeout
    ){
        return send_packet_handler::send(buffs, nsamps_per_buff, metadata, timeout);
    }

    bool recv_async_msg(
        uhd::async_metadata_t &async_metadata, double timeout = 0.1
    ){
        return send_packet_handler::recv_async_msg(async_metadata, timeout);
    }

private:
    size_t _max_num_samps;
};

} // namespace sph
} // namespace transport
} // namespace uhd

#endif /* INCLUDED_LIBUHD_TRANSPORT_SUPER_SEND_PACKET_HANDLER_HPP */
