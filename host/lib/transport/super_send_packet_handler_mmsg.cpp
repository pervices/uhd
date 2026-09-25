//
// Copyright 2023-2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/transport/super_send_packet_handler_mmsg.hpp>
#include <vector>

// Smart pointers
#include <memory>

#include <cmath>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/file.h>

#include <uhdlib/utils/preemption_check.hpp>

namespace uhd {
namespace transport {
namespace sph {

send_packet_handler_mmsg::send_packet_handler_mmsg(const std::vector<size_t>& channels, ssize_t max_samples_per_packet, const int64_t device_buffer_size, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, ssize_t device_packet_nsamp_multiple, double tick_rate, const std::shared_ptr<pv_tx_async_msg_queue> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info_owner, std::vector<int> streaming_locks)
    // Ensure max_samples_per_packet is a multiple of the number of samples allowed per packet
    :
    _DEVICE_TARGET_NSAMPS(device_target_nsamps),
    _DEVICE_PACKET_NSAMP_MULTIPLE(device_packet_nsamp_multiple),
    _max_samples_per_packet((max_samples_per_packet / device_packet_nsamp_multiple) * device_packet_nsamp_multiple),
    _MAX_SAMPLE_BYTES_PER_PACKET(_max_samples_per_packet * _bytes_per_sample),
    _TICK_RATE(tick_rate),
    _DEVICE_BUFFER_SIZE(device_buffer_size),
    _NUM_CHANNELS(channels.size()),
    _clock_sync(clock_sync_info_owner.get()),
    _clock_sync_owner(clock_sync_info_owner),
    _intermediate_send_buffer_pointers(_NUM_CHANNELS),
    _intermediate_send_buffer_wrapper(_intermediate_send_buffer_pointers.data(), _NUM_CHANNELS),
    _async_msg_fifo(async_msg_fifo),
    _streaming_locks(streaming_locks),
    _reprime_threshold((int64_t)(device_buffer_size * 0.1))
{
    // Checks and warns the user if the preemption mode is suboptimal
    check_preemption("SEND_PACKET_HANDLER");

    // Copy provided channel list vector to internal channel list
    std::copy(channels.begin(), channels.end(), _channels);


    ch_send_buffer_info_group = std::vector<ch_send_buffer_info>(_NUM_CHANNELS, ch_send_buffer_info(0, HEADER_SIZE, _bytes_per_sample * (_DEVICE_PACKET_NSAMP_MULTIPLE - 1), _sample_rate));

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

        // Sets the send buffer size
        setsockopt(send_socket_fd, SOL_SOCKET, SO_SNDBUF, &_DEFAULT_SEND_BUFFER_SIZE, sizeof(_DEFAULT_SEND_BUFFER_SIZE));

        // The actual size of the send buffer
        int _actual_send_buffer_size = 0;

        // Checks the recv buffer size
        socklen_t opt_len = sizeof(_actual_send_buffer_size);
        getsockopt(send_socket_fd, SOL_SOCKET, SO_SNDBUF, &_actual_send_buffer_size, &opt_len);

        // NOTE: The kernel will set the actual size to be double the requested. So the expected amount is double the requested
        if(_actual_send_buffer_size < 2*_DEFAULT_SEND_BUFFER_SIZE) {
            UHD_LOG_ERROR("SEND_PACKET_HANDLER", "Unable to set send buffer size. Performance will be negatively affected.\n Target size: " + std::to_string(_DEFAULT_SEND_BUFFER_SIZE) + "\nActual size: " + std::to_string(_actual_send_buffer_size/2) + "\nPlease run \"sudo sysctl -w net.core.wmem_max=" + std::to_string(_DEFAULT_SEND_BUFFER_SIZE) + "\"\n");
        }

        int mtu = get_mtu(send_socket_fd, dst_ips[n].c_str());
        if(mtu < MIN_MTU) {
            UHD_LOG_ERROR("SEND_PACKET_HANDLER", "MTU of interface associated with " + dst_ips[n] + "is to small. " + std::to_string(MIN_MTU) + "required, the current value is " + std::to_string(mtu) + ".\n");
            throw uhd::system_error("MTU size to small");
        }

        int set_priority_ret = setsockopt(send_socket_fd, SOL_SOCKET, SO_PRIORITY, &TX_SO_PRIORITY, sizeof(TX_SO_PRIORITY));
        if(set_priority_ret) {
            fprintf(stderr, "Attempting to set tx socket priority failed with error code: %s", strerror(errno));
        }

        send_sockets[n] = send_socket_fd;
    }

    setup_converter(cpu_format, wire_format, wire_little_endian);

    // Check if the governor is set to performance mode, warns the user if it is not
    check_if_only_using_governor();
}

send_packet_handler_mmsg::~send_packet_handler_mmsg(void){
    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        int r = close(send_sockets[n]);
        if(r) {
            fprintf(stderr, "close failed on data send socket with: %s\nThe program may not have closed cleanly\n", strerror(errno));
        }
    }

    if(sendmmsg_errno) {
        char nsec_s[42];
        snprintf(nsec_s, sizeof(nsec_s), "%ld.%.9ld", sendmmsg_failure_time.tv_sec, sendmmsg_failure_time.tv_nsec);

        UHD_LOG_ERROR("SEND_PACKET_HANDLER", "A sendmmsg command failed to send packets with error code " + std::string(strerror(sendmmsg_errno)) + " at time " + std::string(nsec_s));
    }
}

void send_packet_handler_mmsg::set_samp_rate(const double rate) {
    _sample_rate = rate;
    for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
        ch_send_buffer_info_i.buffer_level_manager.set_sample_rate(rate);
    }

    // Drop packets if they are within the lower of 20% of a buffer of being late and 40us
    // 40e-6 was chosen since it is extremely rare for packets to be over 35e-6 seconds late based on clock sync predictions
    drop_lead = std::min(0.2 * _DEVICE_BUFFER_SIZE / _sample_rate, 40e-6);
}

void send_packet_handler_mmsg::enable_blocking_fc(int64_t blocking_setpoint) {
    use_blocking_fc = true;
    if(blocking_setpoint > 0.9 * _DEVICE_BUFFER_SIZE) {
        blocking_setpoint = (uint64_t) (0.9*_DEVICE_BUFFER_SIZE);
    };
    this->blocking_setpoint = blocking_setpoint;
}

void send_packet_handler_mmsg::disable_blocking_fc() {
    use_blocking_fc = false;
}

// Attempt to get the streaming lock for a channel.
void send_packet_handler_mmsg::lock_channel_streaming(size_t channel_num) {
    // Set an exclusive lock on the channel lockfile. Nonblocking so it fails if already locked instead of waiting for it to be unlocked.
    int r = flock(_streaming_locks[channel_num], LOCK_EX | LOCK_NB);
    if (r == -1) {
        int err = errno;
        // EWOULDBLOCK is expected if there is already a lock since we ran with the LOCK_NB flag.
        if (err == EWOULDBLOCK) {
            throw uhd::runtime_error("Another instance of UHD is currently using channel " + std::to_string(channel_num) + ".");
        } else {
            throw uhd::runtime_error("flock failed to lock streaming for channel " + std::to_string(channel_num) + " with error: " + std::string(strerror(err)));
        }
    }
}

send_packet_handler_mmsg::ch_send_buffer_info::ch_send_buffer_info(const size_t size, const size_t vrt_header_size, const size_t cache_size, const double rate)
: _vrt_header_size(vrt_header_size),
sample_cache(std::vector<int8_t>(cache_size)),
buffer_level_manager(rate)
{
    resize_and_clear(size);
}

void send_packet_handler_mmsg::ch_send_buffer_info::resize_and_clear(size_t new_size) {
    msgs.resize(new_size);
    memset(msgs.data(), 0, sizeof(mmsghdr)*new_size);
    // 1 VRT header and 1 data section in every packet, plus the cached samples in the first packet
    iovecs.resize(2*new_size + 1);
    memset(iovecs.data(), 0, sizeof(iovec)*2*new_size);
    vrt_headers.resize(new_size);
    std::fill(vrt_headers.begin(), vrt_headers.end(), std::vector<uint32_t>(_vrt_header_size/sizeof(uint32_t), 0));
    sample_data_start_for_packet.resize(new_size, 0);
}

void send_packet_handler_mmsg::expand_send_buffer_info(size_t new_size) {
    // Resizes the per channel buffers used in the send command
    if(new_size > send_buffer_info_size) {
        send_buffer_info_size = new_size;
        packet_header_infos.resize(new_size);
        for(size_t ch_i = 0; ch_i < _NUM_CHANNELS; ch_i++) {
            ch_send_buffer_info_group[ch_i].resize_and_clear(new_size);
        }
    }
}

int send_packet_handler_mmsg::check_fc_npackets(const size_t ch_i) {
    if(!use_blocking_fc) [[likely]] {

        // Get the buffer level on the unit
        uhd::time_spec_t device_time = _clock_sync->get_device_time();
        int64_t buffer_level = ch_send_buffer_info_group[ch_i].buffer_level_manager.get_buffer_level(device_time);

        int num_packets_to_send = (int) std::ceil((_DEVICE_TARGET_NSAMPS - buffer_level) / ((double)_max_samples_per_packet));

        return num_packets_to_send;

    } else {
        int64_t buffer_level = get_buffer_level_from_device(ch_i);
        return (int) std::ceil((blocking_setpoint - buffer_level) / ((double)_max_samples_per_packet));
    }
}

void send_packet_handler_mmsg::send_eob_packet(const uhd::tx_metadata_t &metadata, double timeout) {

    // Clear the flag indicating that a time was specified for this burst
    // since the burst is over
    specified_time = false;

    // How many dummy samples to send in the eob
    constexpr size_t dummy_samples_in_eob = 1;

    // Create vector of dummy samples, since the FPGA cannot handle 0 sample packets
    std::vector<std::vector<int8_t>> dummy_buffs(_NUM_CHANNELS, std::vector<int8_t>(_BYTES_PER_SAMPLE * dummy_samples_in_eob, 0));
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

    // Record amount of samples dropped so that user may be informed of it if they start a new stream
    // Don't print warning about dropped samples here because it often results in umimportant warnings
    dropped_nsamps_in_cache = nsamps_in_cache;

    // Drop any samples in the cache, since otherwise they would be added to the next burst
    nsamps_in_cache = 0;

    // Sends the eob packet
    send_multiple_packets(dummy_buff_ptrs, dummy_samples_in_eob, eob_md, timeout, true);

    // Unlock the streaming lockfiles for each channel
    for (size_t n = 0; n < _NUM_CHANNELS; n++) {
        flock(_streaming_locks[n], LOCK_UN);
    }
}

int send_packet_handler_mmsg::get_mtu(int socket_fd, std::string ip) {
    //Start of linked list containing interface info
    struct ifaddrs *ifaces = nullptr;

    // Converts the target IP to a network-order 32 bit integer
    struct in_addr sdr_ip_addr;
    if(inet_pton(AF_INET, ip.c_str(), &sdr_ip_addr) != 1) {
        throw uhd::runtime_error("inet_pton error when converting " +ip);
    }
    uint32_t sdr_ip = sdr_ip_addr.s_addr;

    // Gets a linked list of all interfaces
    getifaddrs(&ifaces);
    for(ifaddrs *iface = ifaces; iface != NULL; iface = iface->ifa_next) {

        // Verifies this interface has an address and netmask assigned
        if(iface->ifa_addr != nullptr && iface->ifa_netmask != nullptr) {
            // Verifies the address and netmask are IPV4
            if(iface->ifa_addr->sa_family == AF_INET && iface->ifa_netmask->sa_family == AF_INET) {
                // Get the ip and netmask of the network interface in network byte order
                uint32_t interface_ip = ((struct sockaddr_in*)(iface->ifa_addr))->sin_addr.s_addr;
                uint32_t netmask = ((struct sockaddr_in*)(iface->ifa_netmask))->sin_addr.s_addr;

                // Checks if the sdr's ip is within this interface's subnet.
                if((interface_ip & netmask) != (sdr_ip & netmask)) {
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

    throw uhd::system_error("SEND: no interface with subnet matching ip " + ip + " found");
}

size_t send_packet_handler_mmsg::send(
    const uhd::tx_streamer::buffs_type &sample_buffs,
    const size_t nsamps_to_send,
    const uhd::tx_metadata_t &metadata,
    const double timeout
) {
    // If no converter is required data will be written directly into buffs, otherwise it is written to an intermediate buffer
    const uhd::tx_streamer::buffs_type *send_buffer = (converter_used) ? prepare_intermediate_buffers_and_convert(sample_buffs, nsamps_to_send) : &sample_buffs;

    // If the user ever specified a time enable dropping late packets to help with phase
    specified_time = metadata.has_time_spec || specified_time;

    size_t previous_nsamps_in_cache = nsamps_in_cache;

    // FPGAs can sometimes only receive multiples of a set number of samples
    size_t actual_nsamps_to_send = (((nsamps_in_cache + nsamps_to_send) / _DEVICE_PACKET_NSAMP_MULTIPLE) * _DEVICE_PACKET_NSAMP_MULTIPLE);
    size_t desired_nsamps_to_cache = nsamps_to_send + nsamps_in_cache - actual_nsamps_to_send;

    if(actual_nsamps_to_send == 0) {
        // If a start of burst command has no packets, and is not also an end of burstcache timestamp and keep until next call
        if(metadata.start_of_burst && !metadata.end_of_burst) {
            cached_sob = true;
            // If the SOB to cache has a timespec, cache it so it can be applied later
            if(metadata.has_time_spec) {
                sob_time_cache = metadata.time_spec;
            }
            // If no time spec was provided (or the provided time spec was 0)
            // set the time spec to -1 to indicate that it should be auto applied when used
            else {
                sob_time_cache = -1.0;
            }
            return 0;
        } else if(metadata.end_of_burst) {
            send_eob_packet(metadata, timeout);
            
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
    if(cached_sob) [[unlikely]] {
        cached_sob = false;
        modified_metadata.start_of_burst = true;
        // -1 indicates no time spec was provided with the cached SOB request
        modified_metadata.has_time_spec = sob_time_cache != -1.0;
        modified_metadata.time_spec = sob_time_cache;
        modified_metadata.time_spec = sob_time_cache;
    }

    // Automatically apply start time if none was provided
    // NOTE: must be after the cached_sob was applied
    if(modified_metadata.start_of_burst && !modified_metadata.has_time_spec ) [[unlikely]] {
        modified_metadata.has_time_spec = true;
        modified_metadata.time_spec = _clock_sync->get_device_time() + SEND_NOW_DELAY;

    // Reprime if the buffer goes to low and the user did not specify a timestamp.
    // This causes a phase shift with respect to everything outside the streamer.
    // Said phase shift is acceptable in order to match upstream's behaviour and since it is probably okay to phase shift if the user didn't specify a time
    //
    // The user has not specified times
    // This is not an end of burst with no samples
    // The system is not in trigger mode where timestamps are ignored (!use_blocking_fc)
    // (Implcitly from else) this is not a start of burst
    } else if(!specified_time && !(modified_metadata.end_of_burst && actual_nsamps_to_send == 0) && !use_blocking_fc) [[unlikely]] {
        // Get prediced buffer level
        uhd::time_spec_t device_time = _clock_sync->get_device_time();
        // buffer_level_manager will be the same for all channels within a streamer so we can just check the first
        int64_t buffer_level = ch_send_buffer_info_group[0].buffer_level_manager.get_buffer_level(device_time);

        // If we are not mid reprime and the buffer level is below the target threshold
        if( device_time > ch_send_buffer_info_group[0].buffer_level_manager.peek_last_sob() && buffer_level  < _reprime_threshold) [[likely]] {

            printf("Reprime triggered after: %lu\n", samples_since_last_reprime);
            samples_since_last_reprime = 0;
            // Time to start a new pseudo burst to recover
            // Reprime to 90% of the target buffer level
            uhd::time_spec_t reprime_time = device_time + SEND_NOW_DELAY;
            // Update the buffer tracker to manage the new time
            for(auto& ch_send_buffer_info_i : ch_send_buffer_info_group) {
                ch_send_buffer_info_i.buffer_level_manager.recovery_prep(reprime_time);
            }
            // Apply the start time for the new pseudo burst
            modified_metadata.has_time_spec = true;
            modified_metadata.time_spec = reprime_time;
        }
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
    samples_since_last_reprime += actual_samples_sent - cached_samples_sent + actual_nsamples_to_cache;
    return actual_samples_sent - cached_samples_sent + actual_nsamples_to_cache;
}

void send_packet_handler_mmsg::setup_converter(const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian) {
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


send_packet_streamer_mmsg::send_packet_streamer_mmsg(const std::vector<size_t>& channels, ssize_t max_samples_per_packet, const int64_t device_buffer_size, std::vector<std::string>& dst_ips, std::vector<int>& dst_ports, int64_t device_target_nsamps, ssize_t device_packet_nsamp_multiple, double tick_rate, const std::shared_ptr<pv_tx_async_msg_queue> async_msg_fifo, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, std::shared_ptr<uhd::usrp::clock_sync> clock_sync_info, std::vector<int> streaming_locks):
sph::send_packet_handler_mmsg(channels, max_samples_per_packet, device_buffer_size, dst_ips, dst_ports, device_target_nsamps, device_packet_nsamp_multiple, tick_rate, async_msg_fifo, cpu_format, wire_format, wire_little_endian, clock_sync_info, streaming_locks)
{
}
    
bool send_packet_streamer_mmsg::recv_async_msg(
    uhd::async_metadata_t &async_metadata, double timeout
){
    // Return !pop because in this case pop return 0 on success, this function returns true on success
    return !_async_msg_fifo->pop(&async_metadata, timeout);
}

void send_packet_streamer_mmsg::push_async_msg( uhd::async_metadata_t &async_metadata ){
    _async_msg_fifo->push(&async_metadata);
}

void send_packet_streamer_mmsg::enable_blocking_fc(uint64_t blocking_setpoint) {
    // TODO: change tx_streamer to use int64_t instead of uint64_t
    send_packet_handler_mmsg::enable_blocking_fc((int64_t)blocking_setpoint);
}

void send_packet_streamer_mmsg::disable_blocking_fc() {
    send_packet_handler_mmsg::disable_blocking_fc();
}

void send_packet_streamer_mmsg::post_output_action(const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t)
{
    throw uhd::not_implemented_error("post_output_action is not implemented for this device");
}

} // namespace sph
} // namespace transport
} // namespace uhd
