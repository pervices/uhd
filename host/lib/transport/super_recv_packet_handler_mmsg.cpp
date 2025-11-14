//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2023-2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/transport/super_recv_packet_handler_mmsg.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/log.hpp>
#include <uhdlib/utils/network_config.hpp>
#include <uhdlib/utils/performance_mode.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include <uhd/utils/thread.hpp>
#include <algorithm>

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

namespace uhd { namespace transport { namespace sph {

recv_packet_handler_mmsg:: recv_packet_handler_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuers)
    :
    _NUM_CHANNELS(recv_sockets.size()),
    _MAX_SAMPLE_BYTES_PER_PACKET(max_sample_bytes_per_packet),
    _HEADER_SIZE(header_size),
    _TRAILER_SIZE(trailer_size),
    _stream_cmd_issuers(cmd_issuers),
    _recv_sockets(recv_sockets),
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

    // Checks if preemption is disabled/voluntary and warns user if it is not
    check_pre_empt();

    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        check_rx_ring_buffer_size(dst_ip[n]);
    }

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
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Unable to set recv buffer size. Performance will be negatively affected.\n Target size: " + std::to_string(DEFAULT_RECV_BUFFER_SIZE) + "\nActual size: " + std::to_string(actual_recv_buffer_size/2) + "\nPlease run \"sudo sysctl -w net.core.rmem_max=" + std::to_string(DEFAULT_RECV_BUFFER_SIZE) + "\"\n");
        }

        // Verify the interface can handle large packets
        int mtu = get_mtu(_recv_sockets[n], dst_ip[n].c_str());
        if(mtu < MIN_MTU) {
            UHD_LOG_ERROR("RECV_PACKET_HANDLER", "MTU of interface associated with " + dst_ip[n] + "is to small. " + std::to_string(MIN_MTU) + "required, the current value is " + std::to_string(mtu) + ".\n");
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

    setup_converter(cpu_format, wire_format, wire_little_endian);

    // Check if the governor is set to performance mode, warns the user if it is not
    check_if_only_using_governor();

    // Create manager for receive threads and access to buffer recv data
    recv_manager = async_recv_manager::auto_make(device_total_rx_channels, recv_sockets, header_size, max_sample_bytes_per_packet);

    overflow_messenger = std::thread(send_overflow_messages_loop, this);
}

recv_packet_handler_mmsg::~recv_packet_handler_mmsg(void)
{
    stop_overflow_loop = true;

    async_recv_manager::auto_unmake(recv_manager);
    // recv_manager must be deleted before closing sockets
    for(size_t n = 0; n < _recv_sockets.size(); n++) {
        int r = close(_recv_sockets[n]);
        if(r) {
            fprintf(stderr, "close failed on data receive socket with: %s\nThe program may not have closed cleanly\n", strerror(errno));
        }
    }

    if(_overflow_occured && _suboptimal_spb) {
        UHD_LOGGER_WARNING("RECV_PACKET_HANDLER_MMSG") << "An overflow occured during a run where a subopitmal number of samples were requested from recv. To reduce the chance of an overflow in the future ensure nsamps_per_buff is multiple of " << _MAX_SAMPLE_BYTES_PER_PACKET / _BYTES_PER_SAMPLE;
    }

    overflow_messenger.join();
}

void recv_packet_handler_mmsg::set_sample_rate(const double rate)
{
    _sample_rate = rate;
}

void recv_packet_handler_mmsg::issue_stream_cmd(const stream_cmd_t& stream_cmd)
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

int recv_packet_handler_mmsg::get_mtu(int socket_fd, std::string ip) {
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

void recv_packet_handler_mmsg::send_overflow_messages_loop(recv_packet_handler_mmsg* self) {
    // Set priority to the lowest
    uhd::set_thread_priority_safe(0, false);

    // Number of oflows already printed
    uint64_t oflows_printed = 0;

    while(!(self->stop_overflow_loop)) {
        // Load fence to ensure getting oflows_printed from other threads doesn't get optimized out
        _mm_lfence();

        uint64_t oflows_to_print = self->oflows_to_print - oflows_printed;
        // Print a D for every recv command that's had an overflow since the last iteration of this loop
        if(oflows_to_print) {
            // Only print up to 50 Ds at a time, skip any extra since more will just clog up the logs
            std::string message(std::min(oflows_to_print, (uint64_t) 50), 'D');
            // Print without the normal UHD formatting to keep the output format the same as Ettus
            std::cout << message;

            oflows_printed += oflows_to_print;
        } else {
            // Sleep until polling again
            ::usleep(250000);
        }

    }
}

void recv_packet_handler_mmsg::check_high_order_alloc_disable() {
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

void recv_packet_handler_mmsg::check_pre_empt() {
    std::string path = "/sys/kernel/debug/sched/preempt";

    FILE *file;

    file = fopen(path.c_str(), "r");

    if(file == NULL) {
        if(errno == EACCES) {
            UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Insufficient permission to check preemption setting. Check " + path + " to manually check it's current setting. It must be set to none or voluntary for optimal performance.\nTo give non root users access to said file run \"sudo mount -o remount,mode=0755 -t debugfs none /sys/kernel/debug/\". \"remount\" is required in order to update permissions due to a bug affecting most variants of kernel 6.);
            // Discussion of the kernel bug mentioned above: https://bugzilla.kernel.org/show_bug.cgi?id=220406
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
        UHD_LOG_WARNING("RECV_PACKET_HANDLER", "Preemption is currently enabled, this may cause infrequent performance issues. Run \"echo voluntary > " + path + "\" as root. Said command must be run as root user, sudo will not work.");
    }
}

void recv_packet_handler_mmsg::check_rx_ring_buffer_size(std::string ip) {
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

recv_packet_streamer_mmsg::recv_packet_streamer_mmsg(const std::vector<int>& recv_sockets, const std::vector<std::string>& dst_ip, const size_t max_sample_bytes_per_packet, const size_t header_size, const size_t trailer_size, const std::string& cpu_format, const std::string& wire_format, bool wire_little_endian, size_t device_total_rx_channels, std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuers)
    : recv_packet_handler_mmsg(recv_sockets, dst_ip, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian, device_total_rx_channels, cmd_issuers)
{
}

void recv_packet_streamer_mmsg::post_input_action(const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t)
{
    throw uhd::not_implemented_error("post_output_action is not implemented for this device");
}

}}} // namespace uhd::transport::sph

