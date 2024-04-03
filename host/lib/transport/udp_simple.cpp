//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/transport/udp_simple.hpp>
#include <uhd/utils/log.hpp>
#include <uhdlib/transport/udp_common.hpp>
#include <boost/format.hpp>

using namespace uhd::transport;
namespace asio = boost::asio;

/***********************************************************************
 * UDP simple implementation: connected and broadcast
 **********************************************************************/
class udp_simple_impl : public udp_simple
{
public:
    udp_simple_impl(
        const std::string& addr, const std::string& port, bool bcast, bool connect)
        : _connected(connect)
    {
        UHD_LOGGER_TRACE("UDP")
            << boost::format("Creating udp transport for %s %s") % addr % port;

        // resolve the address
        asio::ip::udp::resolver resolver(_io_service);
        asio::ip::udp::resolver::query query(
            asio::ip::udp::v4(), addr, port, asio::ip::resolver_query_base::all_matching);
        _send_endpoint = *resolver.resolve(query);

        // create and open the socket

        socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);

        // allow broadcasting
        int broadcast_enable = bcast;
        setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

        // connect the socket
        if (connect) {
            struct sockaddr_in dst_address;
            dst_address.sin_family = AF_INET;
            std::string ipv4_addr = _send_endpoint.address().to_string();
            dst_address.sin_addr.s_addr = inet_addr(ipv4_addr.c_str());
            dst_address.sin_port = htons(_send_endpoint.port());

            int r = ::connect(socket_fd, (struct sockaddr*)&dst_address, sizeof(dst_address));

            if(r) {
                throw uhd::runtime_error("Failed to connect send socket for control packets. Error code:" + std::string(strerror(errno)));
            }
        }

        // Sets socket priority to minimum
        int priority = 0;
        setsockopt(socket_fd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));
    }

    ~udp_simple_impl(void) {
        close(socket_fd);
    }

    size_t send(const asio::const_buffer& buff) override
    {
        return send(buff.data(), buff.size());
    }

    size_t send(const void* buff, size_t count) override
    {
        if (_connected) {
            // MSG_CONFIRM to avoid uneccessary control packets being sent to verify the destination is where it already is
            ssize_t data_sent = ::send(socket_fd, buff, count, MSG_CONFIRM & route_good);
            if(data_sent == -1) {
                fprintf(stderr, "send failed for control packet. errno: %s\n", strerror(errno));
                return 0;
            } else {
                return data_sent;
            }
        }

        struct sockaddr_in dst_address;
        memset(&dst_address, 0, sizeof(dst_address));
        dst_address.sin_family = AF_INET;
        std::string ipv4_addr = _send_endpoint.address().to_string();
        dst_address.sin_addr.s_addr = inet_addr(ipv4_addr.c_str());
        dst_address.sin_port = htons(_send_endpoint.port());

        ssize_t ret = sendto(socket_fd, buff, count, MSG_CONFIRM & route_good, (struct sockaddr*)&dst_address, sizeof(dst_address));

        if(ret > 0) {
            return ret;
        } else {
            fprintf(stderr, "sendto failed for control packet. errno: %s\n", strerror(errno));
            // Return 0 to keep behaviour from asio
            return 0;
        }
    }

    size_t recv(const asio::mutable_buffer& buff, double timeout) override
    {
        return recv(buff.data(), buff.size(), timeout);
    }


    size_t recv(void* buff, size_t size, double timeout) override
    {
        const int32_t timeout_ms = static_cast<int32_t>(timeout * 1000);

        if (not wait_for_recv_ready(socket_fd, timeout_ms)) {
            return 0;
        }
        ssize_t data_received = 0;
        if(_connected) {
            // MSG_DONTWAIT since wait_for_recv_ready will already wait for data to be ready. If this would block something has gone wrong and return to avoid blocking
            data_received = ::recv(socket_fd, buff, size, MSG_DONTWAIT);
            if(data_received == -1) {
                data_received = 0;
            }
        } else {
            struct sockaddr_in src_address;
            memset(&src_address, 0, sizeof(src_address));
            uint32_t addr_len = sizeof(src_address);

            data_received = ::recvfrom(socket_fd, buff, size, MSG_DONTWAIT, (struct sockaddr*)&src_address, &addr_len);
            if(data_received == -1) {
                data_received = 0;
            }

            recv_ip = inet_ntoa(src_address.sin_addr);
        }

        // If data has been received, then we know routing is good
        if(data_received) {
            route_good = ~0;
        } else {
            route_good = 0;
        }

        return (size_t)data_received;
    }

    std::string get_recv_addr(void) override
    {
        return recv_ip;
    }

    std::string get_send_addr(void) override
    {
        return _send_endpoint.address().to_string();
    }

private:
    bool _connected;
    asio::io_service _io_service;
    asio::ip::udp::endpoint _send_endpoint;
    // IP address received packets originated from
    std::string recv_ip = "0.0.0.0";
    // Set to 0 until a packet has been received, ~0 once a packet has been received
    // If this has been confirmed send can be called with MSG_CONFIRM
    int route_good = 0;
    int socket_fd;
};

udp_simple::~udp_simple(void)
{
    /* NOP */
}

/***********************************************************************
 * UDP public make functions
 **********************************************************************/
udp_simple::sptr udp_simple::make_connected(
    const std::string& addr, const std::string& port)
{
    return sptr(new udp_simple_impl(addr, port, false, true /* no bcast, connect */));
}

udp_simple::sptr udp_simple::make_broadcast(
    const std::string& addr, const std::string& port)
{
    return sptr(new udp_simple_impl(addr, port, true, false /* bcast, no connect */));
}

/***********************************************************************
 * Simple UART over UDP
 **********************************************************************/
#include <boost/thread/thread.hpp>
class udp_simple_uart_impl : public uhd::uart_iface
{
public:
    udp_simple_uart_impl(udp_simple::sptr udp)
    {
        _udp = udp;
        _len = 0;
        _off = 0;
        this->write_uart(""); // send an empty packet to init
    }

    void write_uart(const std::string& buf) override
    {
        _udp->send(asio::buffer(buf));
    }

    std::string read_uart(double timeout) override
    {
        std::string line;
        const boost::system_time exit_time =
            boost::get_system_time()
            + boost::posix_time::milliseconds(long(timeout * 1000));
        do {
            // drain anything in current buffer
            while (_off < _len) {
                const char ch = _buf[_off++];
                _line += ch;
                if (ch == '\n') {
                    line.swap(_line);
                    return line;
                }
            }

            // recv a new packet into the buffer
            _len = _udp->recv(asio::buffer(_buf),
                std::max(
                    (exit_time - boost::get_system_time()).total_milliseconds() / 1000.,
                    0.0));
            _off = 0;

        } while (_len != 0);
        return line;
    }

private:
    udp_simple::sptr _udp;
    size_t _len, _off;
    uint8_t _buf[udp_simple::mtu];
    std::string _line;
    int socket_fd;
};

uhd::uart_iface::sptr udp_simple::make_uart(sptr udp)
{
    return uart_iface::sptr(new udp_simple_uart_impl(udp));
}
