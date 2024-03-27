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
        _socket = socket_sptr(new asio::ip::udp::socket(_io_service));
        _socket->open(asio::ip::udp::v4());

        // allow broadcasting
        _socket->set_option(asio::socket_base::broadcast(bcast));

        // connect the socket
        if (connect)
            _socket->connect(_send_endpoint);
    }

    size_t send(const asio::const_buffer& buff) override
    {
        if (_connected) {
            // MSG_CONFIRM to avoid uneccessary control packets being sent to verify the destination is where it already is
            ssize_t data_sent = ::send(_socket->native_handle(), buff.data(), buff.size(), MSG_CONFIRM & route_good);
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

        ssize_t ret = sendto(_socket->native_handle(), buff.data(), buff.size(), MSG_CONFIRM & route_good, (struct sockaddr*)&dst_address, sizeof(dst_address));

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
        const int32_t timeout_ms = static_cast<int32_t>(timeout * 1000);

        if (not wait_for_recv_ready(_socket->native_handle(), timeout_ms)) {
            return 0;
        }
        ssize_t data_received = 0;
        // TODO: migrate to libc recvfrom for non connected socket to remove boost
        if(_connected) {
            // MSG_DONTWAIT since wait_for_recv_ready will already wait for data to be ready. If this would block something has gone wrong and return to avoid blocking
            data_received = ::recv(_socket->native_handle(), buff.data(), buff.size(), MSG_DONTWAIT);
            if(data_received == -1) {
                data_received = 0;
            }
        } else {
            data_received = (ssize_t)_socket->receive_from(asio::buffer(buff), _recv_endpoint);
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
        return _recv_endpoint.address().to_string();
    }

    std::string get_send_addr(void) override
    {
        return _send_endpoint.address().to_string();
    }

private:
    bool _connected;
    asio::io_service _io_service;
    socket_sptr _socket;
    asio::ip::udp::endpoint _send_endpoint;
    asio::ip::udp::endpoint _recv_endpoint;
    // Set to 0 until a packet has been received, ~0 once a packet has been received
    // If this has been confirmed send can be called with MSG_CONFIRM
    int route_good = 0;
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
};

uhd::uart_iface::sptr udp_simple::make_uart(sptr udp)
{
    return uart_iface::sptr(new udp_simple_uart_impl(udp));
}
