// Copyright 2026 Per Vices Corporation
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#pragma once

#include <string>
#include <stdint.h>

namespace uhd { namespace transport {

/**
 * A simple manager for using a tcp socket
 */
class tcp_simple {

private:

    // File descriptor of the tcp socket
    int tcp_socket_fd = -1;

public:

    /**
     * Make a new TCP connection
     *
     * @param addr The IP address to connect to
     * @param port The port to connect to
     *
     * @throws std::system_error getaddrinfo, socket, or connect failed
     */
    tcp_simple(const std::string& addr, const uint16_t port);

    /**
     * Close the TCP connection
     */
    ~tcp_simple();

    /**
     * Sends a TCP packet
     *
     * @param buff A buffer contianing the payload to send
     * @param size The number of bytes to send
     *
     * @throws std::system_error An error occured during send(2)
     * @throws std::runtime_error Fewer bytes were send during send than intended but it reported success. This is defined as an exception so that callers cannot get undefined behaviour from assuming the program worked
     */
    void send(const void* buff, size_t size);

    /**
     * Receives a TCP packet
     *
     * @param buff The buffer to receive into
     * @param size The size of the buffer
     * @param timeout How long to wait before timing out in seconds
     *
     * @return The number of bytes received, or 0 in the event of a timeout
     *
     * @throws std::system_error Error during ppoll (waiting for packet), or recv(2)
     * @throw std::runtime_error The host closed the connection, or size was 0. This is an error because this class assumes that only it (the client) will close the connection
     */
    size_t recv(void* buff, size_t size, double timeout = 0.1);
};


}} // namespace uhd::transport
