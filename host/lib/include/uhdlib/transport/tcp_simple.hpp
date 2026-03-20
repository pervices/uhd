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

    /**
     * Make a new TCP connection
     *
     * @param addr The IP address to connect to
     * @param port The port to connect to
     *
     * @throws TODO TODO this comment
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
     * @return The number of bytes sent
     *
     * @throws TODO TODO this comment
     */
    size_t send(const void* buff, size_t size);

    /**
     * Receives a TCP packet
     *
     * @param buff The buffer to receive into
     * @param size The size of the buffer
     * @param timeout How long to wait before timing out
     *
     * @return The number of bytes received
     *
     * @throws TODO TODO this comment
     */
    size_t recv(void* buff, size_t size, double timeout = 0.1);
};


}} // namespace uhd::transport
