// Copyright 2024 Per Vices Corporation

// Header
#include <uhdlib/transport/tcp_simple.hpp>

// Internal UHD includes
#include <uhd/utils/log.hpp>

// Standard library
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <poll.h>

namespace uhd { namespace transport {

tcp_simple::tcp_simple(const std::string& addr, const uint16_t port) {

    // Struct containing info used to generate
    struct addrinfo hint;
    memset(&hint, 0, sizeof(hint));

    // Let getaddrinfo determine whether ipv4 or ipv6
    hint.ai_family = AF_UNSPEC;
    // TCP
    hint.ai_socktype = SOCK_STREAM;
    // Speed up check by letting getaddrinfo know that it is provided a numeric ip
    hint.ai_flags = AI_NUMERICHOST;

    struct addrinfo* host_address = NULL;

    // Create string from port here to ensure it exists while c_str is in use
    std::string port_s = std::to_string(port);

    // Convert addr to a struct usable by connect, and get whether it is ipv4 or ipv6
    int addr_info_ret = getaddrinfo(addr.c_str(), port_s.c_str(), &hint, &host_address);
    // TODO: error check

    // Create tcp socket
    tcp_socket_fd = socket(host_address->ai_family, SOCK_STREAM, 0);
    // TODO: error check

    int connect_r = connect(tcp_socket_fd, host_address->ai_addr, host_address->ai_addrlen);
    // TODO: error check

    // Frees the host_addressults of getaddrinfo
    freeaddrinfo(host_address);
}

tcp_simple::~tcp_simple() {
    int r = close(tcp_socket_fd);

    // TODO: error check
}

void tcp_simple::send(const void* buff, size_t size) {
    int r = ::send(tcp_socket_fd, buff, size, 0);

    // TODO: error check
    return;
}

// TODO: replace runtime_error with something mroe specific
size_t tcp_simple::recv(void* buff, size_t size, double timeout) {

    struct pollfd  pfds[1];
    pfds[0].fd = tcp_socket_fd;
    // List of the event we want to listen for
    // POLLIN is the only event that should happen, all others should be considered errors
    pfds[0].events = POLLIN | POLLPRI | POLLRDHUP | POLLERR | POLLHUP;
    // Return value, initialize to 0 to prevent undefined errors
    pfds[0].revents = 0;

    struct timespec ts_timeout;
    ts_timeout.tv_sec = (time_t) timeout;
    ts_timeout.tv_nsec = (long) ((timeout - ts_timeout.tv_sec) * 1000000000);

    int recv_ready = ppoll(pfds, 1, &ts_timeout, NULL);

    if(recv_ready < 0) {
        // TODO: call ppoll again with the remaining time if EINTR is returned
        UHD_LOG_ERROR("TCP_SIMPLE", "Error from ppoll while waiting for packet: " + strerror(errno));
        throw std::system_error(errno, std::generic_category(), "Error during ppoll when waiting for packet(s)");
    } else if(recv_ready == 0) {
        // No packet was ready
        return 0;
    }

    ssize_t bytes_received = ::recv(tcp_socket_fd, buff, size, MSG_DONTWAIT);

    if(bytes_received < 0) {
        // This should be impossible, but is included just in case to ensure the program doesn't freeze
        if(errno == EAGAIN || errno == EWOULDBLOCK) {
            UHD_LOG_ERROR("TCP_SIMPLE", "ppoll indicated data ready but no data was found");
            throw std::runtime_error("Missing rx packet");
        } else {
            // All other errors shouldn't happen
            // TODO: verify that EINTR can't happen since we already know there is data from ppoll
            UHD_LOG_ERROR("TCP_SIMPLE", "recv failed with: " + strerror(errno));
            throw std::system_error(errno, std::generic_category(), "recv failed");
        }
        // The SDR should never be the one to close the connection. A closed connection indicates something went very wrong
    } else if(bytes_received == 0) {
        UHD_LOG_ERROR("TCP_SIMPLE", "The SDR unexpectedly closed the connection");
        throw std::runtime_error("Connection closed");
        // recv successful
    } else {
        return bytes_received;
    }
}

}}
