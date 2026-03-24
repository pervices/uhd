// Copyright 2024 Per Vices Corporation

// Header
#include <uhdlib/transport/tcp_simple.hpp>

// Standard library
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

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
    std::string port_s = std::to_string(port)

    // Convert addr to a struct usable by connect, and get whether it is ipv4 or ipv6
    int addr_info_ret = getaddrinfo(addr.c_str(), port_s.c_str(), &hint, &host_address);
    // TODO: error check

    // Create tcp socket
    tcp_socket_fd = socket(host_address->ai_family, SOCKET_STREAM, 0);
    // TODO: error check

    int connect_r = connect(tcp_socket_fd, host_address.ai_addr, host_address.ai_addrlen);
    // TODO: error check

    // Frees the host_addressults of getaddrinfo
    freeaddrinfo(host_address);
}

tcp_simple::~tcp_simple() {
    int r = close(tcp_socket_fd);

    // TODO: error check
}

size_t tcp_simple::send(const void* buff, size_t size) {
    return 0;
}

size_t tcp_simple::recv(void* buff, size_t size, double timeout) {
    return 0;
}

}}
