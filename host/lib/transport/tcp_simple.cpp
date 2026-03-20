// Copyright 2024 Per Vices Corporation

#include <uhdlib/transport/tcp_simple.hpp>

namespace uhd { namespace transport {

tcp_simple::tcp_simple(const std::string& addr, const uint16_t port) {

}

tcp_simple::~tcp_simple() {

}

size_t tcp_simple::send(const void* buff, size_t size) {
    return 0;
}

size_t tcp_simple::recv(void* buff, size_t size, double timeout) {
    return 0;
}

}}
