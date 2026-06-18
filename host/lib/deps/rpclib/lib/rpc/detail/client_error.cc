#include <format>

#include "rpc/detail/client_error.h"

namespace rpc {
namespace detail {

client_error::client_error(code c, const std::string &msg)
    : what_(std::format("client error C{}: {}",
                        static_cast<uint16_t>(c), msg)) {}

const char *client_error::what() const noexcept { return what_.c_str(); }
}
}

