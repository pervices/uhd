// Copyright 2024 Per Vices Corporation

// UHD
#include <uhdlib/utils/numa_helpers.hpp>
#include <uhd/utils/log.hpp>

// libnuma
#include <numaif.h>

// Standard library
#include <cerrno>
#include <cstring>
#include <string>
#include <system_error>
#include <fcntl.h>
#include <unistd.h>
#include <cctype>
#include <charconv>
#include <string_view>

namespace uhd {

bool is_numa_relevant() {
    // Use magic statics so the result is only needed to be computed once
    static const bool relevant = [] {
        // Check if the kernel supports NUMA
        if (numa_available() < 0) {
            UHD_LOG_WARNING("NUMA", "The kernel does not support NUMA. NUMA related optimizations will not be applied.");
            return false;
        }
        // Check if we have multiple NUMA nodes
        return numa_num_configured_nodes() > 1;
    }();
    return relevant;
}
scoped_numa_membind::scoped_numa_membind(const bitmask* node_mask)
: _previous_mode(MPOL_DEFAULT)
{
    if(numa_available() < 0) {
        UHD_LOG_ERROR("NUMA", "mbind helper called but the kernel does not support NUMA");
        throw std::system_error(
            std::make_error_code(std::errc::function_not_supported),
            "mbind helper called but the kernel does not support NUMA"
        );
    }

    if(numa_bitmask_weight(node_mask)) {
        throw std::invalid_argument("Scope bind to multiple sockets requested. Only binding to 1 is supported");
    }


    // Allocate early so it exists for the destructor when the class does nothing
    _previous_nodes = numa_allocate_nodemask();

    // Do nothing when node_mask is null
    if(node_mask == nullptr) {
        return;
    }

    // Get the number of nodes in the mask
    unsigned int mask_num_nodes = numa_bitmask_weight(node_mask);

    // Do nothing if no nodes specified
    if(mask_num_nodes == 0) {
        return;
    }

    // Get the current mem policy and store it in _previous_mode, and the mask for it in _previous_nodes
    if(get_mempolicy(&_previous_mode, _previous_nodes->maskp, _previous_nodes->size + 1, nullptr, 0) != 0) {
        UHD_LOG_WARNING("NUMA", "Unable to get the current NUMA memory policy: " + std::string(strerror(errno)) + ". Memory will not be bound to the requested NUMA node(s).");
        return;
    }

    if(set_mempolicy(MPOL_PREFERRED, node_mask->maskp, node_mask->size + 1) != 0) {
        UHD_LOG_WARNING("NUMA", "Unable to bind memory to the requested NUMA node(s): " + std::string(strerror(errno)) + ". Performance may be impacted.");
        return;
    } else {
        // Mem policy was set successfully
        _active = true;
    }
}
scoped_numa_membind::~scoped_numa_membind()
{
    if(_active) {
        if(set_mempolicy(_previous_mode, _previous_nodes->maskp, _previous_nodes->size + 1) != 0) {
            UHD_LOG_WARNING("NUMA", "Unable to restore the previous NUMA memory policy: " + std::string(strerror(errno)));
        }
    }
    numa_free_nodemask(_previous_nodes);
}

// Reads a small sysfs attribute file and returns its contents with trailing
// whitespace (such as the newline) removed.
// Throws std::system_error with the errno from open()/read() on failure.
// Intended for reading numa nodes associated with network interfaces, double check if used for other purposes
static std::string read_sysfs_value(const std::string& path) {
    // Use open()/read() rather than C++ functions with they are not guaranteed to preserver errno
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) [[unlikely]] {
        throw std::system_error(errno, std::generic_category(), "Unable to open " + path);
    }

    // sysfs NUMA values are a few characters at most
    // This will need to be expanded if it is used for other purposes
    char buf[32];
    ssize_t bytes_read;
    do {
        bytes_read = ::read(fd, buf, sizeof(buf));
    } while (bytes_read < 0 && errno == EINTR);   // retry if interrupted by a signal

    // Recored errono before close() can overwrite it
    const int read_error = errno;
    ::close(fd);

    if (bytes_read < 0) [[unlikely]] {
        throw std::system_error(read_error, std::generic_category(), "Unable to read " + path);
    }

    // Copy the C string read to a C++ string
    std::string result(buf, static_cast<std::size_t>(bytes_read));
    // Remove trailing whitespace
    while(!result.empty()) {
        if(std::isspace(static_cast<unsigned char>(result.back()))) {
            result.pop_back();
        }
    }
    return result;
}

/*
 * Converts a string in base 10 to an int.
 *
 * Stricter than stoi which allows for unexpected trailing character.
 * 
 * @param value The string to convert
 * @param source The file that contained value. (Used of for a more informative error message)
 *
 * @return value converted to int
 *
 * @throw std::system_error value contains unexpected characters.
 *                          This is a system error because it is used when reading sysfs.
 *                          If this is repurposed if should be replaced with something more generic.
 */

static int parse_int(const std::string& value, const std::string& source) {
    int result = 0;
    const char* const first = value.data();
    const char* const last  = first + value.size();

    // Parse the string from first to last for a base 10 int and store the value in result
    const std::from_chars_result parsed = std::from_chars(first, last, result);

    //
    if (parsed.ec != std::errc() || parsed.ptr != last) [[unlikely]] {
        throw std::system_error(
            std::make_error_code(std::errc::invalid_argument),
            "Unable to parse " + source + ": '" + value + "'"
        );
    }
    return result;
}

// Returns the NUMA node the given network interface's device is attached to.
//
// Reads /sys/class/net/<iface>/device/numa_node. Assumes numa_available() has
// already been checked, since the result is validated against numa_max_node().
//
// Always returns a valid node index or throws std::system_error:
//   - errc::no_such_file_or_directory  interface doesn't exist, or it is virtual
//                                      (lo, bridge, VLAN, tun) and has no device/
//   - errc::invalid_argument           the file doesn't contain an integer
//   - errc::no_message_available       the kernel reports no NUMA affinity (-1)
//                                      or the value is out of range
//   - other errno values               any other open()/read() failure
int get_numa_node_for_iface(const std::string& iface) {
    const std::string path = "/sys/class/net/" + iface + "/device/numa_node";

    try {
        const std::string value = read_sysfs_value(path);
        const int node = parse_int(value, path);

        // The kernel writes -1 when the device's NUMA affinity can't be determined.
        if (node < 0 || node > numa_max_node()) [[unlikely]] {
            throw std::system_error(std::make_error_code(std::errc::no_message_available),
                                    path + " reports no valid NUMA node: '" + value + "'");
        }

        return node;
    } catch (const std::system_error& e) {
        // Catch any errors to print an error message specifying the interface then rethrow
        UHD_LOG_ERROR("NUMA", "Unable to determine NUMA node for interface " + iface + ": " + e.what());
        throw;
    }
}

} // namespace
