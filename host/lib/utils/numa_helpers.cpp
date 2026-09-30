// Copyright 2024 Per Vices Corporation

// UHD
#include <uhdlib/utils/numa_helpers.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/log.hpp>

// libnuma
#include <numaif.h>

// libc
#include <cerrno>
#include <cstring>
#include <string>

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

}
