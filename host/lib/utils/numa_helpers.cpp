// Copyright 2024 Per Vices Corporation

#include <uhdlib/utils/numa_helpers.hpp>
#include <uhd/utils/log.hpp>

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

}
