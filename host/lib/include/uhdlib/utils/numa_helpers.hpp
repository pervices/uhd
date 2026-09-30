// Copyright 2024 Per Vices Corporation
#pragma once

// libc
#include <memory>

// libnuma
#include <numa.h>

namespace uhd {

/**
 * Check if NUMA is both enabled in the kernel and relevant (more than 1 node)
 *
 * This function is not optimized for speed with multiple nodes.
 * If calling this from the critical path you must cache the result in your node.
 *
 * @return Return true if the kernel supports NUMA and more than 1 node exists.
 */
bool is_numa_relevant();

}
