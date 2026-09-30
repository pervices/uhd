// Copyright 2024 Per Vices Corporation
#pragma once

// libc
#include <string>

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

/**
 * Binds memory (MPOL_PREFERRED) allocated by the calling thread to the given NUMA node(s) for the lifetime of this object,
 * then restores the thread's previous memory policy.
 *
 * Pages are placed on the node when first written to. Make sure to write to pages while this is active.
 * Threads created while this is in scope inherit the binding and keep it after this goes out of scope.
 * Only affects memory placement, not which CPUs the thread runs on.
 * Does nothing if node_mask is null.
 * Use is_active to check if the bind was successfull. It will awlays fail with kernel <5.15 and multiple nodes
 */
class scoped_numa_membind {
public:
    /**
     * @param node_mask The NUMA node(s) to bind to. Do nothing when it is null or empty.
     *                  Must have only one node.
     *
     * @throw std::system_error The kernel does not support NUMA.
     */
    explicit scoped_numa_membind(const bitmask* node_mask);

    ~scoped_numa_membind();

    scoped_numa_membind(const scoped_numa_membind&) = delete;
    scoped_numa_membind& operator=(const scoped_numa_membind&) = delete;

    /**
     * Returns true if the memory bind was successfull.
     *
     * 
     *
     * @return Returns true if memory bind was successfull
     */
    inline bool is_active() {
        return _active;
    }

private:
    // The memory policy to restore on destruction
    int _previous_mode;
    /**
     * The nodes to restore on destruction.
     * Make sure to call numa_free_nodemask on exit or where hte constructor throws after it's creation
     */
    bitmask* _previous_nodes;

    // Whether the binding was applied and needs to be undone
    bool _active = false;
};

/**
 * Returns the NUMA node the given network interface's device is attached to,
 * read from /sys/class/net/<iface>/device/numa_node.
 *
 * Assumes numa_available() has already been checked, since the result is
 * validated against numa_max_node().
 *
 * @param iface The name of the network interface (e.g. "eth0").
 *
 * @return A valid NUMA node index. Never returns -1 or > numa_max_node().
 *
 * @throw std::system_error The NUMA node could not be determined. The error code is one of:
 *                          - errc::no_such_file_or_directory: the interface doesn't exist,
 *                            or it is virtual (lo, bridge, VLAN, tun) and has no device/.
 *                          - errc::invalid_argument: the file doesn't contain an integer.
 *                          - errc::no_message_available: the kernel reports no NUMA affinity
 *                            (-1), or the value is out of range.
 *                          - Any other errno value from a failed open() or read().
 */
int get_numa_node_for_iface(const std::string& iface);

} // namespace uhd