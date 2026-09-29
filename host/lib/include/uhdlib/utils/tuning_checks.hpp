//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstddef>
#include <sched.h>
#include <string>

namespace uhd {

// A collection of functions for checking if the program is tuned properly

// _safe indicates the function doesn't throw an exception.
// _safe is included in case we want to add a version that throw exceptions in the future.
/**
 * Check if the provided affinity mask will keep the network sockets on the correct NUMA node.
 * Prints a warning if the mask will allow a thread to run on a different NUMA node than the sockets.
 *
 * @param affinity_mask The mask indicating which cores the thread being checked can use.
 * @param socket_fd An array of file descriptors for the sockets to check. They must be AF_INET sockets.
 * @param socket_fd_len The number of elements in socket_fd.
 * @param message_prefix A message to prepend to log/print messages for clarity about who is calling this check.
 *
 * @return Return 0 if affinity_mask will keep a thread on the correct NUMA node for socket_fd or if the system only has 1 NUMA node (not applicable). Return a positive value if the NUMA node of the sockets does not match the affinity mask or the mask spans multiple nodes. Return a negative value if the check itself failed.
 */
int check_numa_safe(const cpu_set_t affinity_mask, int socket_fd[], size_t socket_fd_len, std::string message_prefix = "");

/**
 * Check if NUMA is enabled on the system.
 * 
 * Prints a warning if numa cannot be checked.
 *
 * @return Return true if multiple NUMA nodes exist. Returns false if there is either 1 NUMA node or the check for the number of numa nodes failed.
 */
bool numa_relevant();

/**
 * Add the numa node netowrk interface used by the specified socket to the mask.
 *
 * @param socket_fd The socket to get the numa mask for.
 * @param node_mask The mask to OR the node used by the socket with/store the result.
 *
 * @throw std::system_error Throw this error if the kernel does not support NUMA
 * @throw TODO
 */
void add_numa_mask_of_socket(int socket_fd, bitmask* node_mask);

}; /* namespace uhd */
