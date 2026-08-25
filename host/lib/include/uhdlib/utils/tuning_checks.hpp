//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstddef>
#include <sched.h>

namespace uhd {

// A collection of functions for checking if the program is tuned properly

/**
 * Check if the provided affinity mask will keep the network sockets on the correct NUMA node.
 * Prints a warning if the mask will allow a thread to run on a different NUMA node than the sockets.
 *
 * @param affinity_mask The mask indicating which cores the thread being checked can use.
 * @param socket_fd An array of file descriptors for the sockets to check. They must be AF_INET sockets.
 * @param socket_fd_len The number of elements in socket_fd.
 *
 * @throws system_error TODO: remove throwing errors
 *
 * @return Return 0 if affinity_mask will keep a thread on the correct NUMA node for socket_fd. Return a positive value if the NUMA node of the sockets does not match the affinity mask or the mask spans multiple nodes. Return a negative value if the check itself failed.
 */
int check_numa(const cpu_set_t affinity_mask, int socket_fd[], size_t socket_fd_len);

}; /* namespace uhd */
