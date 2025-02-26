//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

// Header file for classes related to clock sync between the host and device
// WIP: the clock sync loop itself is currently done by bm_thread_fn in the device's respective impl files

#pragma once

#include <stdint.h>

namespace uhd { namespace usrp {

class clock_sync_shared_info
{
private:
    static constexpr size_t CACHE_LINE_SIZE = 64;

    // Stores if the predicted time and actual time have convered (clock sync completed)
    alignas(CACHE_LINE_SIZE) bool is_converged;
    bool resync_requested;
    // The difference
    alignas(CACHE_LINE_SIZE) double diff;
};

}} // namespace uhd::usrp
