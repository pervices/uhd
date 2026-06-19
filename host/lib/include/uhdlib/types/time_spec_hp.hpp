//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstdint>

namespace uhd::usrp {
    /**
     * A high performance integer only timespec that supports atomic operations
     */
    class alignas(16) time_spec_hp{
        // The seconds portion of time
        int64_t seconds;
        // The tick portion of time
        int64_t ticks;
        // The tick rate in Hz
        double tick_rate;

    };
}
