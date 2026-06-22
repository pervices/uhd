//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/types/time_spec_atomic.hpp>

using namespace uhd;

time_spec_atomic::time_spec_atomic(int64_t secs, int64_t ticks, double tick_rate) noexcept {
    store(secs, ticks, tick_rate);
}

void time_spec_atomic::store(int64_t secs, int64_t ticks, double tick_rate) {
    _secs = 0;
    _ticks = 0;
    _tick_rate = 0;
}
