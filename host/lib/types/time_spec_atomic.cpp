//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/types/time_spec_atomic.hpp>

using namespace uhd;

time_spec_atomic::time_spec_atomic(int64_t secs, int64_t ticks, double tick_rate) noexcept {
    write_count.store(0, std::memory_order_release);
    store(secs, ticks, tick_rate);
}

void time_spec_atomic::store(int64_t secs, int64_t ticks, double tick_rate) {
    write_count.fetch_add(1, std::memory_order_release);
    _secs = secs;
    _ticks = ticks;
    _tick_rate = tick_rate;
    write_count.fetch_add(1, std::memory_order_release);
}

time_spec_atomic time_spec_atomic::load() {
    int64_t intial_write_count = 0;
    int64_t end_write_count = -1;
    int64_t target_seconds;
    int64_t target_ticks;
    do {
        intial_write_count = write_count.load(std::memory_order_acquire);
        target_seconds = _secs;
        target_ticks = _ticks;

         end_write_count= write_count.load(std::memory_order_acquire);

    } while (
        // Repeat load if the class was updated while loading
        intial_write_count != end_write_count ||
        // Repeat load if the class is mid update (write_count is odd)
        (intial_write_count & 0x1) );

    return time_spec_atomic(target_seconds, target_ticks, _tick_rate);
}
