//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstdint>
#include <atomic>

namespace uhd {


class time_spec_atomic {
    // The seconds portion of the time
    int64_t _secs;
    // The ticks portion of the time
    int64_t _ticks;
    /**
     * The tick rate
     * TODO: make constant
     */
    double _tick_rate;

    /**
     * The number of times the class has been written to
     * Store operations increment the count by 1 at the start and end.
     * If it's odd a write is in progress
     */
    std::atomic<int64_t> write_count;

    // --- 1. LIFE-CYCLE CONSTRUCTORS ---

    /**
     * The default initialize to 0
     */
    explicit time_spec_atomic() noexcept
    :
    time_spec_atomic(0, 0, 0)
    {}


    explicit time_spec_atomic(int64_t secs, int64_t ticks, double tick_rate) noexcept;

    /**
     * NOTE: this is thread safe against loads, not against multiple writers
     */
    void store(int64_t secs, int64_t ticks, double tick_rate);

    time_spec_atomic load();

    // --- 2. DELETED STRUCTURAL COPY/MOVE ---
    // Moving or copying multi-member structures results in data tearing
    time_spec_atomic(const time_spec_atomic&) = delete;
    time_spec_atomic(time_spec_atomic&&) = delete;

    // --- 3. DELETED COMPILER-GENERATED DEFAULT ASSIGNMENTS & CASTS ---
    // Prevents direct raw object assignments or dangerous structural downgrades
    time_spec_atomic& operator=(const time_spec_atomic&) = delete;

    // --- 4. DELETED MEMORY POINTER ALIASING ---
    // Direct addresses let threads modify individual fields bypassing internal sync
    time_spec_atomic* operator&() = delete;
    const time_spec_atomic* operator&() const = delete;
    time_spec_atomic* operator->() = delete;
    time_spec_atomic& operator*() = delete;

    // --- 5. DELETED READ-MODIFY-WRITE (RMW) OPERATORS ---
    // Math operations on a multi-type structure are inherently unaligned and unsafe
    time_spec_atomic operator++(int) = delete;
    time_spec_atomic operator--(int) = delete;
    time_spec_atomic& operator++()    = delete;
    time_spec_atomic& operator--()    = delete;

    time_spec_atomic& operator+=(const time_spec_atomic&) = delete;
    time_spec_atomic& operator-=(const time_spec_atomic&) = delete;

    // --- 6. DELETED CONDITIONAL EVALUATIONS ---
    // Block multi-step branches which cause race conditions
    bool operator==(const time_spec_atomic&) const  = delete;
    bool operator!=(const time_spec_atomic&) const  = delete;
    bool operator<(const time_spec_atomic&) const   = delete;
    bool operator<=(const time_spec_atomic&) const  = delete;
    bool operator>(const time_spec_atomic&) const   = delete;
    bool operator>=(const time_spec_atomic&) const  = delete;
};

} // namepspace uhd
