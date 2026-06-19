//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstdint>

namespace uhd {
    /**
     * A high performance integer only timespec that supports atomic operations
     */
    class alignas(16) time_spec_hp{
        // The seconds portion of time
        const int64_t _seconds;
        // The tick portion of time
        const int64_t _ticks;
        // The tick rate in Hz
        const double _tick_rate;

        /**
         * Creates a new instance of the class
         * @param seconds The seconds portion of time
         * @param ticks The tick portion of time
         * @param tick_rate The tick rate
         */
        time_spec_hp(int64_t seconds, int64_t ticks, double tick_rate);

        /**
        * Delete all operators so that only manually created atomic ones can be used
        */

        // Copy constructor
        time_spec_hp(const time_spec_hp&) = delete;

        // Copy assignment
        time_spec_hp& operator=(const time_spec_hp&) = delete;

        // Move constructor
        time_spec_hp(time_spec_hp&&) = delete;

        // Move assignment
        time_spec_hp& operator=(time_spec_hp&&) = delete;

        // Delete all comparisson operators
        bool operator==(const time_spec_hp&) const = delete;
        bool operator!=(const time_spec_hp&) const = delete;
        bool operator<(const time_spec_hp&) const = delete;
        bool operator<=(const time_spec_hp&) const = delete;
        bool operator>(const time_spec_hp&) const = delete;
        bool operator>=(const time_spec_hp&) const = delete;
    };
}
