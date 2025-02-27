//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

// Header file for classes related to clock sync between the host and device
// WIP: the clock sync loop itself is currently done by bm_thread_fn in the device's respective impl files

#pragma once

// Fixed width numbers
#include <stdint.h>
// Smart pointers
#include <memory>
// Fences
#include <immintrin.h>

namespace uhd { namespace usrp {

static constexpr size_t CACHE_LINE_SIZE = 64;
// TODO: set this dynamically at compile time
static constexpr size_t padded_clock_sync_shared_info_size = 128;//(size_t) ceil(sizeof(clock_sync_shared_info) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;

// Stores data shared between the clock sync thread and any other thread that requires the time
// Intened use:
// The consumer has a shared pointer to this class (possibly also a raw pointer if it impacts speed) since we need performance
// The provider has a weak pointer to this class since we can accept the overhead of getting a lock on it
class clock_sync_shared_info
{
private:

    // Stores if the predicted time and actual time have convered (clock sync completed)
    alignas(CACHE_LINE_SIZE) bool is_converged = false;
    // Stores if a resync has been requested
    bool resync_requested = true;
    // The difference between the device and host time in seconds
    // Put it on it's own cache line to avoid false sharing since it will be updated for often than the previous variables
    // TODO: verify alignas is working properly
    alignas(CACHE_LINE_SIZE) double time_diff = 0;

    // Declare constructor as private to ensure this is only created through make
    clock_sync_shared_info() {
        // Move this to a .cpp file if it becomes complicated
        /* No-op*/
    }

    // Deleter to be used by a shared_ptr
    struct deleter {
        /**
         * Destructs and frees self
         */
        void operator()(clock_sync_shared_info* self) {
            // The destructor must be manually called when using placement new
            self->~clock_sync_shared_info();
            free(self);
        }
    };

public:
    /**
     * Checks if the clocks are synchronized.
     * @return True if the host and device clocks are synchronized
     */
    inline bool is_synced() {
        return is_converged && !resync_requested;
    }

    /**
     * Gets the current time diff. Ensure is_synced is true before calling this
     * @return The current time diff
     */
    inline double get_time_diff() {
        return time_diff;
    }

    /**
     * Updates time diff. Only call this if the clocks are converged
     * @param new_time_diff The new time diff
     */
    inline void set_time_diff(double new_time_diff) {
        time_diff = new_time_diff;
        // Fence to ensure the time diff is set before marking it is converged
        _mm_sfence();
        is_converged = true;
        _mm_sfence();
    }

    /**
     * Sets the flag to indicate that a resync has been requested
     */
    inline void request_resync() {
        resync_requested = true;
        _mm_sfence();
    }

    /**
     * Sets the flag to indicate that a resync has been requested
     */
    inline bool is_resync_requested() {
        return resync_requested;
    }

    /**
     * Set flags to aknowledge a resync request has been received
     */
    inline void resync_acknowledge() {
        // Set convergence flag to false to indicate the process has been restarted
        is_converged = false;
        // Fence to ensure the is_converged flag is set before marking that the resync request has been processed
        _mm_sfence();
        resync_requested = false;
        _mm_sfence();
    }

    /**
     * Create a new clock_sync_shared_info with an associated shared pointer on it's own cache line
     * @return Shared pointer
     */
    static std::shared_ptr<clock_sync_shared_info> make();

};


}} // namespace uhd::usrp
