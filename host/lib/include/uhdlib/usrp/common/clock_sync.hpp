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
// Socket for sending/receiving time diff packets
#include <uhd/transport/udp_simple.hpp>

#include "pidc.hpp"

#include <uhd/types/time_spec.hpp>

namespace uhd { namespace usrp {

static constexpr size_t CACHE_LINE_SIZE = 64;

#pragma pack(push,1)
// Time diff requests to the device
struct time_diff_req {
    uint64_t header;
    int64_t tv_sec;
    int64_t tv_tick;
};
#pragma pack(pop)

#pragma pack(push,1)
// Time diff replies from the device
struct time_diff_resp {
    int64_t tv_sec;
    int64_t tv_tick;
};
#pragma pack(pop)

// Stores data shared between the clock sync thread and any other thread that requires the time
// Intened use:
// The consumer has a shared pointer to this class (possibly also a raw pointer if it impacts speed) since we need performance
// The provider has a weak pointer to this class since we can accept the overhead of getting a lock on it
class clock_sync_shared_info
{
private:

    static constexpr double UPDATES_PER_SECOND = 100;
    // TODO: make UPDATE_PERIOD a constexpr (will require changes to the class)
    static inline uhd::time_spec_t UPDATE_PERIOD = uhd::time_spec_t(1.0/UPDATES_PER_SECOND);

    // Stores if the predicted time and actual time have convered (clock sync completed)
    alignas(CACHE_LINE_SIZE) bool is_converged = false;
    // Stores if a resync has been requested
    bool resync_requested = true;
    // The difference between the device and host time in seconds
    // Put it on it's own cache line to avoid false sharing since it will be updated for often than the previous variables
    // TODO: verify alignas is working properly
    alignas(CACHE_LINE_SIZE) double time_diff = 0;

    // TODO: ensure all variable after this line are aligned to avoid false sharing

    bool sync_thread_running = false;
    bool sync_thread_should_exit = false;
    // TODO: create in constructor
    uhd::transport::udp_simple::sptr sync_port;

    // TODO: set this in the constructor
    double tick_rate = 162.5e6;

    // TODO: have the device set this to true when we know it is needed
    bool clock_sync_desired = true;

    /** PID controller that rejects differences between Crimson's clock and the host's clock.
     *  -> The Set Point of the controller (the desired input) is the desired error between the clocks - zero!
     *  -> The Process Variable (the measured value), is error between the clocks, as computed by Crimson.
     *  -> The Control Variable of the controller (the output) is the required compensation for the host
     *     such that the error is forced to zero.
     *     => Crimson Time Now := Host Time Now + CV
     */
    uhd::pidc time_diff_pidc;

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

    /**
     * Send a packet with the predicted time.
     * Send is inline but not recv because send may be sent from a critical thread, recv is not time sensitive
     * @param prediction The predicted time on the device
     */
    inline void time_diff_send( uhd::time_spec_t prediction ) {
        time_diff_req request;

        // Create request
        request.header = (uint64_t)0x20002 << 16;
        request.tv_sec = prediction.get_full_secs();
        request.tv_tick = (int64_t) ( prediction.get_frac_secs() * 1e9 / tick_rate );

        // Convert request from native little endian to big endian for the FPGA
        // TODO: detect if we are using big or little endian at compile time
        request.header = __builtin_bswap64(request.header);
        request.tv_sec = __builtin_bswap64(request.tv_sec);
        request.tv_tick = __builtin_bswap64(request.tv_tick);

        sync_port->send(&request, sizeof(request));
    }

    /**
     * Receives a packet a packet containing prediction - actual
     * @param reply Where to store hte reply
     * @return Returns true on success, false on failure
     */
    bool time_diff_recv(time_diff_resp & reply);

    /**
     * Resets the time diff pid.
     * This function will reset the time diff pid and create a fresh prediction
     */
    void reset_time_diff_pid();

    /**
     * Updates the pid controller for predicting the time
     * @param tdr The reply containing the difference between the predicted and actual time
     * @param now The time on the host when the request was send
     */
    void time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & request_time );

public:
    /**
     * Checks if the clocks are synchronized.
     * @return True if the host and device clocks are synchronized
     */
    inline bool is_synced() {
        return is_converged && !resync_requested;
    }

    /**
     * Waits until the clocks are synced
     * @throws std::runtime_error Waiting for clock sync has timed out
     */
    void wait_for_sync();

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
     * @return Shared pointer to the newly created instance
     */
    static std::shared_ptr<clock_sync_shared_info> make();

    /**
     * The loop to run clock sync in
     * @param self The class to run clock sync for. A raw pointer is used because the deleter is responsible for ending the thread.
     */
    static void loop_thread_fn(clock_sync_shared_info *self);

};


}} // namespace uhd::usrp
