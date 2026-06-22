//
// Copyright 2026 Per Vices Corporation
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
// Standard library for thread
#include <thread>
// PID controller for clock sync
#include "pidc.hpp"
// Data type used within UHD for storing time
#include <uhd/types/time_spec.hpp>
// Utility forgetting host time
#include <uhdlib/utils/system_time.hpp>
// Class for managing single writer multiple readers
#include <uhdlib/utils/swmr.hpp>

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

/**
 * Manages synchronizing clocks between host and device
 */
class alignas(CACHE_LINE_SIZE) clock_sync
{
private:
    // How many times per second clock sync is checked
    static constexpr double UPDATES_PER_SECOND = 100;
    static constexpr double UPDATE_PERIOD = 1.0 / UPDATES_PER_SECOND;

    /**
     * Start of variables that will frequently be accessed by the critical thread
     * NOTE: they are at the start of the class so that variables not needed by the ciritcal thread act as padding to prevent false sharing from unrelated data
     */

    /**
     * Diference between the host and device time in seconds
     * This isn't actually used by the critical thread, but it's constant so it's fine to have it there
     */
    alignas(CACHE_LINE_SIZE) const double _tick_rate;

    // Tick rate of time diff packets
    swmr<time_spec_t> time_diff = swmr<time_spec_t>(time_spec_t(0.0));

    // Stores if the predicted time and actual time have convered (clock sync completed)
    volatile bool is_converged = false;

    // Stores if a resync has been requested
    volatile bool resync_requested = true;

    // Stores if setting time is in progress
    volatile bool set_time_in_progress = false;

    /*
     * Start of member vaiables that are not used by the critical thread
     * They must be on a separate cache line from variables used by the critical thread but otherwise don't matter
     */

    // NOTE: at present the PID is only used in the sync thread. If will need to be aligned and padded to cache lines if it is changes so that the critical thread(s) access it directly
    /** PID controller that rejects differences between Crimson's clock and the host's clock.
     *  -> The Set Point of the controller (the desired input) is the desired error between the clocks - zero!
     *  -> The Process Variable (the measured value), is error between the clocks, as computed by Crimson.
     *  -> The Control Variable of the controller (the output) is the required compensation for the host
     *     such that the error is forced to zero.
     *     => Crimson Time Now := Host Time Now + CV
     */
    alignas(CACHE_LINE_SIZE) uhd::pidc time_diff_pidc;

    // The socket used to send and receive time diffs
    transport::udp_simple::sptr sync_socket;

    std::thread sync_thread;

    // The seconds part of the last time the user attempted to set the time on the device
    volatile int64_t last_time_set_seconds = INT64_MIN;

    // Tells the sync thread to exit
    volatile bool sync_thread_should_exit = false;

    // Indicates that clock sync matters
    volatile bool clock_sync_desired = false;

    /*
     * Start of variables set during the constructor.
     * They must be a sparate cache line from other stuff for false sharing but can be on the same one as each other.
     */

    /**
     * Create an instance of clock_sync.
     * The constructor is private to ensure this is only created through make.
     * @param ip The ip address to perform clock sync with
     * @param port The port the device receives clock sync messages on
     * @param tick_rate The tick rate the device expects to time diff packets
     */
    clock_sync(std::string ip, uint16_t port, double tick_rate);

    ~clock_sync();

    // Deleter to be used by a shared_ptr
    struct deleter {
        /**
         * Destructs and frees self
         */
        void operator()(clock_sync* self) {
            // The destructor must be manually called when using placement new
            self->~clock_sync();
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
        request.tv_tick = (int64_t) ( (prediction.get_frac_secs() * _tick_rate) );

        // Convert request from native little endian to big endian for the FPGA
        // TODO: detect if we are using big or little endian at compile time
        request.header = __builtin_bswap64(request.header);
        request.tv_sec = __builtin_bswap64(request.tv_sec);
        request.tv_tick = __builtin_bswap64(request.tv_tick);

        sync_socket->send(&request, sizeof(request));
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

    /**
     * Updates time diff. Only call this if the clocks are converged
     * @param new_time_diff The new time diff
     */
    inline void set_time_diff(time_spec_t new_time_diff) {
        time_diff.store(new_time_diff);
        // Fence to ensure the time diff is set before marking it is converged
        _mm_sfence();
        is_converged = true;
        _mm_sfence();
    }

    /**
     * Gets the flag to indicate that a resync has been requested
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
        _mm_sfence();

        // Fence to ensure the is_converged flag is set before marking that the resync request has been processed
        resync_requested = false;
        _mm_sfence();
    }

public:
    /**
     * Checks if the clocks are synchronized.
     * @return True if the host and device clocks are synchronized
     */
    inline bool is_synced() {
        _mm_lfence();
        return is_converged && !resync_requested;
    }

    /**
     * Waits until the clocks are synced
     * @throws std::runtime_error Waiting for clock sync has timed out
     */
    void wait_for_sync();

    /**
     * Tells this that setting time has been initiated.
     * It is used to indicate than any time values will be inaccurate
     * @param planned_time_s The time the caller plans on setting the time to in seconds
     */
    inline void set_time_initiated(int64_t planned_time_s) {
        set_time_in_progress = true;
        resync_requested = true;

        // This could be set during set_time_finished, but it is set here to avoid additional fences to set it before clearing set_time_in_progress
        last_time_set_seconds = planned_time_s;

        _mm_sfence();
    }

    /**
     * Tells this that setting time has been compelted.
     * Clock sync can be resumed
     */
    inline void set_time_finished() {
        set_time_in_progress = false;
        resync_requested = true;
        _mm_sfence();
    }

    /**
     * Sets whether or not clock sync is needed.
     * When clock sync is not needed error messages are supressed an
     * @param desired True indicates clock sync is needed
     */
    void set_clock_sync_desired(bool desired);

    /**
     * Gets the time that a packet sent now would arrive at the device
     */
    inline time_spec_t get_device_time() {
        // The fence inside is_synced is sufficient so we don't need one here

        // Wait for clock sync to finish
        if(!is_synced()) [[likely]] {
            wait_for_sync();
        }

        _mm_lfence();

        time_spec_t current_time_diff;
        time_diff.load(&current_time_diff);

        return uhd::get_system_time() + current_time_diff;
    }

    /**
     * Create a new clock_sync with an associated shared pointer on it's own cache line
     * @param ip The ip address to perform clock sync with
     * @param port The port the device receives clock sync messages on
     * @param tick_rate The tick rate the device expects to time diff packets
     * @return Shared pointer to the newly created instance
     */
    static std::shared_ptr<clock_sync> make(std::string ip, uint16_t port, double tick_rate);

    /**
     * The loop to run clock sync in
     * @param self The class to run clock sync for. A raw pointer is used because the deleter is responsible for ending the thread.
     */
    static void loop_thread_fn(clock_sync *self);

};


}} // namespace uhd::usrp
