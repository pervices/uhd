//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/common/clock_sync.hpp>

// For getting the time on the host
#include <uhdlib/utils/system_time.hpp>

// For sending formatted error and warning message
#include <uhd/utils/log.hpp>

// For reducing the priority of the sync thread
#include <uhd/utils/thread.hpp>

using namespace uhd;
using namespace uhd::usrp;

static constexpr size_t padded_clock_sync_size = (size_t) ceil(sizeof(clock_sync) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;

//#define MEASURE_ACCURACY

clock_sync::clock_sync(std::string ip, uint16_t port, double tick_rate)
    :
    _tick_rate(tick_rate)
    {

    // Configure PID

    time_diff_pidc = uhd::pidc_tl(
        0.0, // desired set point is 0.0s error
        1.0, // measured K-ultimate occurs with Kp = 1.0, Ki = 0.0, Kd = 0.0
        // measured P-ultimate is inverse of 1/2 the flow-control sample rate
        2.0 / UPDATES_PER_SECOND
    );

    time_diff_pidc.set_error_filter_length( UPDATES_PER_SECOND );
    time_diff_pidc.set_max_error_for_convergence( 10e-6 );

    // Create port used to send/receive time diffs
    sync_socket = transport::udp_simple::make_connected(ip, std::to_string(port));

    sync_thread = std::thread(loop_thread_fn, this);
}

clock_sync::~clock_sync() {
    sync_thread_should_exit = true;
    _mm_sfence();
    sync_thread.join();
}

// Wait for convergence
void clock_sync::wait_for_sync() {
    for(
        time_spec_t time_then = uhd::get_system_time(),
            time_now = time_then
            ;
        (!is_synced())
            ;
        time_now = uhd::get_system_time()
    ) {
        if ( (time_now - time_then).get_full_secs() > 20 ) {
            UHD_LOG_ERROR("CLOCK_SYNC", "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling the device?");
            throw std::runtime_error( "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling the device?" );
        }
        ::usleep( 10000 );
    }
}

bool clock_sync::time_diff_recv(time_diff_resp & reply) {

    // Receive reply packet
    size_t bytes_received = sync_socket->recv( &reply, sizeof(reply));

    if(bytes_received > 0) {

        // Swap byte order from big to native
        // TODO: detect if we are using big or little endian at compile time, this currently assumes little endian
        reply.tv_sec = __builtin_bswap64(reply.tv_sec);
        reply.tv_tick = __builtin_bswap64(reply.tv_tick);
        return true;

    } else {
        // Error, no packet received
        return false;
    }
}

void clock_sync::reset_time_diff_pid() {
    _mm_mfence();

    uhd::time_spec_t reset_now = uhd::get_system_time();

    struct time_diff_resp reset_tdr;

    time_diff_send( reset_now );

    _mm_mfence();

    time_diff_recv( reset_tdr );

    double new_offset = (double) reset_tdr.tv_sec + (reset_tdr.tv_tick /  _tick_rate);

    time_diff_pidc.reset(reset_now, new_offset);

    _mm_sfence();
}

/// SoB Time Diff: feed the time diff error back into out control system
void clock_sync::time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now ) {

    static const double sp = 0.0;

    double pv = (double) tdr.tv_sec + (tdr.tv_tick / _tick_rate);

    double cv = time_diff_pidc.update_control_variable( sp, pv, now );

    bool reset_advised = false;

    bool time_diff_converged = time_diff_pidc.is_converged( now, &reset_advised );

    // We only update is_converged when it changes to avoid unnecessarily invalidating it and requiring the other core to fetch the new identical value
    // Record that convergance was gained
    if(time_diff_converged && !is_converged) [[unlikely]] {
        is_converged = true;
    // Record that convergance was lost
    } else if(!time_diff_converged && is_converged) [[unlikely]] {
        is_converged = false;
    }
    // Fence to ensure is_converged was updated
    _mm_sfence();

    if(reset_advised) {
        reset_time_diff_pid();
    }

    // For SoB, record the instantaneous time difference + compensation
    if (time_diff_converged ) {
        set_time_diff( cv );
    }
}

std::shared_ptr<clock_sync> clock_sync::make(std::string ip, uint16_t port, double tick_rate) {
    // Create using placement new
    clock_sync* raw_pointer = (clock_sync*) aligned_alloc(CACHE_LINE_SIZE, padded_clock_sync_size);
    new (raw_pointer) clock_sync(ip, port, tick_rate);

    std::shared_ptr<clock_sync> ptr(raw_pointer, deleter());

    return ptr;
}

void clock_sync::set_clock_sync_desired(bool desired) {
    clock_sync_desired = desired;
    _mm_sfence();
}

void clock_sync::loop_thread_fn( clock_sync *self ) {
#ifdef MEASURE_ACCURACY
    // The worst difference between the predicted and actual device time while synced
    static double worst_difference = 0;
#endif

    // Set thread priority to default since this isn't high priority
    uhd::set_thread_priority_safe(0, false);

    // Flag so that we only print the error message for failed recv once
    bool dropped_recv_message_printed = false;
    bool reply_failed = false;

    uhd::time_spec_t host_control_time, then, dt;
    struct timespec req, rem;

    struct time_diff_resp tdr;

    //Get initial offset
    self->reset_time_diff_pid();

    _mm_lfence();
    for(
        host_control_time = uhd::get_system_time(),
        then = host_control_time + UPDATE_PERIOD
        ;

        ! self->sync_thread_should_exit
        ;

        then += UPDATE_PERIOD,
        host_control_time = uhd::get_system_time()
    ) {
        if(self->is_resync_requested()) {
            // Record that the resync request has been ackcknowledged (also sets it as desynced)
            self->resync_acknowledge();
            // Reset PID to clear old values
            self->reset_time_diff_pid();
        }

        dt = then - host_control_time;
        if ( dt > 0.0 ) {
            // Wait until its time for the next sync packet if its in the future
            req.tv_sec = dt.get_full_secs();
            req.tv_nsec = dt.get_frac_secs() * 1e9;
            nanosleep( &req, &rem );
        } else {
            // Skip this request if we are late
            continue;
        }

        // Skip this round if a previous one failed and clock sync is not needed
        if(reply_failed && !self->clock_sync_desired) {
            continue;
        }

        double time_diff = self->time_diff_pidc.get_control_variable();

        // Start of fence to ensure that nothing get's reordered between getting the system time and sending the prediction
        _mm_mfence();

        // The time of the host when the time on device was predicted
        uhd::time_spec_t host_prediction_time = uhd::get_system_time();

        // The time we predict the device to have
        uhd::time_spec_t device_predicted_time = host_prediction_time + time_diff;

        // Send the predicted time
        self->time_diff_send( device_predicted_time );

        // End of fenced area to prevent time reordering
        _mm_mfence();

        // Get the predicted time minus the actual time
        bool reply_good =  self->time_diff_recv( tdr );

        // Update flag used to track if clock sync is working
        reply_failed = !reply_good;

        if (reply_good) {
            self->time_diff_process( tdr, host_prediction_time );

#ifdef MEASURE_ACCURACY
            if(self->is_synced()) {
                double difference = tdr.tv_sec + ( tdr.tv_tick / self->_tick_rate );
                worst_difference = std::max(difference, worst_difference);
            }
#endif

        // Print error message if clock sync matters and we haven't already done so
        } else if (!dropped_recv_message_printed && self->clock_sync_desired) {
            UHD_LOG_ERROR("CLOCK_SYNC", "Failed to receive packet used by clock synchronization");
                dropped_recv_message_printed = true;
        }
    }
#ifdef MEASURE_ACCURACY
    // 81us = 20% of a 65536 buffer at 162.5Msps (Crimson)
    // 26us = 20% of a 131072 buffer at 1Gsps (noDDR Cyan)
    // 8.7us = 20% of a 131072 buffer at 3Gsps (noDDR Cyan)
    // 1.8ms = 20% of a 4608000 buffer at 500Msps (Chestnut)
    UHD_LOG_INFO("CLOCK_SYNC", "The worst prediction while synced was: " + std::to_string(worst_difference));
#endif
}
