//
// Copyright 2025 Per Vices Corporation
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

static constexpr size_t padded_clock_sync_shared_info_size = (size_t) ceil(sizeof(clock_sync_shared_info) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;

clock_sync_shared_info::clock_sync_shared_info(std::string ip, uint16_t port, double tick_rate)
    :
    _tick_rate(tick_rate)
    {

    // Configure PID
    time_diff_pidc.set_error_filter_length( UPDATES_PER_SECOND );
    time_diff_pidc.set_max_error_for_convergence( 10e-6 );

    // Create port used to send/receive time diffs
    sync_socket = transport::udp_simple::make_connected(ip, std::to_string(port));

    sync_thread = std::thread(loop_thread_fn, this);
}

clock_sync_shared_info::~clock_sync_shared_info() {
    sync_thread_should_exit = true;
    _mm_sfence();
    sync_thread.join();
}

// Wait for convergence
void clock_sync_shared_info::wait_for_sync() {
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

bool clock_sync_shared_info::time_diff_recv(time_diff_resp & reply) {

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

void clock_sync_shared_info::reset_time_diff_pid() {
    UHD_LOG_ERROR("CLOCK_SYNC", "Resetting time diff");
    uhd::time_spec_t reset_now = uhd::get_system_time();

    struct time_diff_resp reset_tdr;

    time_diff_send( reset_now );
    time_diff_recv( reset_tdr );

    double new_offset = (double) reset_tdr.tv_sec + (reset_tdr.tv_tick /  _tick_rate);

    time_diff_pidc.reset(reset_now, new_offset);
}

/// SoB Time Diff: feed the time diff error back into out control system
void clock_sync_shared_info::time_diff_process( const time_diff_resp & tdr, const uhd::time_spec_t & now ) {

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
        if(reset_advised) {
            UHD_LOG_ERROR("CLOCK_SYNC", "Reset advised");
        }
        reset_time_diff_pid();
    }

    // For SoB, record the instantaneous time difference + compensation
    if (time_diff_converged ) {
        set_time_diff( cv );
    }
}

std::shared_ptr<clock_sync_shared_info> clock_sync_shared_info::make(std::string ip, uint16_t port, double tick_rate) {
    // Create using placement new
    clock_sync_shared_info* raw_pointer = (clock_sync_shared_info*) aligned_alloc(CACHE_LINE_SIZE, padded_clock_sync_shared_info_size);
    new (raw_pointer) clock_sync_shared_info(ip, port, tick_rate);

    std::shared_ptr<clock_sync_shared_info> ptr(raw_pointer, deleter());

    return ptr;
}

void clock_sync_shared_info::set_clock_sync_desired(bool desired) {
    clock_sync_desired = desired;
    _mm_sfence();
}

static int64_t num_time_diffs = 0;

void clock_sync_shared_info::loop_thread_fn( clock_sync_shared_info *self ) {
    UHD_LOG_ERROR("CLOCK_SYNC", "Synce thread running");

    // Set thread priority to default since this isn't high priority
    uhd::set_thread_priority_safe(0, false);

    // Flag so that we only print the error message for failed recv once
    bool dropped_recv_message_printed = false;

    uhd::time_spec_t now, then, dt;
    uhd::time_spec_t crimson_now;
    struct timespec req, rem;

    struct time_diff_resp tdr;

    //Get offset
    now = uhd::get_system_time();
    self->time_diff_send( now );
    self->time_diff_recv( tdr );
    self->time_diff_pidc.set_offset((double) tdr.tv_sec + (tdr.tv_tick / self->_tick_rate));

    UHD_LOG_ERROR("CLOCK_SYNC", "A1");

    _mm_lfence();
    for(
        now = uhd::get_system_time(),
        then = now + UPDATE_PERIOD
        ;

        ! self->sync_thread_should_exit
        ;

        then += UPDATE_PERIOD,
        now = uhd::get_system_time()
    ) {
        if(self->is_resync_requested()) {
            // Record that the resync request has been ackcknowledged (also sets it as desynced)
            self->resync_acknowledge();
            // Reset PID to clear old values
            self->reset_time_diff_pid();
        }

        dt = then - now;
        if ( dt > 0.0 ) {
            req.tv_sec = dt.get_full_secs();
            req.tv_nsec = dt.get_frac_secs() * 1e9;
            nanosleep( &req, &rem );
        } else {
            continue;
        }

        double time_diff = self->time_diff_pidc.get_control_variable();

        _mm_mfence();

        now = uhd::get_system_time();
        crimson_now = now + time_diff;

        // Send the predicted time
        self->time_diff_send( crimson_now );
        // Get the difference between the predicted and real time
        bool reply_good =  self->time_diff_recv( tdr );

        _mm_mfence();

        UHD_LOG_ERROR("CLOCK_SYNC", "crimson_now.get_real_secs(): " + std::to_string(crimson_now.get_real_secs()));
        UHD_LOG_ERROR("CLOCK_SYNC", "tdr.tv_sec(): " + std::to_string(tdr.tv_sec));
        UHD_LOG_ERROR("CLOCK_SYNC", "tdr.tv_tick(): " + std::to_string(tdr.tv_tick));

        if(!reply_good) {
                UHD_LOG_ERROR("CLOCK_SYNC", "recv clock sync error");
        }

        if (reply_good) {
            self->time_diff_process( tdr, now );
            num_time_diffs++;
        } else if (!dropped_recv_message_printed && self->clock_sync_desired) {
            // TODO: give up if sync failed and clock_sync_desired is false
            UHD_LOG_ERROR("CLOCK_SYNC", "Failed to receive packet used by clock synchronization");
            dropped_recv_message_printed = true;
        }
        // lfence to update _bm_thread_should_exit for the for loop
        _mm_lfence();
    }
}
