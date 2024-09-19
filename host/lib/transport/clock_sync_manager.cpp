// Copyright 2024 Per Vices Corporation

#include <uhd/transport/clock_sync_manager.hpp>

#include <uhd/exception.hpp>
#include <uhdlib/utils/system_time.hpp>
#include <uhd/utils/log.hpp>
#include <immintrin.h>

// Used to convert to/from network endianness
#include <boost/endian/conversion.hpp>

namespace uhd { namespace transport {

clock_sync_manager::clock_sync_manager(uhd::transport::udp_simple::sptr iface, std::shared_ptr<std::mutex> iface_mutex, double tick_rate)
:
_iface(iface),
_iface_mutex(iface_mutex),
_tick_rate(tick_rate),
should_exit(false),
pid_a(0.0, 1.0, 2.0/ UPDATE_RATE),
pid_b(0.0, 1.0, 2.0/ UPDATE_RATE),
active_pid(&pid_a)
{
    // Set the initial time difference between the device and host
    time_diff_resp tdr;
    uhd::time_spec_t host_time = uhd::get_system_time();
    time_diff_querry(&host_time, &tdr);
    uhd::time_spec_t initial_guess = uhd::time_spec_t(tdr.tv_sec, tdr.tv_tick, _tick_rate);
    pid_a.reset(&host_time, &initial_guess);
    pid_b.reset(&host_time, &initial_guess);

    clock_sync_thread = std::thread(clock_sync_loop, this);
}

clock_sync_manager::~clock_sync_manager() {
    should_exit = true;
    clock_sync_thread.join();
}

void clock_sync_manager::clock_sync_loop(clock_sync_manager* self) {
    // Theoretically there is a race condition here, but that would require get_predicted_time to take a full update period, which should never happen

    while(!self->should_exit) {
        update_pid(self, &self->pid_a);
        // Wait half of the update period before udpating the other PID
        usleep((1e6 /UPDATE_RATE) / 2);

        update_pid(self, &self->pid_b);
        // Wait half of the update period before udpating the other PID
        usleep((1e6 /UPDATE_RATE) / 2);
    }
}

void clock_sync_manager::update_pid(clock_sync_manager* self, pid* pid_to_update) {
    // Struct to store time diff replies in
    time_diff_resp tdr;

    uhd::time_spec_t host_time = uhd::get_system_time();
    uhd::time_spec_t predicted_time = pid_to_update->get_predicted_time(&host_time);

    bool querry_good = self->time_diff_querry(&predicted_time, &tdr);
    // Update PID if attempt to querry the time diff worked
    if(querry_good) [[likely]] {
        uhd::time_spec_t time_error = uhd::time_spec_t(tdr.tv_sec, tdr.tv_tick, self->_tick_rate);

        // Update the PID controller with the measured error
        pid_to_update->update(&host_time, &predicted_time, &time_error);
        // Ensures and writes to to pid_to_update are completed beffore setting it to be the PID used to process user requests
        _mm_sfence();
    }
    self->active_pid = pid_to_update;
    // Ensures active_pid is updated immediately so that pid_b is no longer being accessed when it started getting updated
    _mm_sfence();
}

bool clock_sync_manager::time_diff_querry( const uhd::time_spec_t* predicted_time, time_diff_resp* tdr) {
    time_diff_req pkt;

    // Sets the values used by the packet
    pkt.header = (uint64_t)0x20002 << 16;
    pkt.tv_sec = predicted_time->get_full_secs();
    pkt.tv_tick = (int64_t) (predicted_time->get_frac_secs() / _tick_rate);

    // Converts from the host (little endian) to the endianness expected by the device (big endian)
    boost::endian::native_to_big_inplace( pkt.header );
    boost::endian::native_to_big_inplace( pkt.tv_sec );
    boost::endian::native_to_big_inplace( pkt.tv_tick );

    // Lock iface
    _iface_mutex->lock();
    // Send packet
    _iface->send( &pkt, sizeof( pkt ) );
    // Get reply packet
    size_t r = _iface->recv( tdr, sizeof( *tdr ) );
    _iface_mutex->unlock();

    if(r == 0) {
        return false;
    } else {
        // Converts from the endianness used by the device the device (big endian) to the host (little endian)
        boost::endian::big_to_native_inplace( tdr->tv_sec );
        boost::endian::big_to_native_inplace( tdr->tv_tick );
        return true;
    }
}

clock_sync_manager::pid::pid(double setpoint, double Ku, double Pu)
:
_setpoint(setpoint),
Kp(Ku / 2.2),
Ki(Kp / (2.2 * Pu)),
Kd(Pu * (Pu / 6.3)),
// TODO: see if derivative_filter_frequency can be reduced
derivative_filter_frequency( Pu / 2.0 ),
// Create error_filter with 20 elements
error_filter(20)
{
    // Validate tuning parameters
    if(Ku < 0 || Pu < 0) {
        throw value_error( "Ku and Pu must be non-negative" );
    }
    // TODO: continue
}

void clock_sync_manager::pid::reset(const uhd::time_spec_t* const host_time, const uhd::time_spec_t* const offset) {
    // Clear PID memories
    integral_memory = 0;
    filtered_derivative = 0;

    // Clear error filter sma
    error_filter.reset();

    // Set the new initial guess
    _last_update_time = *host_time;
    _offset = *offset;
}

void clock_sync_manager::pid::get_predicted_time(const uhd::time_spec_t* const host_time) {

}

void clock_sync_manager::pid::update(const uhd::time_spec_t* const host_time, const uhd::time_spec_t* const predicted_time, const uhd::time_spec_t* const time_error) {
    // Difference in time between this and the previous update
    double dt = (*host_time - _last_update_time).get_real_secs();

    // Skip update if dt is to small to avoid numberical instability
    if ( dt < 1e-9 ) {
        UHD_LOG_ERROR("CLOCK_SYNC_MANAGER", "Time diff PID updates called to close together");
        return;
    }

    // Get the error (difference between predicted and actual)
    double error = time_error->get_real_secs() - _setpoint;

    // Updates the error filter
    // The error filter is used to check for convergence, not part of the PID controller itself
    error_filter.update(error);

    // Check if the error is so back
    if(error > INSTANTENOUS_MAX_ERROR) {
        // TODO: add check so this doesn't occur when setting the time
        if(converged) {
            UHD_LOG_ERROR("CLOCK_SYNC_MANAGER", "Sudden loss of clock sync");
        }
        converged = false;
    }

    // proportional
    double P = Kp * error;

    integral_memory += error * dt;
    // integral
    i += e * dt;
    double I = Ki * i;

    // Calculate the derivative
    // Noise can cause the derivative component to go haywire, to mitigate this an exponential low pass filter is apllied
    double alpha = 1 - exp(-derivative_filter_frequency*dt*2*M_PI);
    double derivative = (error - previous_error) / dt;
    // f(n) = (1-a) * f(n-1) + a * f(n)
    double filtered_derivated = (( 1 - alpha ) * filtered_derivated) + (alpha * derivative);

    double filtered_error = abs(error_filter.get_average());

    // derivative component
    double D = Kd * filtered_derivative;

    // Update predicted difference between the host and device
    difference = P + I + D;

    // if ( filtered_error >= RESET_THRESHOLD ) {
        // TODO: add way of indicates a reset is advised
    // }

    previous_error = error;

    // Update the flag to detect if clock sync is complete
    converged = std::abs(filtered_error) < FILTERED_MAX_ERROR;
}

}}
