// Copyright 2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <atomic>
#include <thread>
#include <mutex>

#include <uhd/types/time_spec.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/utils/sma.hpp>

namespace uhd { namespace transport {

/**
 * Calculates the time a packet sent over an SFP port will arrive at it's destination.
 */
class clock_sync_manager {

public:

    /**
     * Create a clock_sync_manager.
     * @param iface A wrapper for the socket used to communicate over the SFP port.
     * @param iface_mutex A mutex used to control access to iface.
     * @param tick_rate The tick rate used by the device's time diff packets.
     */
    clock_sync_manager(uhd::transport::udp_simple::sptr iface, std::shared_ptr<std::mutex> iface_mutex, double tick_rate);

    /**
     * Destructs clock_sync_manager.
     * Stops the thread clock synchronization is running it
     */
    ~clock_sync_manager();

    /**
     * Gets the time a packet sent now over the the SFP port would arrive at the device. Will block if clock synchronization is not complete.
     * @return the predicted time on the device.
     */
    uhd::time_spec_t get_device_time_fine();

    /**
     * Gets the time a packet sent now over the the SFP port would arrive at the device. If clock sync is not complete it will fall back to getting the time over iface, which is less accurate.
     * @return the predicted time on the device.
     */
    uhd::time_spec_t get_device_time_coarse();

private:
    /**
     * How of clock sync updates in Hz
     */
    static constexpr double UPDATE_RATE = 100;

    #pragma pack(push,1)
    /**
     * Time diff request packet
     */
    struct time_diff_req {
        uint64_t header;
        int64_t tv_sec;
        int64_t tv_tick;
    };
    #pragma pack(pop)

    #pragma pack(push,1)
    /**
     * Time diff reply packet.
     * Contains the difference between the time predicted in time_diff_req and the actual time.
     */
    struct time_diff_resp {
        int64_t tv_sec;
        int64_t tv_tick;
    };
    #pragma pack(pop)
    /**
     * A PID controller used for clock synchronization
     */

    class pid {

    public:
        /**
         * Create a Tyreus-Luyben Tuned PID Controller.
         * An alternative, more stable tuning than Ziegler-Nichols.
         * @param setpoint The target value
         * @param Ku Ultimate gain
         * @param Pu Ultimate period
         */
        pid(double setpoint, double Ku, double Pu);

        /**
         * Updates the PID controller.
         * @param host_time The time on the host computer when the time diff prediction was made. No direct relationship with the time on the device.
         * @param predicted_time The predicted time on the device.
         * @param reply The difference between the predicted time and the actual time
         */
        void update(const uhd::time_spec_t* const host_time, const uhd::time_spec_t* const predicted_time, const uhd::time_spec_t* const time_error);

        /**
         * Reset the PID.
         * @param host_time Time on the host when initial guess was made
         * @param initial_guess Initial guess for the difference in time between the host and the device
         */
        void reset(const uhd::time_spec_t* const host_time, const uhd::time_spec_t* const initial_guess);

        /**
         * Check if the PID has converged.
         * @return Return true if synchronization is complete.
         */
        inline bool is_converged() {
            return converged;
        }

        /**
         * Gets the predicted time on the device.
         * @param host_time The time on the host system
         */
        uhd::time_spec_t get_predicted_time(const uhd::time_spec_t* host_time);

    private:
        /**
         * Threshold for marking the clock as diverged from a single error measurement.
         */
        static constexpr double INSTANTENOUS_MAX_ERROR = 20e-6;
        /**
         * Threshold for marking the clock as diverged from filtered error measurements.
         */
        // TODO: fine tune this value, it can probably be reduced
        static constexpr double FILTERED_MAX_ERROR = 10e-6;

        /**
         * The target value
         */
        double _setpoint;
        /**
         * Proportional gain.
         */
        double Kp;
        /**
         * Integral gain.
         */
        double Ki;
        /**
         *
         */
        double integral_memory = 0;
        /**
         * Derivative gain.
         */
        double Kd;
        /**
         * The frequency of the low pass filter used to prevent instability with derivative gain
         */
        double derivative_filter_frequency;
        /**
         * The derivate from the
         */
        double filtered_derivative = 0;
        /**
         * Difference between host and device time when clock sync started.
         */
        uhd::time_spec_t _offset;
        /**
         * The time of the last pid update.
         */
        uhd::time_spec_t _last_update_time;
        /**
         * The error from the previous step of the PID.
         * Used to calculate the derivative.
         */
        double previous_error = 0;
        /**
         * Moving average filter.
         * A filter used to get the average of several measurements to check for convergence. It is NOT part of the PID controller itself
         */
        uhd::sma error_filter;
        /**
         * The difference between the host and device time.
         */
        // TODO replace a static difference updated periodically with mx + b where m is the ratio of the clock rate between the host and device (should be close to but not exactly 0), and b is the difference in clocks the one of them (TBD which) is 0
        double difference = 0;
        /**
         * True when the host and device clocks are synced
         */
        bool converged;
    };

    /**
     * A wrapper for the socket used to communicate over the SFP port.
     */
    uhd::transport::udp_simple::sptr _iface;
    /**
     * A mutex used to control access to iface.
     */
    std::shared_ptr<std::mutex> _iface_mutex;
    /**
     * The tick rate used by the device's time diff packets
     */
    double _tick_rate;
    /**
     * The thread where clock synchronization runs.
     */
    std::thread clock_sync_thread;

    /**
     * Flag used to tell clock_sync_loop to exit
     */
    std::atomic<bool> should_exit;

    /**
     * First of 2 PID controllers used to calculate the time.
     */
    pid pid_a;
    /**
     * Second of 2 PID controllers used to calculate the time.
     */
    pid pid_b;
    /**
     * Pointer to the PID controller to use when getting time. Multiple PIDs are used be able to avoid race conditions when updating them without blocking get_predicted_time requests
     */
    pid* active_pid;

    /**
     * Loop run inside clock_sync_thread.
     * Measures time difference between host and device and updates the pid
     * @param self The clock sync manager, it exists because member functions cannot be used to start threads
     */
    static void clock_sync_loop(clock_sync_manager* self);

    /**
     * Updates the pid.
     * Gets the current prediction and updates the specified PID with the new error, then changes the active PID to it
     * @param self The clock sync manager, it exists because member functions cannot be used to start threads
     * @param pid_to_update THe PID to update. Also sets active_pid in self to pid_to_update
     */
    static void update_pid(clock_sync_manager* self, pid* pid_to_update);

    /**
     * Sends and receives time diff requests
     * @param predicted_time A pointer to the current predicted arrival time
     * @param tdr A pointer to the reply, which contains the difference between the predicted time and actual time
     * @return Return true on success, false otherwise
     */
    bool time_diff_querry( const uhd::time_spec_t* predicted_time, time_diff_resp* tdr);

};

}}
