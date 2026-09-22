// Copyright 2023-2024 Per Vices Corporation
#pragma once

#include <cstdint>
#include <uhd/types/time_spec.hpp>
#include <vector>

namespace uhd { namespace transport {

// Used to track the buffer level
class buffer_tracker {

public:

    void set_sample_rate( const double rate );

    void set_start_of_burst_time( const uhd::time_spec_t & sob );
    
    // Removes the last sob added to the list
    void pop_back_start_of_burst_time();
    void set_end_of_burst_time( const uhd::time_spec_t & sob );
    // Removes the last sob added to the list
    void pop_back_end_of_burst_time();

    int64_t get_buffer_level( const uhd::time_spec_t & now );

    void update( const uint64_t nsamples_sent );

    /**
     * Prepare to recover from an underflow.
     *
     * Clear the current tracking and start a new pseudo start of burst at next_packet.
     * This is only to be used when the user does not provide a time stamp
     *
     * @param next_time The time to send the next packet
     */
    void recovery_prep( const uhd::time_spec_t & next_packet );

    /**
     * Get the time of the last start of burst received
     *
     * @return The time of the last start of burst received
     */
    uhd::time_spec_t peek_last_sob();

    buffer_tracker( const double rate );

private:
    double nominal_sample_rate = 0;

    // Total number of samples sent, will roll over
    uint64_t total_samples_sent = 0;
    // How much the target buffer level is being over/undershot by

    // At least one start of burst has been recorded
    bool first_sob_set = false;

    // Stores times start and end times of periods where no samples are sent
    std::vector<uhd::time_spec_t> blank_period_start = {uhd::time_spec_t(0.0)};
    std::vector<uhd::time_spec_t> blank_period_stop;
    // Time skipped by past blank periods
    // When a blank period is in the past, removed it from the list of blank periods and add the samples skipped to here
    uhd::time_spec_t blanked_time = uhd::time_spec_t(0.0);

};
}}
