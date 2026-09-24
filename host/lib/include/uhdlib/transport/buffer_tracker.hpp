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

    buffer_tracker( const double rate );

private:
    double nominal_sample_rate = 0;

    // Total number of samples sent, will roll over
    uint64_t total_samples_sent = 0;
    // How much the target buffer level is being over/undershot by

    // Time when the unit begins transmitting (start of burst)
    // Used as a reference point for basing other calculations off of
    bool first_sob_set = false;
    uhd::time_spec_t first_sob_time;

    // Stores times start and end times of periods where no samples are sent
    std::vector<uhd::time_spec_t> blank_period_start = {uhd::time_spec_t(0.0)};
    std::vector<uhd::time_spec_t> blank_period_stop;
    // Time skipped by past blank periods
    // When a blank period is in the past, removed it from the list of blank periods and add the samples skipped to here
    uhd::time_spec_t blanked_time = uhd::time_spec_t(0.0);

};
}}
