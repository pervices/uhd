#pragma once

#include <cstdint>
#include <uhd/types/time_spec.hpp>
#include <vector>

namespace uhd { namespace transport {

// Used to track the buffer level
class buffer_tracker {

public:

#ifdef DEBUG_PRIMING
    // Only print error message if there are two consecutive buffer levels of 0 when priming
    bool failure_to_prime_antirace = false;
    // Samples were in the buffer, therefore samples should be getting accepted
    bool buffer_samples_confirmed = false;
    bool priming_message_printed = false;
#endif

    // Target buffer level
	const int64_t nominal_buffer_level;

	void set_sample_rate( const double rate );

	bool start_of_burst_pending( const uhd::time_spec_t & now );
    void set_start_of_burst_time( const uhd::time_spec_t & sob );
    // Removes the last sob added to the list
    void pop_back_start_of_burst_time();
    void set_end_of_burst_time( const uhd::time_spec_t & sob );
    // Removes the last sob added to the list
    void pop_back_end_of_burst_time();

	int64_t get_buffer_level( const uhd::time_spec_t & now );

	void update_buffer_level_bias( const int64_t level, const uhd::time_spec_t & now );

	void update( const uint64_t nsamples_sent );

	buffer_tracker( const int64_t targer_buffer_level, const double rate );

//     buffer_tracker& operator=(const buffer_tracker&) {
//         return &buffer_tracker(nominal_buffer_level, nominal_sample_rate);
//     }

private:
    bool sob_reset = false;
    double nominal_sample_rate = 0;

    // Total number of samples sent, will roll over
    uint64_t total_samples_sent = 0;
    // How much the target buffer level is being over/undershot by
    // This variable is accessed by multiple threads, but since it is only written by one thread, and the reads are not time sensative no locking system is required
    int64_t buffer_level_bias = 0;

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
