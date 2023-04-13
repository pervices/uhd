#pragma once

#include <cstdint>
#include <uhd/types/time_spec.hpp>

namespace uhd { namespace transport {

// Used to track the buffer level
class buffer_tracker {

public:

    // Target buffer level
	const uint64_t nominal_buffer_level;

	void set_sample_rate( const double rate );

	bool start_of_burst_pending( const uhd::time_spec_t & now );
	void set_start_of_burst_time( const uhd::time_spec_t & sob );

	int64_t get_buffer_level( const uhd::time_spec_t & now );

	void update_buffer_level_bias( const uint64_t level );

	void update( const uint64_t nsamples_sent );

	buffer_tracker( const uint64_t targer_buffer_level, const double rate );

private:
    double nominal_sample_rate = 0;

    // Total number of samples sent, will roll over
    uint64_t total_samples_sent = 0;
    // How much the target buffer level is being over/undershot by
    // This variable is accessed by multiple threads, but since it is only written by one thread, and the reads are not time sensative no locking system is required
    int64_t buffer_level_bias = 0;

    // Time when the unit begins transmitting (start of burst)
    uhd::time_spec_t sob_time;

};
}}
