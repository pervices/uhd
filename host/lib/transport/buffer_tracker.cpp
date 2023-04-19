#include <uhd/transport/buffer_tracker.hpp>
#include <iostream>

namespace uhd { namespace transport {

// Used to track the buffer level
// update_buffer_level_bias can be called from any thread, everything else should only be called from the same thread

void buffer_tracker::set_sample_rate( const double rate ) {
    nominal_sample_rate = rate;
}

// Returns true if waiting for start time
bool buffer_tracker::start_of_burst_pending( const uhd::time_spec_t & now ) {
    return now < sob_time;
}

// Sets the time when streaming begins
void buffer_tracker::set_start_of_burst_time( const uhd::time_spec_t & sob ) {
    sob_time = sob;
}

// Gets the predicted buffer level at the time requested
int64_t buffer_tracker::get_buffer_level( const uhd::time_spec_t & now ) {
    if(start_of_burst_pending(now)) {
        // TODO: make sure the proper behaviour is kept while overflowing int64_t
        return (int64_t)(total_samples_sent + buffer_level_bias);
    } else {
        uhd::time_spec_t time_streaming = now - sob_time;
        uint64_t samples_consumed =(uint64_t)(time_streaming.get_real_secs() * nominal_sample_rate);
        int64_t predicted_buffer_level = int64_t(total_samples_sent - samples_consumed);
        return predicted_buffer_level + buffer_level_bias;
    }
}

// Adjusts the bias in buffer level prediction
// level: measured buffer level
// Now: the time the buffer was read at
// Updating buffer level bias shouldn't be time sensitive so its fine to not implement thread synchronization mechanisms
void buffer_tracker::update_buffer_level_bias( const int64_t level, const uhd::time_spec_t & now ) {
    // Only update bias if most of the required samples to fill the buffer have been sent
    if(!start_of_burst_pending(now) && total_samples_sent > (uint64_t) nominal_buffer_level) {
        int64_t new_buffer_level_bias = buffer_level_bias + (int64_t)((level - nominal_buffer_level) * 0.01);

        // Limit the bias to 1% of the desired buffer level per 1s of streaming
        int64_t buffer_bias_limit = (int64_t)((now - sob_time).get_real_secs() * nominal_buffer_level * 0.01);
        if(buffer_level_bias > buffer_bias_limit) {
            buffer_level_bias = buffer_bias_limit;
        } else if(buffer_level_bias < -buffer_bias_limit) {
            buffer_level_bias = -buffer_bias_limit;
        } else {
            buffer_level_bias = new_buffer_level_bias;
        }
    }
}

void buffer_tracker::update( const uint64_t samples_sent ) {
    total_samples_sent += samples_sent;
}

buffer_tracker::buffer_tracker( const int64_t targer_buffer_level, const double rate )
:
nominal_buffer_level( targer_buffer_level ),
nominal_sample_rate( rate )
{
}

}}
