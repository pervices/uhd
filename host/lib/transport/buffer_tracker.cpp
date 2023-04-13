#include <uhd/transport/buffer_tracker.hpp>

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
        printf("T80\n");
        return (int64_t)(total_samples_sent + buffer_level_bias);
    } else {
        printf("T100\n");
        uhd::time_spec_t time_streaming = now - sob_time;
        uint64_t samples_consumed = (time_streaming.get_full_secs() * nominal_sample_rate) + (uint64_t) (time_streaming.get_frac_secs() * nominal_sample_rate);
        int64_t predicted_buffer_level = int64_t(total_samples_sent - samples_consumed);
        printf("time_streaming.get_frac_secs(): %lf\n", time_streaming.get_frac_secs());
        printf("nominal_sample_rate: %lf\n", nominal_sample_rate);
        printf("time_streaming.get_real_secs(): %lf\n", time_streaming.get_real_secs());
        printf("(time_streaming.get_frac_secs() * nominal_sample_rate): %lf\n", (time_streaming.get_frac_secs() * nominal_sample_rate));
        printf("samples_consumed: %lu\n", samples_consumed);
        printf("total_samples_sent: %lu\n", total_samples_sent);
        printf("predicted_buffer_level: %li\n", predicted_buffer_level);
        return predicted_buffer_level + buffer_level_bias;
    }
}

// Adjusts the bias in buffer level prediction
// level: measured buffer level
void buffer_tracker::update_buffer_level_bias( const uint64_t level ) {
    // Only update bias if most of the required samples to fill the buffer have been sent
    if(total_samples_sent > 0.75*nominal_buffer_level) {
        buffer_level_bias = (int64_t)((buffer_level_bias - level) * 0.06);
    }
}

void buffer_tracker::update( const uint64_t samples_sent ) {
    total_samples_sent += samples_sent;
}

buffer_tracker::buffer_tracker( const uint64_t targer_buffer_level, const double rate )
:
nominal_buffer_level( targer_buffer_level ),
nominal_sample_rate( rate )
{}

}}
