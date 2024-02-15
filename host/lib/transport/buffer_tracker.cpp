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
    return now < first_sob_time;
}

// Sets the time when this burst ends
void buffer_tracker::set_start_of_burst_time( const uhd::time_spec_t & sob ) {
    blank_period_stop.push_back(sob);
    if(first_sob_set) {
    } else {
        first_sob_time = sob;
        first_sob_set = true;
    }
}

// Sets the time when this burst ends
void buffer_tracker::pop_back_start_of_burst_time() {
    blank_period_stop.pop_back();
}

// Sets the time when streaming begins
void buffer_tracker::set_end_of_burst_time( const uhd::time_spec_t & eob ) {
    // Skip setting eob time if no sob has been issued
    if(!first_sob_set) {
        return;
    }
    // Record the start of a new blanking period
    blank_period_start.push_back(eob);
}
// Sets the time when this burst ends
void buffer_tracker::pop_back_end_of_burst_time() {
    return blank_period_start.pop_back();
}

// Gets the predicted buffer level at the time requested
int64_t buffer_tracker::get_buffer_level( const uhd::time_spec_t & now ) {

    // For debugging
    // if(blank_period_start.size() > blank_period_stop.size() && !buffer_mistmatch_printed) {
    //     printf("more blank period stops than starts\n");
    //     buffer_mistmatch_printed = true;
    // }

    // Finds blank periods in the past
    int64_t blank_periods_to_remove = 0;
    for(uint64_t n = 0; (n < blank_period_stop.size()) && (blank_period_stop[n] < now); n++) {
        // Add past blank periods to the blanked time total
        blanked_time+= (blank_period_stop[n] - blank_period_start[n]);
        blank_periods_to_remove++;
    }
    //Removed past blank periods from the list of future/current blank periods
    if(blank_periods_to_remove > 0) {
        blank_period_start.erase(blank_period_start.begin(), blank_period_start.begin() + blank_periods_to_remove);
        blank_period_stop.erase(blank_period_stop.begin(), blank_period_stop.begin() + blank_periods_to_remove);
    }

    // How much time has been spent in the current blank period
    uhd::time_spec_t partial_blank_period;
    if(blank_period_start.size() > 0 && now > blank_period_start[0]) {
        partial_blank_period = now - blank_period_start[0];
    } else {
        // There are no blank periods waiting
        partial_blank_period = uhd::time_spec_t(0.0);
    }


    uhd::time_spec_t time_streaming = now - blanked_time - partial_blank_period;
    uint64_t samples_consumed = (uint64_t)(time_streaming.get_full_secs() * nominal_sample_rate) + (uint64_t)(time_streaming.get_frac_secs() * nominal_sample_rate);
    if(samples_consumed > total_samples_sent) {
        return 0;
    } else {
        int64_t predicted_buffer_level = int64_t(total_samples_sent - samples_consumed);
        return std::max(predicted_buffer_level + buffer_level_bias, (int64_t) 0);
    }
}

// Adjusts the bias in buffer level prediction
// level: measured buffer level
// Now: the time the buffer was read at
// Updating buffer level bias shouldn't be time sensitive so its fine to not implement thread synchronization mechanisms
void buffer_tracker::update_buffer_level_bias( const int64_t level, const uhd::time_spec_t & now ) {
    // If the buffer level is 0 then the predicted level is definely wrong (since it can be negative)
    // Therefore skip updating bias
    if(level == 0) {
        return;
    }

#ifdef DEBUG_PRIMING
    else {
        if(level != 0 || buffer_samples_confirmed) {
            buffer_samples_confirmed = true;
            return;
        }
        else if(total_samples_sent > 0) {
            if(failure_to_prime_antirace) {
                priming_message_printed = true;
                std::cerr << "Buffer failed to prime" << std::endl;
            } else {
                failure_to_prime_antirace = true;
            }
        }
    }
#endif

    int64_t predicted_buffer_level = get_buffer_level(now);
    // Only update bias if most of the required samples to fill the buffer have been sent
    if(!start_of_burst_pending(now)) {
        // If buffer level is above the target, adjust the bias upwards
        if(level > nominal_buffer_level) {
            buffer_level_bias += (int64_t)((level - nominal_buffer_level) * 0.01);
            return;
        // If the buffer level is below expected adjust bias downwards
        } else if(level < predicted_buffer_level) {
            buffer_level_bias += (int64_t)((level - predicted_buffer_level) * 0.01);
            return;
        // Otherwise do nothing
        } else {
            return;
        }
    }
}

void buffer_tracker::update( const uint64_t samples_sent ) {
    total_samples_sent += samples_sent;
}

buffer_tracker::buffer_tracker( const int64_t targer_buffer_level, const double rate )
:
nominal_buffer_level( targer_buffer_level ),
nominal_sample_rate( rate ),
blanked_time(0.0)
{
}

}}
