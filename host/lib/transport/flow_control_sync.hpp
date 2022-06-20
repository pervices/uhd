#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_SYNC_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_SYNC_HPP_

#include <mutex>

#include <boost/format.hpp>

#include <uhd/exception.hpp>
#include "flow_control.hpp"
#include "uhd/utils/sma.hpp"

#if 0
#define DEBUG_FLOW_CONTROL
#endif


namespace uhd {

// Class for keeping track of the buffer level synchronously, not thread safe
class flow_control_sync: virtual uhd::flow_control {

public:
	typedef std::shared_ptr<uhd::flow_control_sync> sptr;

	const size_t buffer_size;
    // target buffer level
    const size_t nominal_buffer_level;
    // intended sample rate
	double nominal_sample_rate = 0;
    
    // number of samples sent since the last get level request was issued (not nsamps sent since level was recevied)
    size_t samples_sent_since_last_request = 0;
    
    // buffer level at the last nsamps request + number of samples sent between the the buffer level request was issued and the reply was received
    size_t buffer_level_at_last_request = 0;
    
    // buffer level during the last
    uhd::time_spec_t buffer_level_request_time = 0.0;
    
	uhd::time_spec_t sob_time = 0.0;

	static sptr make( const double nominal_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size ) {
		return sptr( (uhd::flow_control_sync *) new flow_control_sync( nominal_sample_rate, nominal_buffer_level_pcnt, buffer_size ) );
	}

	size_t get_buffer_size() {
		return buffer_size;
	}
	
	size_t get_nominal_buffer_level() {
		return nominal_buffer_level;
	}
	double get_nominal_sample_rate() {
		return nominal_sample_rate;
	}
	
	void set_sample_rate( const uhd::time_spec_t & now, const double & rate ) {
		boost::ignore_unused( now );
		nominal_sample_rate = rate;
	}

	bool start_of_burst_pending( const uhd::time_spec_t & now ) {
		return now < sob_time;
	}
	
	void set_start_of_burst_time( const uhd::time_spec_t & sob ) {
		sob_time = sob;
	}

	uhd::time_spec_t get_start_of_burst_time() {
		return sob_time;
	}
	
	ssize_t get_buffer_level( const uhd::time_spec_t & now ) {
        
        if( BOOST_UNLIKELY( start_of_burst_pending( now ) ) ) {
            return buffer_level_at_last_request + samples_sent_since_last_request;
        } else {
            size_t nsamples_consumed = interp( buffer_level_request_time, now, nominal_sample_rate );
            ssize_t buffer_level = buffer_level_at_last_request + samples_sent_since_last_request - nsamples_consumed;
#ifdef DEBUG_FLOW_CONTROL
			if(buffer_level < 0) {
				std::cout << "Underflow occuring" << std::endl;
			}
#endif
			return buffer_level;

        }
	}
	
	void set_buffer_level( const size_t level, const uhd::time_spec_t & last_known ) {
        if( BOOST_UNLIKELY( start_of_burst_pending( last_known ) ) ) {
            buffer_level_request_time = sob_time;
        } else {
            buffer_level_request_time = last_known;
        }
        buffer_level_at_last_request = level + samples_sent_since_last_request;
	}

	// This flow control tracks the number samples since the the last time the buffer was updated
	void reset_samples_sent_since_last_request() {
		samples_sent_since_last_request = 0;
	}

	uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send, const uhd::time_spec_t &now ) {

		(void)nsamples_to_send;

		uhd::time_spec_t time_until_next_send;
		double current_buffer_level;

        if ( BOOST_UNLIKELY( start_of_burst_pending( now ) ) ) {
            current_buffer_level = get_buffer_level( now );

            if ( nominal_buffer_level > current_buffer_level ) {
                time_until_next_send = 0.0;
            } else {
                time_until_next_send = sob_time - now;
            }
            
        } else {
            current_buffer_level = get_buffer_level( now );
            time_until_next_send = ( current_buffer_level - (double)nominal_buffer_level ) / nominal_sample_rate;
        }

		return time_until_next_send;
	}

	void update( const size_t nsamples_sent, const uhd::time_spec_t & now ) {

        (void)now;        
        samples_sent_since_last_request+= nsamples_sent;

	}

	flow_control_sync()
	:
		buffer_size( 0 ),
		nominal_buffer_level( 0 ),
		nominal_sample_rate( 0 ),
		samples_sent_since_last_request( 0 ),
        buffer_level_at_last_request( 0 )
	{
	}

	flow_control_sync( const double nominal_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size )
	:
		buffer_size( buffer_size ),
		nominal_buffer_level( nominal_buffer_level_pcnt * buffer_size ),
		nominal_sample_rate( nominal_sample_rate ),
		samples_sent_since_last_request( 0 ),
        buffer_level_at_last_request( 0 )
	{
		if (
			false
			|| nominal_buffer_level_pcnt < 0
			|| nominal_buffer_level_pcnt > 1
		) {
			throw uhd::value_error(
				(
					boost::format( "Invalid buffer level %f" )
					% nominal_buffer_level_pcnt
				).str()
			);
		}
		if ( nominal_sample_rate <= 0 ) {
			throw uhd::value_error(
				(
					boost::format( "Invalid sample_rate %f" )
					% nominal_sample_rate
				).str()
			);
		}
	}

};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_SYNC_HPP_ */
