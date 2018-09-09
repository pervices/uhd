#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_NONLINEAR_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_NONLINEAR_HPP_

#include <mutex>

#include <boost/format.hpp>

#include <uhd/exception.hpp>
#include "flow_control.hpp"
#include "sma.hpp"

namespace uhd {

class flow_control_nonlinear: virtual uhd::flow_control {

public:

	static constexpr double upper_margin_pcnt = 0.95;
	static constexpr double lower_margin_pcnt = 0.10;

	const size_t buffer_size;
	const size_t nominal_buffer_level;
	double nominal_sample_rate;

	std::mutex lock;
	ssize_t buffer_level;
	uhd::time_spec_t buffer_level_set_time;
	uhd::time_spec_t sob_time;

	uhd::sma buffer_level_filter;

	static sptr make( const double nominal_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size ) {
		return sptr( (uhd::flow_control *) new flow_control_nonlinear( nominal_sample_rate, nominal_buffer_level_pcnt, buffer_size ) );
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

		std::lock_guard<std::mutex> _lock( lock );

		return unlocked_start_of_burst_pending( now );
	}
	void set_start_of_burst_time( const uhd::time_spec_t & sob ) {

		std::lock_guard<std::mutex> _lock( lock );

		sob_time = sob;
	}
	uhd::time_spec_t get_start_of_burst_time() {

		std::lock_guard<std::mutex> _lock( lock );

		return sob_time;
	}
	ssize_t get_buffer_level( const uhd::time_spec_t & now ) {

		ssize_t r;

		std::lock_guard<std::mutex> _lock( lock );

		r = unlocked_get_buffer_level( now );

		if ( r < 0 ) {
			r = 0;
		}
		if ( r > (ssize_t)buffer_size - 1 ) {
			r = buffer_size - 1;
		}

		return r;
	}
	void set_buffer_level( const size_t level, const uhd::time_spec_t & now ) {

		std::lock_guard<std::mutex> _lock( lock );

		unlocked_set_buffer_level( level );
		buffer_level_set_time = now;
	}
	void set_buffer_level_async( const size_t level ) {

		ssize_t _level = level;

		std::lock_guard<std::mutex> _lock( lock );

		_level = buffer_level + 0.06 * ( _level - buffer_level );

		unlocked_set_buffer_level( _level );
	}

	uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send, const uhd::time_spec_t &now ) {

		(void)nsamples_to_send;

		uhd::time_spec_t dt;
		double bl;

		std::lock_guard<std::mutex> _lock( lock );

		if ( BOOST_UNLIKELY( unlocked_start_of_burst_pending( now ) ) ) {

            bl = unlocked_get_buffer_level( now );

            if ( nominal_buffer_level > bl ) {
				dt = 0.0;
			} else {
				dt = sob_time - now;
			}
		} else {
			bl = unlocked_get_buffer_level( now );
			dt = ( bl - (double)nominal_buffer_level ) / nominal_sample_rate;
		}

		return dt;
	}

	void update( const size_t nsamples_sent, const uhd::time_spec_t & now ) {

		uhd::time_spec_t then;

		std::lock_guard<std::mutex> _lock( lock );

		buffer_level += nsamples_sent;
		buffer_level = unlocked_get_buffer_level( now );
		if ( BOOST_LIKELY( unlocked_start_of_burst_pending( now ) ) ) {
			buffer_level_set_time = sob_time;
		} else {
			buffer_level_set_time = now;
		}

#ifdef DEBUG_FLOW_CONTROL
		// underflow
		if ( BOOST_UNLIKELY( buffer_level < 0 ) ) {
			std::string msg =
				(
					boost::format( "Underflow occurred %u / %u" )
					% buffer_level
					% buffer_size
				).str();
			throw uhd::value_error( msg );
		}
		// overflow
		if ( BOOST_UNLIKELY( buffer_level > (ssize_t)buffer_size - 1 ) ) {
			std::string msg =
				(
					boost::format( "Overflow occurred %u / %u" )
					% buffer_level
					% buffer_size
				).str();
			throw uhd::value_error( msg );
		}
#endif
	}

protected:
	flow_control_nonlinear()
	:
		buffer_size( 0 ),
		nominal_buffer_level( 0 ),
		nominal_sample_rate( 0 ),
		buffer_level( 0 )
	{
	}

	flow_control_nonlinear( const double nominal_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size )
	:
		buffer_size( buffer_size ),
		nominal_buffer_level( nominal_buffer_level_pcnt * buffer_size ),
		nominal_sample_rate( nominal_sample_rate ),
		buffer_level( 0 ),
		buffer_level_filter( 1 )
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

	bool unlocked_start_of_burst_pending( const uhd::time_spec_t & now ) {
		return now < sob_time;
	}

	ssize_t unlocked_get_buffer_level( const uhd::time_spec_t & now ) {
		ssize_t r = buffer_level;

		// decrement the buffer level only when we are actively sending
		if ( BOOST_LIKELY( ! unlocked_start_of_burst_pending( now ) ) ) {
			uhd::time_spec_t a = buffer_level_set_time;
			uhd::time_spec_t b = now;
			size_t nsamples_consumed = interp( a, b, nominal_sample_rate );
			r -= nsamples_consumed;
		}

		return r;
	}

	void unlocked_set_buffer_level( const ssize_t level ) {

//		if ( BOOST_UNLIKELY( level >= buffer_size ) ) {
//			std::string msg =
//				(
//					boost::format( "Invalid buffer level %u / %u" )
//					% level
//					% buffer_size
//				).str();
//			throw uhd::value_error( msg );
//		}

		buffer_level = level;
	}
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_NONLINEAR_HPP_ */
