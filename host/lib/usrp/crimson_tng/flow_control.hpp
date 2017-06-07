#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_

#include <iostream>
#include <cmath>
#include <mutex>

#include <boost/core/ignore_unused.hpp>
#include <boost/format.hpp>

#include <uhd/exception.hpp>

#include "../../../include/uhd/utils/pidc_tl.hpp"

#ifndef DEBUG_FLOW_CONTROL
#define DEBUG_FLOW_CONTROL 1
#ifndef DEBUG_FLOW_CONTROL_EXCEPTIONS
#define DEBUG_FLOW_CONTROL_EXCEPTIONS 1
#endif
#endif

namespace uhd {

/**
 * This is a slightly odd wrapper around a PID controller.
 *
 * What makes it odd, is that the software-based PID controller cannot sample
 * the Process Variable nearly fast enough due to the fact that it is
 * I/O bound.
 *
 * However, we can infer the Process Variable by simply counting samples that
 * we have sent. It's an approximation, albeit a fairly accurate one. The
 * Process Variable is considered as rejected disturbance. The flow controller
 * allows for PV updates synchronously or asynchronously via monitor methods.
 * That is to say, a separate thread can sample the PV at a particular interval
 * and correct our approximation.
 *
 * Our Plant is responsible for translating the Process Variable and Control
 * Variable to a sample rate which the system uses to determine what delay to
 * use until the next burst of data is sent, thus rate-limiting transmission.
 */
class flow_control {

public:

	static constexpr double upper_margin_pcnt = 0.95;
	static constexpr double lower_margin_pcnt = 0.10;

	const size_t buffer_size;
	const double nominal_buffer_level;
	const double nominal_sample_rate;
	const double pid_sample_rate;

	std::mutex lock;
	ssize_t buffer_level;
	uhd::time_spec_t buffer_level_set_time;

	double sample_rate;
	pidc_tl pidc;

	uhd::time_spec_t sob_time;

#ifdef DEBUG_FLOW_CONTROL
	uhd::time_spec_t msg_time;
#endif

	/**
	 * flow_control ctor
	 *
	 * @param nominal_sample_rate        the operating tx rate [samples / s]
	 * @param pid_sample_rate            the rate at which the process variable is to be sampled [Hz]
	 * @param nominal_buffer_level_pcnt  the desired buffer level [%]
	 * @param buffer_size                the buffer size [samples]
	 */
	flow_control( const double nominal_sample_rate, const double pid_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size )
	:
		buffer_size( buffer_size ),
		nominal_buffer_level( nominal_buffer_level_pcnt * buffer_size ),
		nominal_sample_rate( nominal_sample_rate ),
		pid_sample_rate( pid_sample_rate ),
		buffer_level( 0 ),
		sample_rate( nominal_sample_rate ),
		pidc(
			nominal_buffer_level_pcnt * buffer_size,
			std::min( nominal_buffer_level_pcnt, 1 - nominal_buffer_level_pcnt ),
			2 / pid_sample_rate
		)
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
		if ( nominal_sample_rate <= 0 || nominal_sample_rate < 10 * pid_sample_rate ) {
			throw uhd::value_error(
				(
					boost::format( "Invalid sample_rate %f" )
					% nominal_sample_rate
				).str()
			);
		}

#ifdef DEBUG_FLOW_CONTROL
		msg_time = uhd::time_spec_t::get_system_time();
#endif
	}

	flow_control( const uhd::flow_control &other )
	:
		buffer_size( other.buffer_size ),
		nominal_buffer_level( other.nominal_buffer_level ),
		nominal_sample_rate( other.nominal_sample_rate ),
		pid_sample_rate( other.pid_sample_rate )
	{
		buffer_level = other.buffer_level;
		sample_rate  = other.sample_rate;
		pidc         = other.pidc;

#ifdef DEBUG_FLOW_CONTROL
		msg_time     = other.msg_time;
#endif
	}

	virtual ~flow_control()
	{
	}

	/**
	 * Report whether the specified time is less than the start of burst time.
	 *
	 * @param now  the time to compare with the start of burst time
	 * @return true if the specified time is less than the start of burst time, otherwise false
	 */
	bool start_of_burst_pending( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {

		std::lock_guard<std::mutex> _lock( lock );

		return unlocked_start_of_burst_pending( now );
	}
	/**
	 * Set the time for a start of burst
	 *
	 * @param now  the start of burst time
	 */
	void set_start_of_burst_time( const uhd::time_spec_t & sob ) {

		std::lock_guard<std::mutex> _lock( lock );

		sob_time = sob;
	}
	/**
	 * Get the time for a start of burst
	 *
	 * @param now  the start of burst time [default is 0, when no start of burst is set]
	 */
	uhd::time_spec_t get_start_of_burst_time() {

		std::lock_guard<std::mutex> _lock( lock );

		return sob_time;
	}

	/**
	 * interp()
	 *
	 * Using linear interpolation, compute the number of samples transmitted
	 * from time a to time b at a given sample rate.
	 *
	 * @param a            start time
	 * @param b            stop time
	 * @param sample_rate  the rate at which samples are sent [ samples / s ]
	 *
	 * @return the number of samples sent in b-a seconds
	 */
	static size_t interp( const uhd::time_spec_t & a, const uhd::time_spec_t & b, const double sample_rate ) {

		if ( BOOST_UNLIKELY( b < a ) ) {
			throw value_error(
				(
					boost::format( "time b (%f) < time a (%f)" )
					% b.get_real_secs()
					% a.get_real_secs()
				).str()
			);
		}

		return floor( ( b - a ).get_real_secs() / sample_rate );
	}

	/**
	 * get_buffer_level
	 *
	 * Get the (approximate) level of the buffer. Under normal operating
	 * conditions, this reflects the level of the actual tx buffer. Periodic
	 * updates of actual tx buffer levels are treated as rejected disturbances.
	 *
	 * Inter-sample buffer levels use linear approximation based on the
	 * current sample rate.
	 *
	 * @return the buffer level [samples]
	 */
	size_t get_buffer_level( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {

		std::lock_guard<std::mutex> _lock( lock );

		return unlocked_get_buffer_level( now );
	}
	/**
	 * set_buffer_level
	 *
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param level   the actual buffer level [samples]
	 */
	void set_buffer_level( const size_t level, const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {

		std::lock_guard<std::mutex> _lock( lock );

		if ( BOOST_UNLIKELY( level > buffer_size ) ) {
			std::string msg =
				(
					boost::format( "Invalid buffer level %u / %u" )
					% level
					% buffer_size
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return;
#endif
		}

		buffer_level = level;
		buffer_level_set_time = now;
	}

	/**
	 * get_buffer_level_pcnt
	 *
	 * Get the (approximate) level of the buffer. Under normal operating
	 * conditions, this reflects the level of the actual tx buffer. Periodic
	 * updates of actual tx buffer levels are treated as rejected disturbances.
	 *
	 * @return the buffer level [%]
	 */
	double get_buffer_level_pcnt( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		return get_buffer_level( now ) / (double) buffer_size;
	}
	/**
	 * set_buffer_level
	 *
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param pcnt   the actual buffer level [%]
	 */
	void set_buffer_level_pcnt( const double pcnt, const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		set_buffer_level( pcnt * buffer_size, now );
	}

	/**
	 * get_sample_rate
	 *
	 * If a start of burst is pending, return the nominal sample rate.
	 *
	 * Otherwise, return the sample rate with flow control compensation.
	 *
	 * The compensated sample rate is constant between calls to update.
	 *
	 * The parameter 'now' is only used to compare with start-of-burst time.
	 *
	 * @param now the time to query the sample rate
	 *
	 * @return the sample rate [sample / s]
	 */
	double get_sample_rate( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {

		std::lock_guard<std::mutex> _lock( lock );

		if ( BOOST_UNLIKELY( unlocked_start_of_burst_pending( now ) ) ) {
			return nominal_sample_rate;
		} else {
			return sample_rate;
		}
	}

	static double update_sample_rate( const ssize_t current_buffer_level, const ssize_t target_buffer_level, const ssize_t nominal_buffer_level, const ssize_t buffer_size, const double current_sample_rate, const double nominal_sample_rate, const uhd::time_spec_t & now, const uhd::time_spec_t & deadline ) {

		double sample_rate;

		uhd::time_spec_t dt;

#ifdef DEBUG_FLOW_CONTROL
		if ( BOOST_UNLIKELY( current_buffer_level >= buffer_size ) ) {
			std::string msg =
				(
					boost::format( "current buffer level (%u) >= buffer_size (%u)" )
					% current_buffer_level
					% buffer_size
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return 0.0;
#endif
		}

		if ( BOOST_UNLIKELY( target_buffer_level >= buffer_size ) ) {
			std::string msg =
				(
					boost::format( "target buffer level (%u) >= buffer_size (%u)" )
					% target_buffer_level
					% buffer_size
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return 0.0;
#endif
		}

		if ( BOOST_UNLIKELY( nominal_buffer_level >= buffer_size ) ) {
			std::string msg =
				(
					boost::format( "nominal buffer level (%u) >= buffer_size (%u)" )
					% nominal_buffer_level
					% buffer_size
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return 0.0;
#endif
		}

		if ( BOOST_UNLIKELY( deadline < now ) ) {
			std::string msg =
				(
					boost::format( "deadline (%f) < now (%f)" )
					% deadline.get_real_secs()
					% now.get_real_secs()
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return current_sample_rate;
#endif
		}
#endif

		//
		// Sample Rate Adjustment Algorithm
		// --------------------------------
		//
		dt = deadline - now;
		if ( dt < 1e-6 ) {
			return current_sample_rate;
		}

		double foo = ( target_buffer_level - current_buffer_level ) / dt.get_real_secs();
		sample_rate = nominal_sample_rate * ( 1 + (foo/nominal_sample_rate ) );

#ifdef DEBUG_FLOW_CONTROL
//		if ( std::abs( sample_rate ) >= 200e6 ) {
//			std::string msg =
//				(
//					boost::format( "abs(sample rate) too high: target_buffer_level: %u, current_buffer_level: %u, dt: %f" )
//					% target_buffer_level
//					% current_buffer_level
//					% dt.get_real_secs()
//				).str();
//#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
//			throw uhd::value_error( msg );
//#else
//			std::cerr << msg << std::endl;
//			return current_sample_rate;
//#endif
//
//		}
#endif

#ifdef DEBUG_FLOW_CONTROL
		if ( sample_rate < 0 ) {
			std::string msg =
				(
					boost::format( "sample rate negative: target_buffer_level: %u, current_buffer_level: %u, dt: %f" )
					% target_buffer_level
					% current_buffer_level
					% dt.get_real_secs()
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return current_sample_rate;
#endif
		}
#endif

		//
		// Clipping
		// --------
		//

		if (
			BOOST_UNLIKELY(
				true
				&& current_buffer_level / (double) buffer_size >= upper_margin_pcnt
				&& sample_rate >= nominal_sample_rate
			)
		) {
			// This is a last-ditch effort to avoid buffer overflow.

			// Underflow is unavoidable if the user stops sending samples, but overflow
			// is a clear indication that the sample rate adjustment algorithm is
			// flawed.

#ifdef DEBUG_FLOW_CONTROL
			std::string msg =
				(
					boost::format( "bad attempt to set sample rate higher at margin: (buffer_level: %u, sample_rate: %f )" )
					% current_buffer_level
					% sample_rate
				).str();
#ifdef DEBUG_FLOW_CONTROL_EXCEPTIONS
			throw uhd::value_error( msg );
#else
			std::cerr << msg << std::endl;
			return current_sample_rate;
#endif
			sample_rate = nominal_sample_rate;
		}
#endif

		if (
			BOOST_UNLIKELY(
				true
				&& current_buffer_level / (double) buffer_size <= lower_margin_pcnt
				&& sample_rate <= nominal_sample_rate
			)
		) {
			sample_rate = nominal_sample_rate;
		}

		return sample_rate;
	}

	/**
	 * update
	 *
	 * Report to the flow controller, that the caller has sent additional
	 * samples and that the flow controller should adjust its buffer levels,
	 * internal state, and sample rate, appropriately.
	 *
	 * @param nsamples_sent   The number of samples sent
	 * @param now             The time at which the samples were sent [default: current system time]
	 */
	void update( const size_t nsamples_sent, const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {

		uhd::time_spec_t then;
		double sp;
		double pv;
		double cv;

		std::lock_guard<std::mutex> _lock( lock );

		//
		// Update Buffer Level
		//

		buffer_level = unlocked_get_buffer_level( now );
		buffer_level += nsamples_sent;

		buffer_level = buffer_level >= (ssize_t)buffer_size ? buffer_size - 1 : buffer_level;
		buffer_level = buffer_level <= 0 ? 0 : buffer_level;

		buffer_level_set_time = now;

		//
		// Update PID Controller
		//

		if ( 0.0 == pidc.get_last_time() ) {
			then = ( now - 1 / pid_sample_rate );
			pidc.set_last_time( then.get_real_secs() );
			//then -= ( now - then ).get_real_secs() * 0.10;
		} else {
			then = uhd::time_spec_t( pidc.get_last_time() );
		}

		sp = nominal_buffer_level;
		pv = buffer_level;

		if ( now >= then ) {

			cv = pidc.update_control_variable( sp, pv, now.get_real_secs() );

			then = now + 1 / pid_sample_rate;

		} else {
			cv = pidc.get_control_variable();
		}

		//
		// Update Sample Rate
		//

		// Plant: Must convert desired buffer level to sample rate
		// =======================================================
		sample_rate = update_sample_rate( buffer_level, cv, nominal_buffer_level, buffer_size, sample_rate, nominal_sample_rate, now, then );

#ifdef DEBUG_FLOW_CONTROL
//		if ( uhd::time_spec_t::get_system_time() > msg_time + 1.0 ) {
//			msg_time = uhd::time_spec_t::get_system_time();
//
			std::cerr <<
				"current_buffer_level: " << buffer_level << ", "
				"target_buffer_level: " << cv << ", "
				"sample_rate: " << sample_rate << ", "
				"dt: " << ( then - now ).get_real_secs() <<
				std::endl;
//
//		}
#endif

	}

	/**
	 * get_time_until_next_send
	 *
	 * The primary purpose of a flow controller is to act as a rate-limiter.
	 * If samples are sent too quickly, input buffers can overflow and
	 * data is corrupted. If samples are sent too slowly, input buffers can
	 * underflow resulting in signal corruption (typically zeros are inserted).
	 *
	 * This functions determines the amount of time to wait until it is
	 * necessary to send data in order to maintain the nominal sample rate.
	 *
	 * @param nsamples_to_send   The number of samples the caller would like to send
	 * @param now                The time to wait from
	 * @return                   The amount of time to wait from 'now'
	 */
	uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send, const uhd::time_spec_t &now = uhd::time_spec_t::get_system_time() ) {

		// TODO: potentially add extra delay if nsamples_to_send is 'large' so that we do not overflow
		boost::ignore_unused( nsamples_to_send );

		uhd::time_spec_t dt;
		double cv;
		double pv;

		std::lock_guard<std::mutex> _lock( lock );

		if ( BOOST_UNLIKELY( unlocked_start_of_burst_pending( now ) ) ) {

			dt = sob_time - now;
			dt -= nominal_buffer_level / nominal_sample_rate;

		} else {

			cv = pidc.get_control_variable();
			pv = unlocked_get_buffer_level( now );

			dt = ( cv - pv ) / sample_rate;

			dt = dt < 0.0 ? 0.0 : dt;
		}

		return dt;
	}

private:
	bool unlocked_start_of_burst_pending( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		return now < sob_time;
	}

	size_t unlocked_get_buffer_level( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		ssize_t r = buffer_level;

		// decrement the buffer level only when we are actively sending
		if ( BOOST_LIKELY( ! unlocked_start_of_burst_pending( now ) ) ) {
			r -= interp( now < buffer_level_set_time ? now : buffer_level_set_time, now, sample_rate );
		}

		r = r < 0 ? 0 : r;

		return r;
	}
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_ */
