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

	const size_t buffer_size;
	const double nominal_buffer_level;
	const double nominal_sample_rate;
	const double pid_sample_rate;

	std::recursive_mutex lock;
	bool first_update;
	ssize_t buffer_level;
	double sample_rate;
	pidc_tl pidc;

	uhd::time_spec_t msg_time;

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
		first_update( true ),
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
		first_update = other.first_update;
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
	 * get_buffer_level
	 *
	 * Get the (approximate) level of the buffer. Under normal operating
	 * conditions, this reflects the level of the actual tx buffer. Periodic
	 * updates of actual tx buffer levels are treated as rejected disturbances.
	 *
	 * @return the buffer level [samples]
	 */
	size_t get_buffer_level() {
		size_t r;
		lock.lock();
		r = buffer_level;
		lock.unlock();
		return r;
	}
	/**
	 * set_buffer_level
	 *
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param level   the actual buffer level [samples]
	 */
	void set_buffer_level( const size_t level ) {
		lock.lock();
		if ( level > buffer_size ) {
			throw uhd::value_error(
				(
					boost::format( "Invalid buffer level %u / %u" )
					% level
					% buffer_size
				).str()
			);
		}
		buffer_level = level;
		lock.unlock();
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
	double get_buffer_level_pcnt() {
		return get_buffer_level() / (double) buffer_size;
	}
	/**
	 * set_buffer_level
	 *
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param pcnt   the actual buffer level [%]
	 */
	void set_buffer_level_pcnt( const double pcnt ) {
		set_buffer_level( pcnt * buffer_size );
	}

	/**
	 * get_sample_rate
	 *
	 * Get the current, compensating, sample rate. The sample rate
	 * is the output of the flow controller. The flow controller
	 * translates from desired buffer level, in samples, to sample rate.
	 *
	 * @return the current, compensating, sample rate
	 */
	double get_sample_rate() {
		double r;
		lock.lock();
		r = sample_rate;
		lock.unlock();
		return r;
	}

	/**
	 * update
	 *
	 * Report to the flow controller, that the caller has sent additional
	 * samples and that the flow controller should adjust its buffer levels,
	 * internal state, and sample rate, appropriately.
	 *
	 * @param nsamples_sent   The number of samples sent
	 * @param now             The time at which the samples were sent
	 */
	void update( const size_t nsamples_sent, const uhd::time_spec_t & now ) {

		uhd::time_spec_t then;
		uhd::time_spec_t dt;
		size_t nsamples_consumed;
		double sp;
		double pv;
		double cv;

		lock.lock();

		if ( BOOST_UNLIKELY( first_update ) ) {

			then = now;
			then -= nominal_buffer_level / nominal_sample_rate;

			pidc.set_last_time( then.get_real_secs() );

			buffer_level = nominal_buffer_level;
		}

		then = uhd::time_spec_t( pidc.get_last_time() );
		dt = now - then;

		if ( dt > 0.0 ) {
			nsamples_consumed = floor( dt.get_real_secs() * nominal_sample_rate );

			buffer_level -= nsamples_consumed;
			buffer_level += nsamples_sent;
		} else {
			std::cout << "";
		}

#if defined( DEBUG_FLOW_CONTROL )
		if ( BOOST_UNLIKELY( dt < 0.0 ) ) {
			std::string msg = (
					boost::format( "time-difference is negative! %f" )
					% dt.get_real_secs()
				).str();
			std::cerr << msg << std::endl;
//			throw runtime_error( msg );
		}

		if ( BOOST_UNLIKELY( buffer_level < 0 ) ) {
			std::string msg = (
					boost::format( "buffer level has fallen below 0 and is at %u" )
					% buffer_level
				).str();
			std::cerr << msg << std::endl;
//			throw runtime_error( msg );
		}

		if ( BOOST_UNLIKELY( buffer_level > (ssize_t)buffer_size ) ) {
			std::string msg = (
					boost::format( "buffer level has risen above %u and is at %u" )
					% buffer_size
					% buffer_level
				).str();
			std::cerr << msg << std::endl;
//			throw runtime_error( msg );
		}
#endif

		if ( 0 != nsamples_sent ) {
			sp = nominal_buffer_level;
			pv = buffer_level;
			//if ( now >= pidc.get_last_time() + 1 / pid_sample_rate ) {
				// XXX: do *not* update the PID too frequently!! It is designed to be
				// updated at pid_sample_rate only. As dt -> 0, high frequency updates
				// introduce numerical instability in the differentiator.
				cv = pidc.update_control_variable( sp, pv, now.get_real_secs() );
//			} else {
//				cv = pidc.get_control_variable();
//			}

			// Plant: 1st Order Conversion: CV -> Sample Rate
			// ==============================================
			// CV is the desired buffer level and PV is the actual buffer level
			// dt is based on the tx time of the previous nsamples_sent
			// If sample_rate is extremely high, then that simply forces the
			// get_time_until_next_send() to 0, and then sample_rate is updated
			// again.
			sample_rate = ( cv - pv ) / dt.get_real_secs();
		} else {
			sample_rate = nominal_sample_rate;
		}


#if defined( DEBUG_FLOW_CONTROL )
		if ( BOOST_UNLIKELY( sample_rate < 0 ) ) {
			std::string msg = (
					boost::format( "sample rate has fallen below 0 and is at %f" )
					% sample_rate
				).str();
			std::cerr << msg << std::endl;
//			throw runtime_error( msg );
		}
#endif

		first_update = false;

		lock.unlock();

#ifdef DEBUG_FLOW_CONTROL
		if ( uhd::time_spec_t::get_system_time() > msg_time + 1.0 ) {
			msg_time = uhd::time_spec_t::get_system_time();

			std::cout <<
				"buffer_level: " << buffer_level << ", "
				"sample_rate: " << sample_rate <<
				std::endl;

		}
#endif
	}
	/**
	 * update
	 *
	 * Report to the flow controller, that the caller has sent additional
	 * samples and that the flow controller should adjust its buffer levels,
	 * internal state, and sample rate, appropriately.
	 *
	 * This variant of update uses the current system time.
	 *
	 * @param nsamples_sent
	 */
	void update( const size_t nsamples_sent ) {
		update( nsamples_sent, uhd::time_spec_t::get_system_time() );
	}

	/**
	 * get_time_until_next_send
	 *
	 * The primary purpose of a flow controller is to act as a rate-limiter.
	 * If samples are sent too quickly, input buffers can overflow and
	 * data is corrupted. If samples are sent too slowly, input buffers can
	 * underflow resulting in signal corruption (typically zeros are inserted).
	 *
	 * This functions determines the amount of time to wait between
	 * transmitting bursts of data.
	 *
	 * @param nsamples_to_send   The number of samples the caller would like to send
	 * @param now                The time to wait from
	 * @return                   The amount of time to wait from 'now'
	 */
	uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send, const uhd::time_spec_t &now ) {

		// TODO: potentially add extra delay if nsamples_to_send is 'large' so that we do not overflow
		boost::ignore_unused( nsamples_to_send );

		uhd::time_spec_t dt;
		uhd::time_spec_t last;

		lock.lock();

		if ( first_update ) {

			dt = uhd::time_spec_t( 0, 0 );

		} else {

			last = pidc.get_last_time();

			if ( last > now ) {
				// delayed send
				dt = last - uhd::time_spec_t( nominal_buffer_level / nominal_sample_rate ) - now;
			} else {
				dt = uhd::time_spec_t( buffer_level / sample_rate );
			}
		}

		lock.unlock();

		return dt;
	}
	/**
	 * get_time_until_next_send
	 *
	 * This functions determines the amount of time to wait between
	 * transmitting bursts of data.
	 *
	 * This method variant uses the current system time.
	 *
	 * @param nsamples_to_send   The number of samples the caller would like to send
	 * @return                   The amount of time to wait from 'now'
	 */
	uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send ) {
		return get_time_until_next_send( nsamples_to_send, uhd::time_spec_t::get_system_time() );
	}
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_ */
