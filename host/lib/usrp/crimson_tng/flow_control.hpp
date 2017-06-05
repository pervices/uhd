#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_

#include <iostream>

#include <cmath>

#include <mutex>

#include <boost/format.hpp>

#include <uhd/exception.hpp>

#include "../../../include/uhd/utils/pidc_tl.hpp"

namespace uhd {

/**
 * This is a slightly odd wrapper around a PID controller.
 *
 * What makes it odd, is that the software-based PID controller code cannot
 * sample the Process nearly fast enough due to the fact that it is I/O bound.
 *
 * However, we can infer the
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
			std::min( nominal_buffer_level_pcnt, 1 - nominal_buffer_level_pcnt ) * buffer_size,
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
		if ( nominal_sample_rate <= 0 ) {
			throw uhd::value_error(
				(
					boost::format( "Invalid sample_rate %f" )
					% nominal_sample_rate
				).str()
			);
		}

		// XXX: insert some sanity checks w.r.t. tx sample rate > pid sample rate, etc
	}

	virtual ~flow_control()
	{
	}

	size_t get_buffer_level() {
		size_t r;
		lock.lock();
		r = buffer_level;
		lock.unlock();
		return r;
	}
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

	double get_buffer_level_pcnt() {
		return get_buffer_level() / (double) buffer_size;
	}
	void set_buffer_level_pcnt( const double pcnt ) {
		set_buffer_level( pcnt * buffer_size );
	}

	double get_sample_rate() {
		double r;
		lock.lock();
		r = sample_rate;
		lock.unlock();
		return r;
	}

	void update( const size_t nsamples_sent, const uhd::time_spec_t & now ) {

		uhd::time_spec_t then;
		uhd::time_spec_t dt;
		size_t nsamples_consumed;
		double sp;
		double pv;
		double cv;

		lock.lock();

		if ( first_update ) {

			then = now;
			then -= nominal_buffer_level / nominal_sample_rate;

			pidc.set_last_time( then.get_real_secs() );

			buffer_level = nominal_buffer_level;
		}

		then = uhd::time_spec_t( pidc.get_last_time() );
		dt = now - then;
		nsamples_consumed = round( dt.get_real_secs() * nominal_sample_rate );

		buffer_level -= nsamples_consumed;
		buffer_level += nsamples_sent;

		if ( buffer_level < 0 ) {
			throw runtime_error(
				(
					boost::format( "buffer level has fallen below 0 and is at %u" )
					% buffer_level
				).str()
			);
		}

		sp = nominal_buffer_level;
		pv = buffer_level;
		if ( now >= pidc.get_last_time() + 2 / pid_sample_rate ) {
			cv = pidc.update_control_variable( sp, pv, now.get_real_secs() );
		} else {
			cv = pidc.get_control_variable();
		}

		sample_rate = nominal_sample_rate + ( cv - pv ) * 2 / pid_sample_rate;

		if ( BOOST_UNLIKELY( sample_rate < 0 ) ) {
			throw runtime_error(
				(
					boost::format( "sample rate has fallen below 0 and is at %f" )
					% sample_rate
				).str()
			);
		}

		first_update = false;

		lock.unlock();
	}
	void update( const size_t nsamples_sent ) {
		update( nsamples_sent, uhd::time_spec_t::get_system_time() );
	}

	uhd::time_spec_t get_time_until_next_send( const size_t mtu, const uhd::time_spec_t &now ) {

		uhd::time_spec_t r;
		uhd::time_spec_t dt;
		size_t nsamples_consumed;

		lock.lock();

		if ( first_update ) {
			r = uhd::time_spec_t( 0, 0 );
		} else {
			r = uhd::time_spec_t( buffer_level / sample_rate );
		}

		lock.unlock();
		return r;
	}
	uhd::time_spec_t get_time_until_next_send( const size_t mtu ) {
		return get_time_until_next_send( mtu, uhd::time_spec_t::get_system_time() );
	}
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_ */
