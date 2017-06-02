#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_

#include <mutex>

#include <boost/format.hpp>

#include <uhd/exception.hpp>

#include "../../../include/uhd/utils/pidc_tl.hpp"

namespace uhd {

class flow_control {

public:

	const size_t buffer_size;
	const double nominal_buffer_level;
	const double nominal_sample_rate;
	const double max_fc_sample_rate;

	flow_control( const double sample_rate, const double max_fc_sample_rate, const double buffer_level_pcnt, const size_t buffer_size )
	:
		buffer_size( buffer_size ),
		nominal_buffer_level( buffer_level_pcnt ),
		nominal_sample_rate( sample_rate ),
		max_fc_sample_rate( max_fc_sample_rate ),
		buffer_level( 0 ),
		sample_rate( 0 ),
		pidc(
			buffer_level_pcnt,
			std::min( buffer_level_pcnt, 1 - buffer_level_pcnt ),
			2 / max_fc_sample_rate
		),
		first_update( true )
	{
		if ( buffer_level_pcnt < 0 || buffer_level_pcnt > 1 ) {
			throw uhd::value_error(
				(
					boost::format( "Invalid buffer level %f" )
					% buffer_level
				).str()
			);
		}
		if ( sample_rate <= 0 ) {
			throw uhd::value_error(
				(
					boost::format( "Invalid sample_rate %f" )
					% sample_rate
				).str()
			);
		}
	}

	virtual ~flow_control()
	{
	}

	void set_buffer_level( const size_t level ) {
		lock.lock();
		buffer_level = level;
		lock.unlock();
	}

	double get_sample_rate() {
		double r;
		lock.lock();
		r = buffer_level;
		lock.unlock();
		return r;
	}

	void set_last_update_time( const uhd::time_spec_t & then ) {
		pidc.set_last_time( then.get_real_secs() );
		first_update = false;
	}

	void update( const size_t nsamples_sent, const uhd::time_spec_t & now ) {
		size_t nsamples_consumed;
		uhd::time_spec_t dt;

		lock.lock();

		if ( first_update ) {
			pidc.set_last_time( now.get_real_secs() );
			first_update = false;
		}
		dt = get_elapsed_time( now );
		double nsamples_cons = dt.get_real_secs() / nominal_sample_rate;
		nsamples_consumed = (size_t) nsamples_cons;

		buffer_level += nsamples_sent;
		buffer_level -= nsamples_consumed;

		update_sample_rate( now );

		lock.unlock();
	}
	void update( const size_t nsamples_sent ) {
		update( nsamples_sent, uhd::time_spec_t::get_system_time() );
	}

	size_t get_buffer_level() {
		size_t r;
		lock.lock();
		r = buffer_level;
		lock.unlock();
		return r;
	}

	double get_buffer_level_pcnt() {
		return get_buffer_level() / (double) buffer_size;
	}

protected:
	std::recursive_mutex lock;

	bool first_update;

	size_t buffer_level;
	double sample_rate;
	pidc_tl pidc;

	void update_sample_rate( const uhd::time_spec_t now ) {
		// XXX: mutex must be held!!!

		double dt = now.get_real_secs() - pidc.get_last_time();
		size_t prev_buffer_level = buffer_level;

		buffer_level = buffer_size * pidc.update_control_variable( nominal_buffer_level , get_buffer_level_pcnt() );

		if ( BOOST_UNLIKELY( 0 == dt ) ) {
			sample_rate = ( buffer_level - prev_buffer_level ) / (double) buffer_size * max_fc_sample_rate;
		} else {
			sample_rate = ( buffer_level - prev_buffer_level ) / dt;
		}
	}

	uhd::time_spec_t get_elapsed_time( const uhd::time_spec_t now ) {
		// XXX: mutex must be held!!!

		uhd::time_spec_t last;

		last = pidc.get_last_time();
		if ( BOOST_UNLIKELY( last > now ) ) {
			throw uhd::runtime_error(
				(
					boost::format( "last time is in the future by %f s" )
					% ( last - now ).get_real_secs()
				).str()
			);
		}

		return last - now;
	}
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_ */
