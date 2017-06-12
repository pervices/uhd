#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_

#include <boost/core/ignore_unused.hpp>

#include <uhd/types/time_spec.hpp>

namespace uhd {

class flow_control {

public:

	flow_control( const double nominal_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size );
	flow_control( const uhd::flow_control &other );
	virtual ~flow_control() {}

	/**
	 * @return the sizeof the underlying buffer
	 */
	virtual size_t get_buffer_size();
	/**
	 * @return the desired level of the underlying buffer at steady-state operation
	 */
	virtual size_t get_nominal_buffer_level();
	/**
	 * @return the desired level of the underlying buffer at steady-state operation
	 */
	virtual double get_nominal_buffer_level_pcnt();
	/**
	 * @return the desired level of the underlying buffer at steady-state operation
	 */
	virtual double get_nominal_sample_rate();

	/**
	 * Report whether the specified time is less than the start of burst time.
	 *
	 * @param now  the time to compare with the start of burst time
	 * @return true if the specified time is less than the start of burst time, otherwise false
	 */
	virtual bool start_of_burst_pending( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() );
	/**
	 * Set the time for a start of burst
	 *
	 * @param now  the start of burst time
	 */
	virtual void set_start_of_burst_time( const uhd::time_spec_t & sob );
	/**
	 * Get the time for a start of burst
	 *
	 * @param now  the start of burst time [default is 0, when no start of burst is set]
	 */
	virtual uhd::time_spec_t get_start_of_burst_time();

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
	static inline size_t interp( const uhd::time_spec_t & a, const uhd::time_spec_t & b, const double sample_rate ) {

		size_t r;

		double ta = a.get_real_secs();
		double tb = b.get_real_secs();

		if ( tb > ta ) {
			double dt = tb - ta;
			double nsamps = sample_rate * dt;
			r = floor( nsamps );
		} else {
			r = 0;
		}

		return r;
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
	virtual size_t get_buffer_level( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() );
	/**
	 * set_buffer_level
	 *
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param level   the actual buffer level [samples]
	 */
	virtual void set_buffer_level( const size_t level, const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() );

	/**
	 * get_buffer_level_pcnt
	 *
	 * Get the (approximate) level of the buffer. Under normal operating
	 * conditions, this reflects the level of the actual tx buffer. Periodic
	 * updates of actual tx buffer levels are treated as rejected disturbances.
	 *
	 * @return the buffer level [%]
	 */
	inline double get_buffer_level_pcnt( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		return get_buffer_level( now ) / get_buffer_size();
	}
	/**
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param pcnt   the actual buffer level [%]
	 */
	inline void set_buffer_level_pcnt( const double pcnt, const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		set_buffer_level( pcnt * get_buffer_size(), now );
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
	virtual double get_sample_rate( const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() ) {
		BOOST_UNUSED( now );
		return get_nominal_sample_rate();
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
	virtual void update( const size_t nsamples_sent, const uhd::time_spec_t & now = uhd::time_spec_t::get_system_time() );

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
	virtual uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send, const uhd::time_spec_t &now = uhd::time_spec_t::get_system_time() );
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_ */
