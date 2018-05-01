#ifndef HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_

#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/core/ignore_unused.hpp>

#include <uhd/types/time_spec.hpp>

namespace uhd {

class flow_control {

public:
	typedef boost::shared_ptr<uhd::flow_control> sptr;
	virtual ~flow_control() {}

	/**
	 * @return the sizeof the underlying buffer
	 */
	virtual size_t get_buffer_size() = 0;
	/**
	 * @return the desired level of the underlying buffer at steady-state operation
	 */
	virtual size_t get_nominal_buffer_level() = 0;
	/**
	 * @return the desired level of the underlying buffer at steady-state operation
	 */
	inline double get_nominal_buffer_level_pcnt() {
		return get_nominal_buffer_level() / (double) get_buffer_size();
	}
	/**
	 * @return the desired level of the underlying buffer at steady-state operation
	 */
	virtual double get_nominal_sample_rate() = 0;

	/**
	 * Report whether the specified time is less than the start of burst time.
	 *
	 * @param now  the time to compare with the start of burst time
	 * @return true if the specified time is less than the start of burst time, otherwise false
	 */
	virtual bool start_of_burst_pending( const uhd::time_spec_t & now ) = 0;
	/**
	 * Set the time for a start of burst
	 *
	 * @param now  the start of burst time
	 */
	virtual void set_start_of_burst_time( const uhd::time_spec_t & sob ) = 0;
	/**
	 * Get the time for a start of burst
	 *
	 * @param now  the start of burst time [default is 0, when no start of burst is set]
	 */
	virtual uhd::time_spec_t get_start_of_burst_time() = 0;

	/**
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
	 * Get the (approximate) level of the buffer. Under normal operating
	 * conditions, this reflects the level of the actual tx buffer. Periodic
	 * updates of actual tx buffer levels are treated as rejected disturbances.
	 *
	 * Inter-sample buffer levels use linear approximation based on the
	 * current sample rate.
	 *
	 * @return the buffer level [samples]
	 */
	virtual ssize_t get_buffer_level( const uhd::time_spec_t & now ) = 0;
	/**
	 * Set the buffer level, presumably based on valid data.
	 *
	 * Valid values of buffer level are in the range [0, buffer size), that
	 * is the range zero-inclusive but strictly less than the buffer size.
	 *
	 * @param level   the actual buffer level [samples]
	 * @param now     the current clock time
	 */
	virtual void set_buffer_level( const size_t level, const uhd::time_spec_t & now ) = 0;
	/**
	 * Set the buffer level asynchronously.
	 *
	 * Valid values of buffer level are in the range [0, buffer size), that
	 * is the range zero-inclusive but strictly less than the buffer size.
	 *
	 * @param level   the actual buffer level [samples]
	 */
	virtual void set_buffer_level_async( const size_t level ) = 0;

	/**
	 * Get the (approximate) level of the buffer. Under normal operating
	 * conditions, this reflects the level of the actual tx buffer. Periodic
	 * updates of actual tx buffer levels are treated as rejected disturbances.
	 *
	 * @return the buffer level [%]
	 */
	inline double get_buffer_level_pcnt( const uhd::time_spec_t & now ) {
		return get_buffer_level( now ) / (double) get_buffer_size();
	}
	/**
	 * Set the buffer level, presumably based on valid data.
	 *
	 * @param pcnt   the actual buffer level [%]
	 */
	inline void set_buffer_level_pcnt( const double pcnt, const uhd::time_spec_t & now ) {
		set_buffer_level( pcnt * get_buffer_size(), now );
	}

	/**
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
	virtual double get_sample_rate( const uhd::time_spec_t & now ) {
		boost::ignore_unused( now );
		return get_nominal_sample_rate();
	}

	/**
	 * Users may set the device sample rate after the flow controller is
	 * initantiated. In that case, when updating the device sample rate
	 * also update the flow controller sample rate.
	 *
	 * @param now the time to set the sample rate
	 * @param rate the new sample rate
	 */
	virtual void set_sample_rate( const uhd::time_spec_t & now, const double & rate ) = 0;

	/**
	 * The primary purpose of a flow controller is to act as a rate-limiter.
	 * If samples are sent too quickly, input buffers can overflow and signals
	 * are corrupted. If samples are sent too slowly, input buffers can
	 * underflow resulting in signal corruption (typically zeros are inserted).
	 *
	 * This functions determines the amount of time to wait until it is
	 * necessary to send data in order to maintain the nominal sample rate.
	 *
	 * Users will typically call this function before sending data.
	 *
	 * @param nsamples_to_send   The number of samples the caller would like to send
	 * @param now                The time to wait from
	 * @return                   The amount of time to wait from 'now'
	 */
	virtual uhd::time_spec_t get_time_until_next_send( const size_t nsamples_to_send, const uhd::time_spec_t &now ) = 0;


	/**
	 * Report to the flow controller, that the caller has sent additional
	 * samples and that the flow controller should adjust its buffer levels,
	 * internal state, and sample rate, appropriately.
	 *
	 * Users will typically call this function directly after sending data.
	 *
	 * @param nsamples_sent   The number of samples sent
	 * @param now             The time at which the samples were sent [default: current system time]
	 */
	virtual void update( const size_t nsamples_sent, const uhd::time_spec_t & now ) = 0;

protected:

	flow_control() {}
	flow_control( const double nominal_sample_rate, const double nominal_buffer_level_pcnt, const size_t buffer_size )
	{
		boost::ignore_unused( nominal_sample_rate );
		boost::ignore_unused( nominal_buffer_level_pcnt );
		boost::ignore_unused( buffer_size );
	}
};

}

#include "flow_control_nonlinear.hpp"

#endif /* HOST_LIB_USRP_CRIMSON_TNG_FLOW_CONTROL_HPP_ */
