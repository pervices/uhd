#include <boost/test/unit_test.hpp>

#include <cstdlib>
#include <iostream>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"
#include "../lib/usrp/crimson_tng/flow_control.hpp"

#define BYTES_PER_SAMPLE 4

//#define CONTRIVED
#ifdef CONTRIVED

/**
 * Just to be working with even numbers, let's make the following very very simple assumptions
 *
 * 1. PID sample rate, f_s_PID = 10 Hz
 * 2. TX sample rate, f_s = 5 * f_s_PID
 * 3. The buffer length is 100 samples
 * 4. The desired buffer level is 80% (i.e. 80 samples).
 */

#define f_s_PID  10
#define f_s      ( 5 * f_s_PID )
#define BUF_LEN  100
#define SP       0.8
#define MTU      10

#else

#define f_s_PID  CRIMSON_TNG_UPDATE_PER_SEC
#define f_s      ( CRIMSON_TNG_MASTER_CLOCK_RATE / 6 )
#define BUF_LEN  CRIMSON_TNG_BUFF_SIZE
#define SP       0.8
#define MTU      ( CRIMSON_TNG_MAX_MTU / BYTES_PER_SAMPLE )

#endif

#define DEFAULT_FC \
	f_s,           \
	f_s / MTU,     \
	SP,            \
	BUF_LEN

template<typename T>
static bool is_close( T a, T b, T eps = T(0.0001) ) {
	return abs( a - b ) <= eps * std::max( abs( a ), abs( b ) );
}

BOOST_AUTO_TEST_CASE( test_empty_buffer ) {

	size_t expected_size_t;
	size_t actual_size_t;

	uhd::flow_control fc( DEFAULT_FC );

	expected_size_t = 0;
	actual_size_t = fc.get_buffer_level();
	BOOST_CHECK_MESSAGE( actual_size_t == expected_size_t, "unexpected buffer level" );
}

BOOST_AUTO_TEST_CASE( test_time_until_first_send ) {

	double expected_double;
	double actual_double;

	uhd::flow_control fc( DEFAULT_FC );

	uhd::time_spec_t then = fc.get_time_until_next_send( MTU );

	expected_double = 0;
	actual_double = then.get_real_secs();

	BOOST_CHECK_MESSAGE( is_close( actual_double, expected_double ), "non-zero time to first send" );
}

BOOST_AUTO_TEST_CASE( test_buffer_level_after_first_update ) {

	ssize_t expected_ssize_t;
	ssize_t actual_ssize_t;

	uhd::flow_control fc( DEFAULT_FC );

	fc.update( MTU );

	expected_ssize_t = MTU;
	actual_ssize_t = fc.get_buffer_level();

	BOOST_CHECK_MESSAGE(
		is_close( (double) actual_ssize_t, (double) expected_ssize_t, 0.1 ),
		(
			boost::format( "unexpected buffer level( expected: %u, actual: %u)" )
			% expected_ssize_t
			% actual_ssize_t
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_time_until_second_send ) {

	double expected_double;
	double actual_double;

	uhd::time_spec_t now;
	uhd::time_spec_t then;

	uhd::flow_control fc( DEFAULT_FC );

	now = uhd::time_spec_t::get_system_time();
	fc.update( MTU );
	then = fc.get_time_until_next_send( MTU );
	size_t buffer_level = fc.get_buffer_level();

	// ensure we have a non-zero time to wait (albeit very, very small)
	expected_double = 0;
	actual_double = then.get_real_secs();
	BOOST_CHECK_MESSAGE( actual_double > expected_double, "negative time to second send" );

	// ensure that the time we wait until the second send does not exhaust the buffer resources
	expected_double = buffer_level / fc.get_sample_rate();
	actual_double = ( then - now ).get_real_secs();
	BOOST_CHECK_MESSAGE( actual_double < expected_double, "flow control not compensating after first packet" );
}

BOOST_AUTO_TEST_CASE( test_inter_pid_sample_convergence ) {

	size_t i;
	uhd::time_spec_t t;
	uhd::time_spec_t dt;
	uhd::time_spec_t prev_dt;

	double expected_double;
	double actual_double;

	bool expected_bool;
	bool actual_bool;

	size_t expected_size_t;
	size_t actual_size_t;

	ssize_t buffer_level;
	ssize_t prev_buffer_level;

	uhd::flow_control fc( DEFAULT_FC );

	const size_t max_iterations = BUF_LEN / MTU;

	for(
		t = uhd::time_spec_t::get_system_time(),
			i = 0,
			dt = uhd::time_spec_t( 0, 0 );
		i <= max_iterations;
		i++
	) {
		prev_buffer_level = fc.get_buffer_level();
		prev_dt = dt;

		dt = fc.get_time_until_next_send( MTU, t );
		t += dt; // i.e. sleep( dt )
		fc.update( MTU, t );

		buffer_level = fc.get_buffer_level();
		if ( is_close( SP, buffer_level / (double)BUF_LEN, 0.05 ) ) {
			break;
		}

		// ensure that the difference between actual and desired buffer levels converges to zero
		expected_double = abs( fc.nominal_buffer_level - prev_buffer_level );
		actual_double = abs( fc.nominal_buffer_level - buffer_level );
		BOOST_CHECK_MESSAGE(
			actual_double < expected_double,
			(
				boost::format( "buffer level not converging (expected: %u, actual: %u)" )
				% expected_double
				% actual_double
			).str()
		);

		// ensure that delay grows when converging on the set point
		expected_double = prev_dt.get_real_secs();
		actual_double = dt.get_real_secs();
		BOOST_CHECK_MESSAGE(
			actual_double >= expected_double,
			(
					boost::format( "delay not converging (expected: %u, actual: %u)" )
				% expected_double
				% actual_double
			).str()
		);
	}

	expected_bool = true;
	actual_bool = is_close( (double) fc.nominal_buffer_level, (double) fc.get_buffer_level(), (double) MTU );
	BOOST_CHECK_MESSAGE(
		actual_bool == expected_bool,
		(
			boost::format( "buffer levels did not converge (sp: %f, cv: %f)" )
			% fc.nominal_buffer_level
			% fc.get_buffer_level()
		).str()
	);

	expected_size_t = max_iterations;
	actual_size_t = i;
	BOOST_CHECK_MESSAGE(
		actual_size_t < expected_size_t,
		(
			boost::format( "buffer levels did not converge within %u iterations" )
			% max_iterations
		).str()
	);

	expected_double = fc.nominal_sample_rate;
	actual_double = fc.get_sample_rate();
	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, (double)MTU ),
		(
			boost::format( "sample rates did not converge (expected: %f, actual: %f)" )
			% fc.nominal_sample_rate
			% fc.get_sample_rate()
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_sob ) {

	const uhd::time_spec_t prebuffer_in_time( SP * CRIMSON_TNG_BUFF_SIZE / (double) f_s );
	uhd::time_spec_t now, then, dt;

	uhd::flow_control fc( DEFAULT_FC );

	// let's say that I want to send 5 minutes in the future
	dt =  uhd::time_spec_t( 60 * 5, 0 );
	now = uhd::time_spec_t::get_system_time();
	then = now + dt;

	// so we give flow_control an empty update with the future time
	fc.update( 0, then );

	// now, the next time that the flow_control tells us to send should be

	double expected_double = ( then - prebuffer_in_time - now ).get_real_secs();
	double actual_double;

	actual_double = fc.get_time_until_next_send( -1, now ).get_real_secs();

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double ),
		(
			boost::format( "not the expected amount of time to wait ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);
}
