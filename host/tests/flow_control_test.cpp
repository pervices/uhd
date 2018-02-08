#include <boost/test/unit_test.hpp>

#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"
#define DEBUG_FLOW_CONTROL 1
#include "../lib/usrp/crimson_tng/flow_control.hpp"

#define BYTES_PER_SAMPLE 4

#define f_s      ( CRIMSON_TNG_MASTER_CLOCK_RATE / 6 )
#define BUF_LEN  CRIMSON_TNG_BUFF_SIZE
#define SP       0.8
#define MTU      ( CRIMSON_TNG_MAX_MTU / BYTES_PER_SAMPLE )

#define FC_DEFAULT \
	f_s, SP, BUF_LEN

template<typename T>
static bool is_close( T a, T b, T eps = T(0.0001) ) {
	T diff = std::abs( a - b );
	T _max = std::max( std::abs( a ), std::abs( b ) );
	T _max_eps = eps * _max;
	return diff <= _max_eps;
}

BOOST_AUTO_TEST_CASE( test_empty_buffer ) {

	size_t expected_size_t;
	size_t actual_size_t;

	uhd::time_spec_t now = 0.0;
	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	expected_size_t = 0;
	actual_size_t = fc->get_buffer_level( now );
	BOOST_CHECK_MESSAGE( actual_size_t == expected_size_t, "unexpected buffer level" );
}

BOOST_AUTO_TEST_CASE( test_set_buffer_level ) {

	size_t expected_size_t;
	size_t actual_size_t;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	uhd::time_spec_t now = 0.0;

	fc->set_buffer_level( MTU, now );

	expected_size_t = MTU;
	actual_size_t = fc->get_buffer_level( now );
	BOOST_CHECK_MESSAGE( actual_size_t == expected_size_t, ( boost::format( "buffer_level changed! (expected: %u, actual: %u" ) % expected_size_t % actual_size_t ).str() );
}

BOOST_AUTO_TEST_CASE( test_set_buffer_level_to_buffer_size ) {

	size_t expected_size_t;
	size_t actual_size_t;

	std::exception_ptr expected_exception_ptr;
	std::exception_ptr actual_exception_ptr;

	uhd::time_spec_t now = 0.0;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	expected_size_t = fc->get_buffer_level( now );
	expected_exception_ptr = NULL;
	try {
		// XXX: this should throw an exception
		fc->set_buffer_level( fc->get_buffer_size(), now );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}
	actual_size_t = fc->get_buffer_level( now );
	BOOST_CHECK_MESSAGE( actual_exception_ptr != expected_exception_ptr, "no exception was thrown!" );
	BOOST_CHECK_MESSAGE( actual_size_t == expected_size_t, ( boost::format( "buffer_level changed! (expected: %u, actual: %u" ) % expected_size_t % actual_size_t ).str() );
}

BOOST_AUTO_TEST_CASE( test_set_buffer_level_too_large ) {

	size_t expected_size_t;
	size_t actual_size_t;

	std::exception_ptr expected_exception_ptr;
	std::exception_ptr actual_exception_ptr;

	uhd::time_spec_t now = 0.0;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	expected_size_t = fc->get_buffer_level( now );
	expected_exception_ptr = NULL;
	try {
		fc->set_buffer_level( -1, now );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}
	actual_size_t = fc->get_buffer_level( now );
	BOOST_CHECK_MESSAGE( actual_exception_ptr != expected_exception_ptr, "no exception was thrown!" );
	BOOST_CHECK_MESSAGE( actual_size_t == expected_size_t, ( boost::format( "buffer_level changed! (expected: %u, actual: %u" ) % expected_size_t % actual_size_t ).str() );
}

BOOST_AUTO_TEST_CASE( test_time_until_first_send ) {

	double expected_double;
	double actual_double;

	uhd::time_spec_t now = 0.0;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	uhd::time_spec_t then = fc->get_time_until_next_send( MTU, now );

	expected_double = 0;
	actual_double = then.get_real_secs();

	BOOST_CHECK_MESSAGE(
		actual_double <= expected_double,
		(
			boost::format( "positive time to first send without SoB (expected: %f, actual: %f)" )
			% expected_double
			% actual_double
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_time_until_send_at_steady_state ) {

	double expected_double;
	double actual_double;

	uhd::time_spec_t now = 0.0;
	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	fc->set_buffer_level( fc->get_nominal_buffer_level(), now );
	uhd::time_spec_t then = fc->get_time_until_next_send( MTU, now );

	expected_double = 0;
	actual_double = then.get_real_secs();

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double ),
		(
			boost::format( "wrong time until send at steady-state (expected: %f, actual: %f)" )
			% expected_double
			% actual_double
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_time_until_send_at_one_mtu_above_steady_state ) {

	double expected_double;
	double actual_double;

	uhd::time_spec_t now = 0.0;
	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	fc->set_buffer_level( fc->get_nominal_buffer_level() + MTU, now );
	uhd::time_spec_t then = fc->get_time_until_next_send( MTU, now );

	expected_double = MTU / fc->get_nominal_sample_rate();
	actual_double = then.get_real_secs();

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, 1 / fc->get_nominal_sample_rate() ),
		(
			boost::format( "wrong time until send at steady-state (expected: %f, actual: %f)" )
			% expected_double
			% actual_double
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_buffer_level_after_first_update ) {

	ssize_t expected_ssize_t;
	ssize_t actual_ssize_t;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	uhd::time_spec_t now = 0.0;

	fc->update( MTU, now );

	expected_ssize_t = MTU;
	actual_ssize_t = fc->get_buffer_level( now );

	BOOST_CHECK_MESSAGE(
		actual_ssize_t == expected_ssize_t,
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

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	now = 0.0;
	fc->update( MTU, now );
	then = fc->get_time_until_next_send( MTU, now );
	size_t buffer_level = fc->get_buffer_level( now );

	// ensure that the time we wait until the second send does not exhaust the buffer resources
	expected_double = buffer_level / fc->get_sample_rate( now );
	actual_double = ( then - now ).get_real_secs();
	BOOST_CHECK_MESSAGE( actual_double < expected_double, "flow control not compensating after first packet" );
}

BOOST_AUTO_TEST_CASE( test_convergence_from_below ) {

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

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	const size_t mtu = MTU / 10;
	const size_t max_iterations = BUF_LEN / mtu;

	for(
		t = 0.0,
			i = 0;
		i <= max_iterations;
		i++
	) {
		prev_buffer_level = fc->get_buffer_level( t );
		prev_dt = dt;

		dt = fc->get_time_until_next_send( MTU, t );
		if ( dt > 0.0 ) {
			t += dt; // i.e. sleep( dt )
		}
		fc->update( MTU, t );

		buffer_level = fc->get_buffer_level( t );
		if (
			true
			&& is_close( (double)fc->get_nominal_buffer_level(), (double) buffer_level, (double)MTU )
			&& buffer_level >= (ssize_t)fc->get_nominal_buffer_level()
		) {
			break;
		}

		// ensure that the difference between actual and desired buffer levels converges to zero
		expected_double = abs( (double) ( fc->get_nominal_buffer_level() - prev_buffer_level ) );
		actual_double = abs( (double) ( fc->get_nominal_buffer_level() - buffer_level ) );
		BOOST_CHECK_MESSAGE(
			actual_double < expected_double,
			(
				boost::format( "buffer level not converging (expected: %u, actual: %u)" )
				% expected_double
				% actual_double
			).str()
		);

		if ( i > 0 ) {
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
	}

	expected_bool = true;
	actual_bool = is_close( (double) fc->get_nominal_buffer_level(), (double) fc->get_buffer_level( t ), (double) MTU );
	BOOST_CHECK_MESSAGE(
		actual_bool == expected_bool,
		(
			boost::format( "buffer levels did not converge (sp: %f, cv: %f)" )
			% fc->get_nominal_buffer_level()
			% fc->get_buffer_level( t )
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
}

BOOST_AUTO_TEST_CASE( test_convergence_from_above ) {

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

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	const size_t mtu = MTU / 10;
	const size_t max_iterations = BUF_LEN / mtu;

	for(
		t = 0.0,
			i = 0,
			fc->set_buffer_level( fc->get_buffer_size() - 1, t );
		i <= max_iterations;
		i++
	) {
		prev_buffer_level = fc->get_buffer_level( t );
		prev_dt = dt;

		dt = fc->get_time_until_next_send( mtu, t );
		if ( dt > 0.0 ) {
			t += dt; // i.e. sleep( dt )
		}
		fc->update( 0, t );

		buffer_level = fc->get_buffer_level( t );
		if (
			true
			&& is_close( (double)fc->get_nominal_buffer_level(), (double) buffer_level, (double)mtu )
			&& buffer_level <= (ssize_t)fc->get_nominal_buffer_level()
		) {
			break;
		}

		// should not get here
	}

	expected_bool = true;
	actual_bool = is_close( (double) fc->get_nominal_buffer_level(), (double) fc->get_buffer_level( t ), (double) mtu );
	BOOST_CHECK_MESSAGE(
		actual_bool == expected_bool,
		(
			boost::format( "buffer levels did not converge (sp: %f, cv: %f)" )
			% fc->get_nominal_buffer_level()
			% fc->get_buffer_level( t )
		).str()
	);

	expected_size_t = 1;
	actual_size_t = i;
	BOOST_CHECK_MESSAGE(
		actual_size_t <= expected_size_t,
		(
			boost::format( "buffer levels did not converge within %u iterations" )
			% expected_size_t
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_initial_sob ) {

	double expected_double;
	double actual_double;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	expected_double = 0;
	actual_double = fc->get_start_of_burst_time().get_real_secs();
	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, 0.000000001 ),
		(
			boost::format( "nonzero initial start of burst! (expected: %f, actual: %f)" )
			% expected_double
			% actual_double
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_sob ) {

	uhd::time_spec_t now, then, dt;

	double expected_double;
	double actual_double;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	// let's say that I want to send 5 minutes in the future
	dt =  uhd::time_spec_t( 60 * 5, 0 );
	now = uhd::time_spec_t::get_system_time();
	then = now + dt;
	fc->set_start_of_burst_time( then );

	// now, we should be able to start sending right away
	expected_double = 0;
	actual_double = fc->get_time_until_next_send( MTU, now ).get_real_secs();

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double ),
		(
			boost::format( "not the expected amount of time to wait ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);

	// NEXT, we should be able to fill up the buffer to ~SP
	// and then we should be forced to wait approximately dt seconds (assuming no passage of time)
	for(
		size_t i = 0;
		true
			&& i < fc->get_buffer_size() / MTU
			&& is_close( 0.0, fc->get_time_until_next_send( MTU, now ).get_real_secs() )
		;
		i++
	) {
		fc->update( MTU, now );
	}

	expected_double = fc->get_nominal_buffer_level();
	actual_double = fc->get_buffer_level( now );
	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, fc->get_buffer_size() / (double)MTU ),
		(
			boost::format( "not the expected buffer level after prefill ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);

	expected_double = dt.get_real_secs();
	actual_double = fc->get_time_until_next_send( MTU, now ).get_real_secs();
	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, 0.1 ),
		(
			boost::format( "not the time until next send ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);

	// FURTHERMORE, at the SoB time, the buffer level should be nominal
	expected_double = fc->get_nominal_buffer_level();
	actual_double = fc->get_buffer_level( then );

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, 0.15 ),
		(
			boost::format( "not the expected buffer level at SoB time ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);

	// FURTHERMORE, up to and including the SoB time, the sample rate should be nominal
	expected_double = f_s;
	actual_double = fc->get_sample_rate( then );

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double ),
		(
			boost::format( "not the expected sample rate at SoB time ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);

	// LASTLY, at 1 MTU's worth of time past the SoB, the buffer level should be
	// about 1 MTU's worth of samples below the set point, assuming we do not send any samples
	then += MTU / fc->get_nominal_sample_rate();

	expected_double = fc->get_nominal_buffer_level() - MTU;
	actual_double = fc->get_buffer_level( then );

	BOOST_CHECK_MESSAGE(
		is_close( actual_double, expected_double, MTU / (double) fc->get_buffer_size() ),
		(
			boost::format( "level did not drop after SoB ( expected: %f, actual: %f )" )
			% expected_double
			% actual_double
		).str()
	);
}

BOOST_AUTO_TEST_CASE( test_overflow ) {

	uhd::time_spec_t now, then, dt;

	std::exception_ptr expected_exception_ptr;
	std::exception_ptr actual_exception_ptr;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	expected_exception_ptr = NULL;
	actual_exception_ptr = NULL;
	try {
		for(
			size_t i = 0;
			i < 2 * fc->get_buffer_size() / MTU;
			i++
		) {
			// we should overflow if we keep putting samples into the buffer
			fc->update( MTU, now );
		}
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE( actual_exception_ptr != expected_exception_ptr, "no exception was thrown!" );
}


BOOST_AUTO_TEST_CASE( test_underflow ) {

	uhd::time_spec_t now, then, dt;

	std::exception_ptr expected_exception_ptr;
	std::exception_ptr actual_exception_ptr;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( FC_DEFAULT );

	expected_exception_ptr = NULL;
	actual_exception_ptr = NULL;
	try {
		for(
			size_t i = 0;
			i < 2 * fc->get_buffer_size() / MTU;
			i++
		) {
			// we should overflow if we keep putting samples into the buffer
			fc->update( MTU, now + 5.0 );
		}
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE( actual_exception_ptr != expected_exception_ptr, "no exception was thrown!" );
}
