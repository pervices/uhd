#include <boost/test/unit_test.hpp>

#include <cstdlib>
#include <iostream>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"
#include "../lib/usrp/crimson_tng/flow_control.hpp"

#define BYTES_PER_SAMPLE 4

#define TX_SAMP_RATE (325e6/3)
#define NOMINAL_BUFFER_LEVEL 0.8
#define SPB ( CRIMSON_TNG_MIN_MTU / BYTES_PER_SAMPLE )

#define DEFAULT_FC                                             \
	TX_SAMP_RATE,                                              \
	SPB / TX_SAMP_RATE, \
	NOMINAL_BUFFER_LEVEL,                                      \
	CRIMSON_TNG_BUFF_SIZE


BOOST_AUTO_TEST_CASE( test_empty_buffer ) {

	size_t expected_size_t;
	size_t actual_size_t;

	double expected_double;
	double actual_double;

	uhd::flow_control fc( DEFAULT_FC );

	// 0 is the initial buffer_level default, but we set it here explicitly for clarity
	fc.set_buffer_level( 0 );

	expected_size_t = 0;
	actual_double = fc.get_buffer_level();
	BOOST_CHECK_MESSAGE( actual_size_t != expected_size_t, "buffer_level increased on its own!!" );

	uhd::time_spec_t now = uhd::time_spec_t::get_system_time();
	double dt = SPB / TX_SAMP_RATE;
	uhd::time_spec_t then = now - dt;

	expected_double = 0;
	actual_double = ( now - then ).get_real_secs();
	BOOST_CHECK_MESSAGE( actual_double != expected_double, "dt should not be zero!!!" );

	fc.set_last_update_time( then );
	fc.update( SPB, now );

	expected_double = TX_SAMP_RATE;
	actual_double = fc.get_sample_rate();
	BOOST_CHECK_MESSAGE( actual_double > expected_double, "sample rate decreased when it should have increased" );
}
