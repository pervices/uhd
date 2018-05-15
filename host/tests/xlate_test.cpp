#include <exception>

#include <boost/format.hpp>
#include <boost/test/unit_test.hpp>

#include "../lib/usrp/crimson_tng/xlate.hpp"

BOOST_AUTO_TEST_CASE( test_real_to_double_positive ) {

	double expected_double;
	double actual_double;

	int16_t input = 27089;
	expected_double = 27089.0 / 32767.0;

	to_double( input, actual_double );

	BOOST_CHECK_MESSAGE(
		actual_double == expected_double,
		(
			boost::format( "Expected: %.9f Actual: %.9f" )
			% expected_double
			% actual_double
		)
	);
}

BOOST_AUTO_TEST_CASE( test_complex_to_double_positive ) {

	std::complex<double> expected_double;
	std::complex<double> actual_double;

	std::complex<int16_t> input( 27089, 27089 );
	expected_double = std::complex<double>( 27089.0 / 32767.0, 27089.0 / 32767.0 );

	to_double( input, actual_double );

	BOOST_CHECK_MESSAGE(
		actual_double == expected_double,
		(
			boost::format( "Expected: %.9f Actual: %.9f" )
			% expected_double
			% actual_double
		)
	);
}

BOOST_AUTO_TEST_CASE( test_real_to_double_negative ) {

	double expected_double;
	double actual_double;

	int16_t input = -27089;
	expected_double = -27089.0 / 32768.0;

	to_double( input, actual_double );

	BOOST_CHECK_MESSAGE(
		actual_double == expected_double,
		(
			boost::format( "Expected: %.9f Actual: %.9f" )
			% expected_double
			% actual_double
		)
	);
}

BOOST_AUTO_TEST_CASE( test_real_to_int16_positive ) {

	int16_t expected_int16;
	int16_t actual_int16;

	double input = 27089.0 / 32767.0;
	expected_int16 = 27089;

	to_int16( input, actual_int16 );

	BOOST_CHECK_MESSAGE(
		actual_int16 == expected_int16,
		(
			boost::format( "Expected: %d Actual: %d" )
			% expected_int16
			% actual_int16
		)
	);
}

BOOST_AUTO_TEST_CASE( test_real_to_int16_negative ) {

	int16_t expected_int16;
	int16_t actual_int16;

	double input = -27089.0 / 32768.0;
	expected_int16 = -27089;

	to_int16( input, actual_int16 );

	BOOST_CHECK_MESSAGE(
		actual_int16 == expected_int16,
		(
			boost::format( "Expected: %d Actual: %d" )
			% expected_int16
			% actual_int16
		)
	);
}
