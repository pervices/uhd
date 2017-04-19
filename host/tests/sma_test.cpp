#include <boost/test/unit_test.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/utils/byteswap.hpp>
#include <boost/format.hpp>
#include <cstdlib>
#include <iostream>

#include "uhd/utils/sma.hpp"

BOOST_AUTO_TEST_CASE( test_W16_N0 ) {
	uhd::sma sma;

	double expected = 0;
	double actual = sma.get_average();

	BOOST_CHECK_CLOSE( expected, actual, 1e-6 );
}

BOOST_AUTO_TEST_CASE( test_W16_N2 ) {
	uhd::sma sma;

	sma.update( 1.0 );
	sma.update( 3.0 );

	double expected = 2.0;
	double actual = sma.get_average();

	BOOST_CHECK_CLOSE( expected, actual, 1e-6 );
}

BOOST_AUTO_TEST_CASE( test_W3_N3 ) {
	uhd::sma sma;

	sma.update( 1.0 );
	sma.update( 3.0 );
	sma.update( 2.0 );

	double expected = 2.0;
	double actual = sma.get_average();

	BOOST_CHECK_CLOSE( expected, actual, 1e-6 );
}

BOOST_AUTO_TEST_CASE( test_W3_N4 ) {
	uhd::sma sma(3);

	sma.update( 100.0 );
	sma.update( 1.0 );
	sma.update( 3.0 );
	sma.update( 2.0 );

	double expected = 2.0;
	double actual = sma.get_average();

	BOOST_CHECK_CLOSE( expected, actual, 1e-6 );
}
