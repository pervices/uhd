#include <boost/test/unit_test.hpp>

#include <cstdlib>
#include <iostream>

#include <uhd/transport/vrt_if_packet.hpp>
#include <boost/format.hpp>

#include "../include/uhd/usrp/multi_usrp.hpp"

using namespace uhd;
using namespace usrp;

struct usrp_dev {
	multi_usrp::sptr usrp;
	device::sptr dev;
};

#ifndef ARRAY_SIZE
#define ARRAY_SIZE( x ) ((int)( sizeof( x ) / sizeof( (x)[ 0 ] ) ))
#endif

static double get_atten( device::sptr &dev ) {
	double r = -1.0;
	uhd::property_tree::sptr tree = dev->get_tree();
	r = tree->access<double>( "/mboards/0/dboards/A/rx_frontends/Channel_A/atten/value" ).get();
	return r;
}

static double get_gain( device::sptr &dev ) {
	double r = -1.0;
	uhd::property_tree::sptr tree = dev->get_tree();
	r = tree->access<double>( "/mboards/0/dboards/A/rx_frontends/Channel_A/gain/value" ).get();
	return r;
}

static int get_lna( device::sptr &dev ) {
	int r = -1;
	uhd::property_tree::sptr tree = dev->get_tree();
	r = tree->access<int>( "/mboards/0/dboards/A/rx_frontends/Channel_A/freq/lna" ).get();
	return r;
}

static int get_band( device::sptr &dev ) {
	int r = -1;
	uhd::property_tree::sptr tree = dev->get_tree();
	r = tree->access<int>( "/mboards/0/dboards/A/rx_frontends/Channel_A/freq/band" ).get();
	return r;
}

static void get_usrp_dev( struct usrp_dev *ud ) {
	device_addr_t addr;
	BOOST_REQUIRE_NE( (void *) NULL, (void *) ud );
	ud->usrp = uhd::usrp::multi_usrp::make( addr );
	BOOST_REQUIRE_NE( (void *) NULL, (void *) ud->usrp.get() );
	ud->dev =  ud->usrp->get_device();
	BOOST_REQUIRE_NE( (void *) NULL, ud->dev.get() );
	// we are primarily interested in testing high-band rx gain
	uhd::tune_request_t tune( 2400000000 );
	ud->usrp->set_rx_freq( tune );
	int band = get_band( ud->dev );
	BOOST_REQUIRE_EQUAL( 1, band );
}

BOOST_AUTO_TEST_CASE( test_range_AB ) {
	struct usrp_dev ud;
	get_usrp_dev( & ud );

	double probed_rx_gain[] = { -1.0, 0.0, 31.5, };

	double expected_rx_gain[] = { 0.0, 0.0, 31.5, };
	double actual_rx_gain[ ARRAY_SIZE( probed_rx_gain ) ];

	// atten is maxed
	double expected_atten[] = { 31.75*4, 31.75*4, 31.75*4, };
	double actual_atten[ ARRAY_SIZE( probed_rx_gain ) ];

	// lna is bypassed
	int expected_lna[] = { 1, 1, 1, };
	int actual_lna[ ARRAY_SIZE( probed_rx_gain ) ];

	// gain is varied from 0 to 31.5
	double expected_gain[] = { 0.0, 0.0, 31.5*4, };
	double actual_gain[ ARRAY_SIZE( probed_rx_gain ) ];

	for( int k = 0; k < ARRAY_SIZE( probed_rx_gain ); k++ ) {

		ud.usrp->set_rx_gain( probed_rx_gain[ k ], "" );

		actual_rx_gain[ k ] = ud.usrp->get_rx_gain();
		actual_atten[ k ] = get_atten( ud.dev );
		actual_gain[ k ] = get_gain( ud.dev );
		actual_lna[ k ] = get_lna( ud.dev );

		BOOST_CHECK_CLOSE( expected_rx_gain[ k ], actual_rx_gain[ k ], 0.01 );
		BOOST_CHECK_CLOSE( expected_atten[ k ], actual_atten[ k ], 0.01 );
		BOOST_CHECK_CLOSE( expected_gain[ k ], actual_gain[ k ], 0.01 );
		BOOST_CHECK_EQUAL( expected_lna[ k ], actual_lna[ k ] );
	}
}

BOOST_AUTO_TEST_CASE( test_range_BC ) {
	struct usrp_dev ud;
	get_usrp_dev( & ud );

	double probed_rx_gain[] = { 31.75, 63.25, };

	double expected_rx_gain[] = { 31.75, 63.25, };
	double actual_rx_gain[ ARRAY_SIZE( probed_rx_gain ) ];

	//  atten is varied from 31.75 to 0
	double expected_atten[] = { 31.5*4, 0.0, };
	double actual_atten[ ARRAY_SIZE( probed_rx_gain ) ];

	// lna is bypassed
	int expected_lna[] = { 1, 1, };
	int actual_lna[ ARRAY_SIZE( probed_rx_gain ) ];

	// gain is maxed
	double expected_gain[] = { 31.5*4, 31.5*4, };
	double actual_gain[ ARRAY_SIZE( probed_rx_gain ) ];

	for( int k = 0; k < ARRAY_SIZE( probed_rx_gain ); k++ ) {

		ud.usrp->set_rx_gain( probed_rx_gain[ k ], "" );

		actual_rx_gain[ k ] = ud.usrp->get_rx_gain();
		actual_atten[ k ] = get_atten( ud.dev );
		actual_gain[ k ] = get_gain( ud.dev );
		actual_lna[ k ] = get_lna( ud.dev );

		BOOST_CHECK_CLOSE( expected_rx_gain[ k ], actual_rx_gain[ k ], 0.01 );
		BOOST_CHECK_CLOSE( expected_atten[ k ], actual_atten[ k ], 0.01 );
		BOOST_CHECK_CLOSE( expected_gain[ k ], actual_gain[ k ], 0.01 );
		BOOST_CHECK_EQUAL( expected_lna[ k ], actual_lna[ k ] );
	}
}

BOOST_AUTO_TEST_CASE( test_range_CD ) {
	struct usrp_dev ud;
	get_usrp_dev( & ud );

	double probed_rx_gain[] = { 63.5, 83.25, };

	double expected_rx_gain[] = { 63.5, 83.25, };
	double actual_rx_gain[ ARRAY_SIZE( probed_rx_gain ) ];

	// atten is varied from 20 to 0
	double expected_atten[] = { 19.75*4, 0.0, };
	double actual_atten[ ARRAY_SIZE( probed_rx_gain ) ];

	// lna is not bypassed
	int expected_lna[] = { 0, 0, };
	int actual_lna[ ARRAY_SIZE( probed_rx_gain ) ];

	// gain is maxed
	double expected_gain[] = { 31.5*4, 31.5*4, };
	double actual_gain[ ARRAY_SIZE( probed_rx_gain ) ];

	for( int k = 0; k < ARRAY_SIZE( probed_rx_gain ); k++ ) {

		ud.usrp->set_rx_gain( probed_rx_gain[ k ], "" );

		actual_rx_gain[ k ] = ud.usrp->get_rx_gain();
		actual_atten[ k ] = get_atten( ud.dev );
		actual_gain[ k ] = get_gain( ud.dev );
		actual_lna[ k ] = get_lna( ud.dev );

		BOOST_CHECK_CLOSE( expected_rx_gain[ k ], actual_rx_gain[ k ], 0.01 );
		BOOST_CHECK_CLOSE( expected_atten[ k ], actual_atten[ k ], 0.01 );
		BOOST_CHECK_CLOSE( expected_gain[ k ], actual_gain[ k ], 0.01 );
		BOOST_CHECK_EQUAL( expected_lna[ k ], actual_lna[ k ] );
	}
}

