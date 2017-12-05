#include <iostream>
#include <limits>
#include <cmath>

#include <boost/test/unit_test.hpp>
#include <boost/format.hpp>

#include "uhd/exception.hpp"
#include "uhd/property_tree.hpp"
#include "uhd/types/ranges.hpp"

using namespace uhd;

extern double choose_dsp_nco_shift( double target_freq, double sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, double & lo_freq /* output */ );

// test fixture
struct fix {
	fix( double rate ) {
		dsp = property_tree::make();
		dsp->create<meta_range_t>( "/freq/range" ).set( meta_range_t( 0, 162.5e6 ) );
		dsp->create<double>( "/rate/value" ).set( rate );

		rf = property_tree::make();
     }
     ~fix() {}

     property_tree::sptr dsp;
     property_tree::sptr rf;
};

static void common( double fc, double samp_rate, bool expect_exception ) {

	bool expected_bool;
	bool actual_bool;
	double expected_double;
	double actual_double;

	double lo_freq;
	double nco_shift;
	double sign = 1;

	fix f( samp_rate );

	expected_bool = expect_exception;
	actual_bool = false;
	expected_double = 0;
	actual_double = std::numeric_limits<double>::infinity();

	try {
		nco_shift = choose_dsp_nco_shift( fc, sign, f.dsp, f.rf, lo_freq );
	} catch( ... ) {
		actual_bool = true;
	}

	BOOST_CHECK_MESSAGE(
		expected_bool == actual_bool,
		(
			boost::format( "Exception %s but exception %s" )
			% ( expected_bool ? "expected" : "not expected" )
			% ( expected_bool ? "not thrown" : "thrown" )
		).str()
	);

	if ( ! expect_exception ) {
		BOOST_CHECK_MESSAGE(
			expected_double != actual_double,
			(
				boost::format( "Exception %s but exception %s" )
				% ( expected_bool ? "expected" : "not expected" )
				% ( expected_bool ? "not thrown" : "thrown" )
			).str()
		);

		expected_double = fc;
		actual_double = lo_freq + nco_shift;
		BOOST_CHECK_MESSAGE(
			expected_double == actual_double,
			(
				boost::format( "Expected fc %f but actual fc %f" )
				% (float)expected_double
				% (float)actual_double
			).str()
		);
	}
}

// invalid frequency, expected to fail
BOOST_AUTO_TEST_CASE( test_negative_freq_1sps ) {
	common( -5, 1, true );
}

// center frequency is at 130 MHz, lower part of frequency range is at 0 Hz, upper part is at 260 MHz
BOOST_AUTO_TEST_CASE( test_130MHz_260Msps ) {
	common( 130e6, 260e6, false );
}

// center frequency is at 2.4 GHz, lower part of frequency range is at 2.4e9-130e6 Hz, upper part is 2.4e9+130e6 Hz
BOOST_AUTO_TEST_CASE( test_2400GHz_260Msps ) {
	common( 2.4e9, 260e6, false );
}

// center frequency is at 5.87 GHz, lower part of frequency range is at 6e9-2*130e6 Hz, 6e9-130e6 Hz
BOOST_AUTO_TEST_CASE( test_5870MHz_260Msps ) {
	common( 5.87e9, 260e6, false );
}
