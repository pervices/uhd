#include <iostream>

#include "tune_lo_and_dsp_test_fixture.hpp"

#include "uhd/usrp/multi_usrp.hpp"

#define k *1e3
#define M *1e6

static const meta_range_t FM( 87.9 M, 107.9 M, 200 k );

//static istream & operator>>( istream & is, const meta_range_t & range ) {
//	return is;
//}

void check_tune( const tune_lo_and_dsp_test_fixture &f ) {
	double expected_double = f.req.target_freq;
	double actual_double =
		0
		+ f.res.actual_rf_freq
		+ f.res.actual_dsp_freq
	;
	BOOST_CHECK_CLOSE( expected_double, actual_double, 1 );
}

bool range_overlaps( const meta_range_t &r, double val ) {
	bool ret =
	!(
		(
			// range is entirely less or equal to val
			r.start() <= val && r.stop() <= val
		) || (
			// range is entirely greater than or equal to val
			r.start() >= val && r.stop() >= val
		)
	);
	return ret;
}

bool range_overlaps( const meta_range_t &r1, const meta_range_t &r2 ) {
	return !(
		(
			// range is entirely less or equal to val
			r1.start() <= r2.start() && r1.stop() <= r2.start()
		) || (
			// range is entirely greater than or equal to val
			r1.start() >= r2.stop() && r1.stop() >= r2.stop()
		)
	);
}

void check_overlap( const tune_lo_and_dsp_test_fixture &f, const double hz, bool warn = false ) {
	double lo = f.rf_subtree->access<double>( "/freq/value" ).get();
	double bw = f.dsp_subtree->access<double>( "/rate/value" ).get();
	double nco = f.dsp_subtree->access<double>( "/freq/value" ).get();

	double fc = lo - ( f.tx ? TX_SIGN : RX_SIGN ) * nco;

	meta_range_t range( fc - bw / 2, fc + bw / 2 );

	bool overlap = range_overlaps( range, hz );
	if ( warn ) {
		BOOST_WARN_MESSAGE(
			! overlap,
			"frequency range " << range.to_pp_string() << " overlaps frequency " << hz
		);
	} else {
		BOOST_CHECK_MESSAGE(
			! overlap,
			"frequency range " << range.to_pp_string() << " overlaps frequency " << hz
		);
	}
}

void check_overlap( const tune_lo_and_dsp_test_fixture &f, const meta_range_t & range, bool warn = false ) {
	double lo = f.rf_subtree->access<double>( "/freq/value" ).get();
	double bw = f.dsp_subtree->access<double>( "/rate/value" ).get();
	double nco = f.dsp_subtree->access<double>( "/freq/value" ).get();

	double fc = lo - ( f.tx ? TX_SIGN : RX_SIGN ) * nco;

	meta_range_t r1( fc - bw / 2, fc + bw / 2 );
	meta_range_t r2 = range;

	bool overlap = range_overlaps( r1, r2 );

	if ( warn ) {
		BOOST_WARN_MESSAGE(
			! overlap,
			"range " << r1.to_pp_string() << " overlaps range " << r2.to_pp_string()
		);
	} else {
		BOOST_CHECK_MESSAGE(
			! overlap,
			"range " << r1.to_pp_string() << " overlaps range " << r2.to_pp_string()
		);
	}
}

void check_lo_below( tune_lo_and_dsp_test_fixture & f, double expected_lo, bool warn = false ) {

	double actual_lo = f.rf_subtree->access<double>( "/freq/value" ).get();

	if ( 0 != actual_lo ) {
		if ( warn ) {
			BOOST_WARN_MESSAGE(
				actual_lo >= expected_lo,
				( boost::format( "lo (%f) is below %f" ) % actual_lo % expected_lo ).str()
			);
		} else {
			BOOST_CHECK_MESSAGE(
				actual_lo >= expected_lo,
				( boost::format( "lo (%f) is below %f" ) % actual_lo % expected_lo ).str()
			);
		}
	}
}

void check_lo_modulus( tune_lo_and_dsp_test_fixture & f ) {

	// XXX: @CF: this should be part of the test fixture
	const double lo_step = 25e6;

	double lo = f.rf_subtree->access<double>( "/freq/value" ).get();

	double expected_double = 0;
	double actual_double = fmod( lo, lo_step );

	BOOST_CHECK_MESSAGE(
		expected_double == actual_double,
		(
			boost::format( "LO (%f MHz) is not a multiple of LO step size (%f MHz)" )
			% ( lo / 1e6 )
			% ( lo_step / 1e6 )
		).str()
	);
}

void common( tune_lo_and_dsp_test_fixture & f ) {
	const bool warn = true;

	f.res = uhd::usrp::tune_lo_and_dsp( f.tx ? TX_SIGN : RX_SIGN, f.dsp_subtree, f.rf_subtree, f.req );

	// mandatory (tune request must give use the correct overal result
	check_tune( f );

	// mandatory
	check_overlap( f, 0 );
	check_lo_modulus( f );

	// preferred
	check_overlap( f, 25 M, warn );
	check_overlap( f, FM, warn );
	check_overlap( f, 137 M, warn );

	check_lo_below( f, 150 M, warn );
	check_lo_below( f, 575 M, warn );
}

void check_band( tune_lo_and_dsp_test_fixture & f, int expected_band ) {
	int actual_band;

	actual_band = f.rf_subtree->access<int>( "/freq/band" ).get();

	BOOST_CHECK_EQUAL( expected_band, actual_band );
}

void check_lo( tune_lo_and_dsp_test_fixture & f, double expected_lo ) {
	double actual_lo;

	actual_lo = f.rf_subtree->access<double>( "/freq/value" ).get();

	BOOST_CHECK_CLOSE( expected_lo, actual_lo, 1 );
}

void check_nco( tune_lo_and_dsp_test_fixture & f, double expected_nco ) {
	double actual_nco;

	actual_nco = f.dsp_subtree->access<double>( "/freq/value" ).get();

	BOOST_CHECK_CLOSE( expected_nco, actual_nco, 1 );
}

// TX tests

#define TEST_( _b, _f, _x ) \
	BOOST_AUTO_TEST_CASE( test_ ## _b ## MHz_BW_at_ ## _f ## MHz_Fc ) { \
		double fc = _f M; \
		double bw = _b M; \
		_x \
	}

TEST_( 40, 660,

	//  -----------------------------------------
	//  |                                       |
	// 630                660                  690 -->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 675 M );
	check_nco( f, -15 M );
)

TEST_( 25, 4005,

	//  -----------------------------------------
	//  |                                       |
	// 3997.5            4005                4017.5 -->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4025 M );
	check_nco( f, -20 M );
)

TEST_( 1, 4005,

	//  -----------------------------------------
	//  |                                       |
	// 4004              4005                  4006 -->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4000 M );
	check_nco( f, -20 M );
)

TEST_( 8, 4005,

	//  -----------------------------------------
	//  |                                       |
	// 4001            4005                   4009-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4025 M );
	check_nco( f, -20 M );
)

TEST_( 10, 4015,

	//  -----------------------------------------
	//  |                                       |
	// 4010            4015                   4020-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4025 M );
	check_nco( f, -10 M );
)

TEST_( 40, 4015,

	//  -----------------------------------------
	//  |                                       |
	// 3995            4015                   4035-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4025 M );
	check_nco( f, -10 M );
)

TEST_( 5, 130,

	//  -----------------------------------------
	//  |                                       |
	// 127.5             130                   132.5-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 0 );
	check_lo( f, 0 M );
	check_nco( f, 130 M );
)

TEST_( 20, 130,

	//  -----------------------------------------
	//  |                                       |
	// 120             130                     140-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 150 M );
	check_nco( f, -20 M );
)

TEST_( 30, 90,

	//  -----------------------------------------
	//  |                                       |
	// 75               90                     105-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 0 );
	check_lo( f, 0 M );
	check_nco( f, 90 M );
)

// this test exposed something wonkey about the nco choice
TEST_( 1, 300,

	//  -----------------------------------------
	//  |                                       |
	// 299.5               300                  300.5-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 325 M );
	check_nco( f, -25 M );
)

//
// Channel C tx
//

TEST_( 30, 85,

	//  -----------------------------------------
	//  |                                       |
	// 70               85                     100-->

	tune_lo_and_dsp_test_fixture f( fc, bw, true, 'C' );
	common( f );

	check_band( f, 1 );
	check_lo( f, 100 M );
	check_nco( f, -15 M );
)

//
// High BW Tests
//

TEST_( 130, 85,

	//  -----------------------------------------
	//  |                                       |
	//  0               65                     130-->

	tune_lo_and_dsp_test_fixture f( fc, bw, true );
	common( f );

	check_band( f, 1 );
	check_lo( f, 100 M );
	check_nco( f, -15 M );
)

TEST_( 260, 130,

	//  -----------------------------------------
	//  |                                       |
	//  0               130                     260-->

	tune_lo_and_dsp_test_fixture f( fc, bw, true );
	common( f );

	check_band( f, 1 );
	check_lo( f, 125 M );
	check_nco( f, 5 M );
)

BOOST_AUTO_TEST_CASE( test_260_MHz_BW_at_135_MHz_Fc ) {
	double fc = 135 M;
	double bw = 260 M;

	//  -----------------------------------------
	//  |                                       |
	//  5               135                     265-->

	std::exception_ptr p = nullptr;

	tune_lo_and_dsp_test_fixture f( fc, bw, true, (char)'C' );

	try {

		common( f );

		check_band( f, 1 );
		check_lo( f, 125 M );
		check_nco( f, 5 M );

	} catch( ... ) {
		p = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		nullptr != p,
		"expected exception to be thrown because channel C cannot achieve " << ( f.bw / (1e6) ) << " MSps"
	);
}

//
// Rx tests
//

/*
BOOST_AUTO_TEST_CASE( test_high_band_nco_symmetry ) {

	bool tx = true;

	double wanted_lo = 2450e6;
	double wanted_nco = 3e6;

	double fc = wanted_lo + wanted_nco;
	double bw = 1e6;

	tune_lo_and_dsp_test_fixture f_tx( fc, bw, tx );
	common( f_tx );
	check_band( f_tx, 1 );
	check_lo( f_tx, wanted_lo );
	check_nco( f_tx, wanted_nco );

	tune_lo_and_dsp_test_fixture f_rx( fc, bw, !tx );
	common( f_rx );
	check_band( f_rx, 1 );
	check_lo( f_rx, wanted_lo );
	// for high band, there is no sign-reversal of the NCO shift
	check_nco( f_rx, wanted_nco );
}

BOOST_AUTO_TEST_CASE( test_low_band_nco_antisymmetry ) {

	bool tx = true;

	double wanted_lo = 0;
	double wanted_nco = 30e6;

	double fc = wanted_lo + wanted_nco;
	double bw = 1e6;

	tune_lo_and_dsp_test_fixture f_tx( fc, bw, tx );
	common( f_tx );
	check_band( f_tx, 0 );
	check_lo( f_tx, wanted_lo );
	check_nco( f_tx, wanted_nco );

	tune_lo_and_dsp_test_fixture f_rx( fc, bw, !tx );
	common( f_rx );
	check_band( f_rx, 0 );
	check_lo( f_rx, wanted_lo );
	// for low band, the sign of the NCO shift indicates tx vs rx
	check_nco( f_rx, -wanted_nco );
}
*/
