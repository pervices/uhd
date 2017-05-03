#include <iostream>

#include "tune_lo_and_dsp_test_fixture.hpp"

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
	BOOST_CHECK_CLOSE( expected_double, actual_double, 1 k );
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

void common( tune_lo_and_dsp_test_fixture & f ) {
	const bool warn = true;

	f.res = tune_lo_and_dsp( f.tx ? TX_SIGN : RX_SIGN, f.dsp_subtree, f.rf_subtree, f.req );

	// mandatory (tune request must give use the correct overal result
	check_tune( f );

	// mandatory
	check_overlap( f, 0 );
	//check_overlap( f, 137 M ); // ??
	//check_overlap_nyquist( f );

	// preferred
	check_overlap( f, 25 M, warn );
	check_overlap( f, FM, warn );
	//check_lo_below( f, 575 M, warn );

}

void check_band( tune_lo_and_dsp_test_fixture & f, int expected_band ) {
	int actual_band;

	actual_band = f.rf_subtree->access<int>( "/freq/band" ).get();

	BOOST_CHECK_EQUAL( expected_band, actual_band );
}

void check_lo( tune_lo_and_dsp_test_fixture & f, double expected_lo ) {
	double actual_lo;

	actual_lo = f.rf_subtree->access<double>( "/freq/value" ).get();

	BOOST_CHECK_CLOSE( expected_lo, actual_lo, 10 k );
}

void check_nco( tune_lo_and_dsp_test_fixture & f, double expected_nco ) {
	double actual_nco;

	actual_nco = f.dsp_subtree->access<double>( "/freq/value" ).get();

	BOOST_CHECK_CLOSE( expected_nco, actual_nco, 10 k );
}

/*
[-]--------------------[+]-----------------------+-------------------------+-----------[///////////+////////]---------------+------------[\\\\\\\\\\\+\\\\\\\\\\\\\\>
 |                      |                                                              |                    |                            |                      |    f (MHz)
 0                      25                                                           87.9                  107.9                        137                    162.5
DC                 LO fundamental                                                               FM                                   ADC Cutoff            Max Samp Rate
 */

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
	check_lo( f, 625 M );
	check_nco( f, 35 M );
)

TEST_( 25, 4005,

	//  -----------------------------------------
	//  |                                       |
	// 3997.5            4005                4017.5 -->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 3950 M );
	check_nco( f, -55 M );
)

TEST_( 1, 4005,

	//  -----------------------------------------
	//  |                                       |
	// 4004              4005                  4006 -->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4000 M );
	check_nco( f, -5 M );
)

TEST_( 8, 4005,

	//  -----------------------------------------
	//  |                                       |
	// 4001            4005                   4009-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4000 M );
	check_nco( f, -5 M );
)

TEST_( 10, 4015,

	//  -----------------------------------------
	//  |                                       |
	// 4010            4015                   4020-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 4000 M );
	check_nco( f, -15 M );
)

TEST_( 40, 4015,

	//  -----------------------------------------
	//  |                                       |
	// 3995            4015                   4035-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 3950 M );
	check_nco( f, -65 M );
)

TEST_( 5, 130,

	//  -----------------------------------------
	//  |                                       |
	// 125              130                    135-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 0 );
	check_lo( f, 0 M );
	check_nco( f, -130 M );
)

TEST_( 20, 130,

	//  -----------------------------------------
	//  |                                       |
	// 120             130                     140-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 0 );
	check_lo( f, 150 M );
	check_nco( f, 20 M );
)

TEST_( 30, 90,

	//  -----------------------------------------
	//  |                                       |
	// 75               90                     105-->

	tune_lo_and_dsp_test_fixture f( fc, bw );
	common( f );

	check_band( f, 1 );
	check_lo( f, 0 M );
	check_nco( f, -90 M );
)
