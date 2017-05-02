#include <boost/test/unit_test.hpp>

#include "tune_lo_and_dsp_fixture.hpp"

#define k *1e3
#define M *1e6

static const meta_range_t FM( 87.9 M, 107.9 M, 200 k );

void check_tune( const tune_lo_and_dsp_fixture &f ) {
	double expected_double = f.req.target_freq;
	double actual_double =
		0
		+ f.res.actual_rf_freq
		+ f.res.actual_dsp_freq
	;
	BOOST_CHECK_CLOSE( expected_double, actual_double, 100 k );
}

//
// Frequency range overlap
//

bool partial_range_overlap( const meta_range_t &r1, const meta_range_t &r2 ) {
	return
		true
		&& r1.start() <= r2.start()
		&& r1.stop() > r2.start()
		&& r1.stop() <= r2.stop()
	;
}
bool full_range_overlap( const meta_range_t &r1, const meta_range_t &r2 ) {
	return
		true
		&& r1.start() > r2.start()
		&& r1.stop() <= r2.stop()
	;
}

bool ranges_overlap( const meta_range_t &r1, const meta_range_t &r2 ) {
	return
		false
		|| partial_range_overlap( r1, r2 )
		|| partial_range_overlap( r2, r1 )
		|| full_range_overlap( r1, r2 )
		|| full_range_overlap( r2, r1 )
	;
}

void check_overlap( const tune_lo_and_dsp_fixture &f, const meta_range_t & range  ) {
	double lo = f.rf_subtree->access<double>( "/freq/value" ).get();
	double bw = f.dsp_subtree->access<double>( "/rate/value" ).get();
	double nco = f.dsp_subtree->access<double>( "/freq/value" ).get();

	meta_range_t r1( lo - bw / 2, lo + bw / 2 );
	meta_range_t r2 = range;

	BOOST_CHECK( ! ranges_overlap( r1, r2 ) );
}

//
// Singular frequency overlap
//

bool range_overlaps( const meta_range_t &r, double val ) {
	return !( r.stop() < val || r.start() > val );
}

void check_overlap( const tune_lo_and_dsp_fixture &f, const double hz ) {
	double lo = f.rf_subtree->access<double>( "/freq/value" ).get();
	double bw = f.dsp_subtree->access<double>( "/rate/value" ).get();
	double nco = f.dsp_subtree->access<double>( "/freq/value" ).get();

	meta_range_t range( lo - bw / 2, lo + bw / 2 );

	BOOST_CHECK( ! range_overlaps( range, 0 ) );
}


void check_overlap_nyquist( const tune_lo_and_dsp_fixture &f ) {
	double nyquist = 0;
	switch( f.channel ) {
	case 'A':
	case 'B':
		nyquist = CRIMSON_TNG_MASTER_CLOCK_RATE / 2.0;
		break;
	case 'C':
	case 'D':
		nyquist = CRIMSON_TNG_MASTER_CLOCK_RATE / 4.0;
		break;
	}
	check_overlap( f, nyquist );
}

void check_lo_below( const tune_lo_and_dsp_fixture &f, double val ) {
	double lo = f.rf_subtree->access<double>( "/freq/value" ).get();

	BOOST_WARN_LT( lo, val );
}

void common( double fc, double bw ) {
	for( char ch = 'A'; ch <= 'D'; ch++ ) {
		for( bool tx: { true, false } ) {

			tune_lo_and_dsp_fixture f( ch, tx, fc, bw );

			f.res = tune_lo_and_dsp( f.tx ? TX_SIGN : RX_SIGN, f.dsp_subtree, f.rf_subtree, f.req );

			check_tune( f );
			check_overlap( f, 0 );
			check_overlap( f, 25 M );
			check_overlap( f, FM );
			check_overlap( f, 137 M );
			check_overlap_nyquist( f );
			check_lo_below( f, 575 M );
		}
	}
}

BOOST_AUTO_TEST_CASE( test_40MHz_BW_at_660MHz_Fc ) {
	common( 660 M, 40 M );
}
