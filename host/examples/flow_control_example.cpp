#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"
#include "../lib/usrp/crimson_tng/flow_control.hpp"

#define BYTES_PER_SAMPLE 4

#define f_s      ( CRIMSON_TNG_MASTER_CLOCK_RATE / 6 )
#define BUF_LEN  CRIMSON_TNG_BUFF_SIZE
#define SP       0.8
#define MTU      ( CRIMSON_TNG_MAX_MTU / BYTES_PER_SAMPLE )

template<typename T>
static bool is_close( T a, T b, T eps = T(0.0001) ) {
	return abs( a - b ) <= eps * std::max( abs( a ), abs( b ) );
}

void foo() {

	double t;
	double dt;

	uhd::flow_control::sptr fc = uhd::flow_control_nonlinear::make( f_s, SP, BUF_LEN );

	for( t = 0.0 ; ; ) {

		dt = fc->get_time_until_next_send( MTU, t ).get_real_secs();
		if ( dt > 0.0 ) {
			t += dt; // i.e. sleep( dt )
		}

		fc->update( MTU, t );
	}
}

int main() {
	foo();
	return 0;
}
