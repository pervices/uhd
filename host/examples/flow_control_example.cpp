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

	//uhd::pidc pidc( SP * BUF_LEN, 1.0, 0, 0 );
	uhd::pidc_tl pidc( SP * BUF_LEN, 0.2, MTU / f_s );
	uhd::flow_control fc( f_s, SP, BUF_LEN, pidc, f_s / MTU );

	for( t = 0.0 ; ; ) {

		dt = fc.get_time_until_next_send( MTU, t ).get_real_secs();
		if ( dt < 0.0 ) {
			fc.pidc.update_control_variable( fc.nominal_buffer_level , fc.get_buffer_level( t + dt ), t + dt );
		} else {
			t += dt; // i.e. sleep( dt )
		}

		fc.update( MTU, t );
	}
}

int main() {
	foo();
	return 0;
}
