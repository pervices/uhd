#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"
#include "../lib/usrp/crimson_tng/flow_control.hpp"

#define BYTES_PER_SAMPLE 4

#define f_s_PID  CRIMSON_TNG_UPDATE_PER_SEC
#define f_s      ( CRIMSON_TNG_MASTER_CLOCK_RATE / 6 )
#define BUF_LEN  CRIMSON_TNG_BUFF_SIZE
#define SP       0.8
#define MTU      ( CRIMSON_TNG_MAX_MTU / BYTES_PER_SAMPLE )

#define DEFAULT_FC \
	f_s,           \
	f_s / MTU,     \
	SP,            \
	BUF_LEN

template<typename T>
static bool is_close( T a, T b, T eps = T(0.0001) ) {
	return abs( a - b ) <= eps * std::max( abs( a ), abs( b ) );
}

void foo() {

	size_t i;
	uhd::time_spec_t t;
	uhd::time_spec_t dt;

	uhd::flow_control fc( DEFAULT_FC );

	//const size_t max_iterations = ceil( fc.nominal_sample_rate / fc.pid_sample_rate );
	const size_t max_iterations = 3 * f_s / MTU;

	uhd::time_spec_t t0 = uhd::time_spec_t::get_system_time();

	for(
		t = t0,
			i = 0,
			dt = uhd::time_spec_t( 0, 0 );
		i <= max_iterations;
		i++
	) {
		std::cout
			<< std::setprecision( 10 ) << ( t - t0 ).get_real_secs()
			<< ", "
			<< std::setprecision( 10 ) << fc.get_buffer_level_pcnt()
			<< std::endl;

		dt = fc.get_time_until_next_send( MTU, t );
		t += dt; // i.e. sleep( dt )
		fc.update( MTU, t );
	}
}

int main() {
	foo();
	return 0;
}
