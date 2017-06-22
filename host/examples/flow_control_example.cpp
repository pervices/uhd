#include <thread>

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

#define DEFAULT_FC \
	f_s, SP, BUF_LEN

template<typename T>
static bool is_close( T a, T b, T eps = T(0.0001) ) {
	return abs( a - b ) <= eps * std::max( abs( a ), abs( b ) );
}

static uhd::time_spec_t get_time_now() {
	return uhd::time_spec_t::get_system_time();
}

void print_flow_control_levels( const uhd::time_spec_t & now, std::vector<uhd::flow_control::sptr> fc )
{
	static uhd::time_spec_t last_print_time;

	const size_t twidth = 80;
	size_t gauge_len;

	if ( now < last_print_time + 1.0 ) {
		return;
	}

	last_print_time = now;

	gauge_len = twidth / fc.size();

	//std::cout << '\r';

	for( size_t i = 0; i < twidth; i++ ) {
		std::cout << '\b';
	}

	//std::cout << '\r';

	for( size_t j = 0; j < fc.size(); j++ ) {
		double pcnt = fc[ j ]->get_buffer_level_pcnt( now );
		for( size_t i = 0; i < gauge_len; i++ ) {
			if ( false ) {
			} else if ( 0 == i ) {
				std::cout << (char)( 'A' + j );
			} else if ( gauge_len - 1 == i ) {
				std::cout << '|';
			} else {
				if ( pcnt > i / (double) gauge_len ) {
					std::cout << '=';
				} else {
					std::cout << '-';
				}
			}
		}
	}

	std::cout << std::endl;
}

void async( std::vector<uhd::flow_control::sptr> &fc ) {

	struct timespec req, rem;

	uhd::time_spec_t T = 1 / (double) CRIMSON_TNG_UPDATE_PER_SEC;
	req.tv_sec = T.get_full_secs();
	req.tv_nsec = T.get_frac_secs() * 1e9;

	sleep( 1 );

	for( ;; ) {
		nanosleep( & req, & rem );
		//fc->set_buffer_level_async(  );
	}
}

void foo() {

	const size_t N = 4;

	double quarter_buffer_in_time;

	uhd::time_spec_t now, then, dt;
	struct timespec req, rem;

	std::vector<uhd::flow_control::sptr> _flow_control;
	std::vector<std::thread> _thread;

	now = get_time_now();
	for( size_t i = 0; i < N; i++ ) {
		uhd::flow_control::sptr fc =
			uhd::flow_control_nonlinear::make( DEFAULT_FC );
		fc->set_start_of_burst_time( now + 2.0 );
		_flow_control.push_back( fc );
	}

	quarter_buffer_in_time = ( (double) BUF_LEN / 4 ) / f_s;

//	std::thread th = std::thread( async,  std::ref( _flow_control ) );

	for( size_t i = 0; ; i++, i %= N ) {
		//
		// Flow Control
		//

		now = get_time_now();
		dt =
			_flow_control[ i ]->get_time_until_next_send(
				MTU,
				now
			);
		then = now + dt;
		if ( dt.get_real_secs() > 1e-3 ) {
			dt -= 30e-6;
			req.tv_sec = (time_t) dt.get_full_secs();
			req.tv_nsec = dt.get_frac_secs()*1e9;
			nanosleep( &req, &rem );
		}
		for(
			now = get_time_now();
			now < then;
			now = get_time_now()
		) {
			// nop
			__asm__ __volatile__( "" );
		}

		//
		// Update Flow Control
		//

		_flow_control[ i ]->update( MTU, now );

		//
		// Send Data
		//

		print_flow_control_levels( get_time_now(), _flow_control );
	}
}

int main() {
	foo();
	return 0;
}
