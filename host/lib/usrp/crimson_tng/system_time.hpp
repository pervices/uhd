#ifndef UHD_SYSTEM_TIME_HPP_
#define UHD_SYSTEM_TIME_HPP_

#include <time.h>

#include <cerrno>
#include <cstring>

#include "uhd/exception.hpp"
#include "uhd/types/time_spec.hpp"

namespace uhd {

static inline time_spec_t get_system_time() {
	timespec ts;
	if ( 0 != ::clock_gettime( CLOCK_MONOTONIC, & ts ) ) {
		throw system_error( "clock_gettime(3) failed: " + std::string( ::strerror( errno ) ) + " ( " + std::to_string( errno ) + " )" );
	}
	return time_spec_t( double( ts.tv_sec ), double( ts.tv_nsec ) / 1e9 );
}

}

#endif
