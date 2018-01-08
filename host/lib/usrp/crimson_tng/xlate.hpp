#ifndef XLATE_HPP_

#include <climits>
#include <complex>

/**
 * Convert a single scalar value from int16_t to double
 * @param x the int16_t value to convert
 * @return the resultant double value, normalized to 1
 */
static inline void to_double( const int16_t & s16, double & f64 ) {
	if ( s16 < 0 ) {
		f64 = (double) -s16 / (double) SHRT_MIN;
	} else {
		f64 = (double) s16 / (double) SHRT_MAX;
	}
}

/**
 * Convert a single scalar, complex, value from int16_t to double
 * @param x the complex int16_t value to convert
 * @return the resultant complex double value, with the real and imaginary components normalized to 1
 */
static inline void to_double( const std::complex<int16_t> & s16, std::complex<double> & f64 ) {
	double re;
	double im;
	to_double( s16.real(), re );
	to_double( s16.imag(), im );
	f64.real( re );
	f64.imag( im );
}

/**
 * Convert a vector of complex values from int16_t to double
 * @param x the complex int16_t values to convert
 * @return the resultant complex double values, with the real and imaginary components normalized to 1
 */
static inline void to_double( const std::vector<std::complex<int16_t>> & s16, std::vector<std::complex<double>> & f64 ) {
	f64.resize( s16.size() );
	for( size_t i = 0; i < s16.size(); i++ ) {
		to_double( s16[ i ], f64[ i ] );
	}
}

/**
 * Convert a scalar double to int16_t
 * @param f64 the double value to convert
 * @param s16 the converted int16_t
 */
static inline void to_int16( const double & f64, int16_t & s16 ) {
	double x;
	if ( f64 < 0 ) {
		x = std::round( -f64 * SHRT_MIN );
	} else {
		x = std::round( f64 * SHRT_MAX );
	}
	x = x < SHRT_MIN ? SHRT_MIN : x;
	x = x > SHRT_MAX ? SHRT_MAX : x;
	s16 = (int16_t) x;
}

/**
 * Convert a scalar, complex double to complex int16_t
 * @param f64 the complex double value to convert
 * @param s16 the converted complex int16_t
 */
static inline void to_int16( const std::complex<double> & f64, std::complex<int16_t> & s16 ) {
	int16_t re;
	int16_t im;
	to_int16( f64.real(), re );
	to_int16( f64.imag(), im );
	s16.real( re );
	s16.imag( im );
}

/**
 * Convert a vector of complex doubles to a vector of complex int16_t
 * @param f64 the complex double values to convert
 * @param s16 the converted complex int16_t values
 */
static void to_int16( const std::vector<std::complex<double>> & f64, std::vector<std::complex<int16_t>> & s16 ) {
	s16.resize( f64.size() );
	for( size_t i = 0; i < f64.size(); i++ ) {
		to_int16( f64[ i ], s16[ i ] );
	}
}

#endif

