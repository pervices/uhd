#ifndef CONV_HPP_

#include <algorithm>
#include <complex>
#include <string>
#include <vector>

/**
 * Convolve vectors u and v. This macro handles any combination of real and complex convolution sums.
 *
 * When shape is "same", we first perform a "full" (linear) convolution and then remove elements the front and the
 * back of the resulting vector to match the size of the larger input vectors. This is a MATLAB convention and is
 * really only useful when the convention is to have a signal begin in the *middle* of a vector. For real-world
 * filters, use "trunc" to instead truncate the resulting vector to the size of u.
 *
 * Note, there are a multitude of ways to optimize this for throughput and memory usage. Here, we use a fairly
 * naive approach without any hardware acceleration (e.g. SIMD).
 *
 * @param u an input vector
 * @param v an input vector
 * @param w an output vector, the convolution of u and v
* @param shape The shape of w. One of "full" (default), "same" (same size but N/2 aligned), or "trunc" (same size but aligned to 0th element).
 */
#define CONV( u, v, w, shape ) \
	if ( u.size() < 1 || v.size() < 1 ) { \
		throw new std::invalid_argument( "invalid size for at least one input vector" ); \
	} \
	const size_t M = u.size(); \
	const size_t N = v.size(); \
	const size_t L = M + N - 1; \
	w.resize( L ); \
	std::fill( w.begin(), w.end(), T( 0 ) ); \
	/* for each position l in w */ \
	for( size_t l = 0, m0, m1; l < L; l++ ) { \
		/* for each overlapping position as we slide _u over _v */ \
		m0 = l < N ? 0 : l - ( N - 1 ); \
		m1 = l < M ? l : M - 1; \
		for( size_t m = m0; m <= m1; m++ ) { \
			w[ l ] += u[ m ] * v[ l - m ]; \
		} \
	} \
	size_t O = L - M; \
	if ( "same" == shape ) { \
		/* returns the central portion of the result, same size as first argument */ \
		size_t o0 = 0 == O % 2 ? O / 2 : O / 2 + 1; \
		w.erase( w.begin(), w.begin() + o0 ); \
		w.erase( w.end() - O / 2, w.end() ); \
	} \
	if ( "trunc" == shape ) { \
		/* returns the leading portion of the result, same size as first argument */ \
		w.erase( w.end() - O, w.end() ); \
	}

/**
 * Convolve two similar vectors u and v. This function handles, real * real, and complex * complex convolution sums.
 *
 * When shape is "same", we first perform a "full" (linear) convolution and then remove elements the front and the
 * back of the resulting vector to match the size of the larger input vectors. This is a MATLAB convention and is
 * really only useful when the convention is to have a signal begin in the *middle* of a vector. For real-world
 * filters, use "trunc" to instead truncate the resulting vector to the size of u.
 *
 * Note, there are a multitude of ways to optimize this for throughput and memory usage. Here, we use a fairly
 * naive approach without any hardware acceleration (e.g. SIMD).
 *
 * @param u an input vector
 * @param v an input vector
 * @param w an output vector, the convolution of u and v
 * @param shape The shape of w. One of "full" (default), "same" (same size but N/2 aligned), or "trunc" (same size but aligned to 0th element).
 */
template <class T>
inline
void conv( const std::vector<T> & u, const std::vector<T> & v, std::vector<T> & w, const std::string & shape = "full" ) {
	CONV( u, v, w, shape );
}

/**
 * Convolve two dissimilar vectors u and v. This function handles complex * real convolution sums.
 *
 * When shape is "same", we first perform a "full" (linear) convolution and then remove elements the front and the
 * back of the resulting vector to match the size of the larger input vectors. This is a MATLAB convention and is
 * really only useful when the convention is to have a signal begin in the *middle* of a vector. For real-world
 * filters, use "trunc" to instead truncate the resulting vector to the size of u.
 *
 * Note, there are a multitude of ways to optimize this for throughput and memory usage. Here, we use a fairly
 * naive approach without any hardware acceleration (e.g. SIMD).
 *
 * @param u an input vector
 * @param v an input vector
 * @param w an output vector, the convolution of u and v
 * @param shape The shape of w. One of "full" (default), "same" (same size but N/2 aligned), or "trunc" (same size but aligned to 0th element).
 */
template <class T>
inline
void conv( const std::vector<std::complex<T>> & u, const std::vector<T> & v, std::vector<std::complex<T>> & w, const std::string & shape = "full" ) {
	CONV( u, v, w, shape );
}

/**
 * Convolve two dissimilar vectors u and v. This function handles real * complex convolution sums.
 *
 * When shape is "same", we first perform a "full" (linear) convolution and then remove elements the front and the
 * back of the resulting vector to match the size of the larger input vectors. This is a MATLAB convention and is
 * really only useful when the convention is to have a signal begin in the *middle* of a vector. For real-world
 * filters, use "trunc" to instead truncate the resulting vector to the size of u.
 *
 * Note, there are a multitude of ways to optimize this for throughput and memory usage. Here, we use a fairly
 * naive approach without any hardware acceleration (e.g. SIMD).
 *
 * @param u an input vector
 * @param v an input vector
 * @param w an output vector, the convolution of u and v
 * @param shape The shape of w. One of "full" (default), "same" (same size but N/2 aligned), or "trunc" (same size but aligned to 0th element).
 */
template <class T>
inline
void conv( const std::vector<T> & u, const std::vector<std::complex<T>> & v, std::vector<std::complex<T>> & w, const std::string & shape = "full" ) {
	CONV( u, v, w, shape );
}

#endif
