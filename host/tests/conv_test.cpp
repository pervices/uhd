#include <exception>

#include <boost/format.hpp>
#include <boost/test/unit_test.hpp>

#include "../lib/usrp/crimson_tng/conv.hpp"

template <class T>
static std::string to_string( const std::vector<T> & x ) {

	std::stringstream os;

	os << "[";
	for( size_t i = 0; i < x.size(); i++ ) {
		if ( 0 == i ) {
			os << " ";
		}
		os << x[ i ];
		if ( i < x.size() - 1 ) {
			os << ",";
		}
		os << " ";
	}
	os << "]";

	return os.str();
}

/**
 * Overload the insertion operator to pretty-print vectors.
 * @param os output stream
 * @param foo vector to stringify
 * @return output stream
 */
template <class T>
static std::ostream & operator<< ( std::ostream & os, const std::vector<T> & x )
{
	os << to_string( x );
	return os;
}

BOOST_AUTO_TEST_CASE( test_zero_length ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u, v, w;

	// both zero length
	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr != expected_exception_ptr,
		(
			boost::format( "Exception expected" )
		)
	);

	// u zero length
	v = std::vector<double>{ 0, 1, 2 };
	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr != expected_exception_ptr,
		(
			boost::format( "Exception expected" )
		)
	);

	// v zero length
	u = std::vector<double>{ 0, 1, 2 };
	v = std::vector<double>();
	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr != expected_exception_ptr,
		(
			boost::format( "Exception expected" )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_non_zero_length ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1 }, v { 2, 3 }, w{ 4, 5, 6 };

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_length_full ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1 }, v { 2, 3 }, w( 42, 0 );

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size() + v.size() - 1;
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);
}

BOOST_AUTO_TEST_CASE( test_length_same ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1 }, v { 2, 3 }, w( 42, 0 );

	try {
		conv( u, v, w, "same" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);
}

BOOST_AUTO_TEST_CASE( test_length_trunc ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1 }, v { 2, 3 }, w( 42, 0 );

	try {
		conv( u, v, w, "trunc" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);
}

///
/// conv( u, v, w, "full" )
///

BOOST_AUTO_TEST_CASE( test_results_full_real_double_M4_N4 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2, 3, 4 }, v { 5, 6, 7, 8 }, w{ 42 };

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size() + v.size() - 1;
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 5, 16, 34, 60, 61, 52, 32 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_full_real_double_M3_N3 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2, 3 }, v { 4, 5, 6 }, w{ 42 };

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size() + v.size() - 1;
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 4, 13, 28, 27, 18 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_full_real_double_M2_N4 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2 }, v { 3, 4, 5, 6 }, w{ 42 };

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size() + v.size() - 1;
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 3, 10, 13, 16, 12 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_full_real_double_M3_N4 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2, 3 }, v { 4, 5, 6, 7 }, w{ 42 };

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size() + v.size() - 1;
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 4, 13, 28, 34, 32, 21 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_full_real_double_M4_N3_dirac ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	// XXX: This uses MATLAB's convention that the signal is centered in the *middle* of the vector!
	std::vector<double> u{ 1, 2, 3, 4 }, v { 0, 1, 0 }, w{ 42 };

	try {
		conv( u, v, w );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size() + v.size() - 1;
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 0, 1, 2, 3, 4, 0 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

///
/// conv( u, v, w, "same" )
///

BOOST_AUTO_TEST_CASE( test_results_same_real_double_M4_N4 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2, 3, 4 }, v { 5, 6, 7, 8 }, w{ 42 };

	try {
		conv( u, v, w, "same" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 34, 60, 61, 52 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_same_real_double_M3_N3 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2, 3 }, v { 4, 5, 6 }, w{ 42 };

	try {
		conv( u, v, w, "same" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 13, 28, 27 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_same_real_double_M2_N4 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2 }, v { 3, 4, 5, 6 }, w{ 42 };

	try {
		conv( u, v, w, "same" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 13, 16 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_same_real_double_M3_N4 ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	std::vector<double> u{ 1, 2, 3 }, v { 4, 5, 6, 7 }, w{ 42 };

	try {
		conv( u, v, w, "same" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 28, 34, 32 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

BOOST_AUTO_TEST_CASE( test_results_same_real_double_M4_N3_dirac ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	// XXX: This uses MATLAB's convention that the signal is centered in the *middle* of the vector!
	std::vector<double> u{ 1, 2, 3, 4 }, v { 0, 1, 0 }, w{ 42 };

	try {
		conv( u, v, w, "same" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 1, 2, 3, 4 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}

///
/// conv( u, v, w, "trunc" )
///

BOOST_AUTO_TEST_CASE( test_results_trunc_real_double_M4_N3_dirac ) {

	std::exception_ptr expected_exception_ptr = nullptr;
	std::exception_ptr actual_exception_ptr = nullptr;

	// XXX: This uses MATLAB's convention that the signal is centered in the *middle* of the vector!
	std::vector<double> u{ 1, 2, 3, 4 }, v { 1, 0, 0 }, w{ 42 };

	try {
		conv( u, v, w, "trunc" );
	} catch( ... ) {
		actual_exception_ptr = std::current_exception();
	}

	BOOST_CHECK_MESSAGE(
		actual_exception_ptr == expected_exception_ptr,
		(
			boost::format( "Exception not expected" )
		)
	);

	size_t expected_size_t = u.size();
	size_t actual_size_t = w.size();

	BOOST_CHECK_MESSAGE(
		actual_size_t == expected_size_t,
		(
			boost::format( "Result has unexpected size: expected: %u actual: %u" )
			% expected_size_t
			% actual_size_t
		)
	);

	std::vector<double> expected_vector_double { 1, 2, 3, 4 };
	std::vector<double> actual_vector_double = w;
	BOOST_CHECK_MESSAGE(
		std::equal( actual_vector_double.begin(), actual_vector_double.end(), expected_vector_double.begin() ),
		(
			boost::format( "Result unexpected: expected: %s actual: %s" )
			% to_string( expected_vector_double )
			% to_string( actual_vector_double )
		)
	);
}
