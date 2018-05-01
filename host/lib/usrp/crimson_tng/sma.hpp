#ifndef INCLUDED_UHD_UTILS_SMA_HPP
#define INCLUDED_UHD_UTILS_SMA_HPP

#include <cstddef>

#include <deque>

namespace uhd {

	// Simple, Moving Average
	// See https://en.wikipedia.org/wiki/Moving_average#Simple_moving_average

	class sma {

	public:

		sma( size_t N = 16 )
		:
			N( N ),
			avg( 0 )
		{
		}

		virtual ~sma() {}

		virtual double update( double x ) {

			X.push_back( x );
			if ( X.size() > N ) {
				X.pop_front();
			}

			avg = 0.0;

			for( const auto& v: X ) {
				avg += v;
			}

			avg /= X.size();

			return avg;
		}

		size_t get_num_samples() {
			return X.size();
		}

		void set_window_size( double N ) {
			this->N = N;
		}

		double get_average() {
			return avg;
		}

		void reset() {
			avg = 0;
			X.clear();
		}

	protected:

		// would make this const, but some of our code calls an init() function inside of a constructor,
		// and for the queue, copy-constructors (or move ctors??) are deleted.
		size_t N;
		std::deque<double> X;
		double avg;
	};

} // namespace uhd

#endif /* INCLUDED_UHD_UTILS_SMA_HPP */
