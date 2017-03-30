#ifndef INCLUDED_UHD_UTILS_SMA_HPP
#define INCLUDED_UHD_UTILS_SMA_HPP

#include <queue>

#include <uhd/utils/cma.hpp>

namespace uhd {

	// Simple, Moving Average
	// See https://en.wikipedia.org/wiki/Moving_average#Simple_moving_average

	class sma : public uhd::cma {

	public:

		sma( size_t N = 16 )
		:
			uhd::cma(),
			N( N )
		{
		}

		virtual ~sma() {}

		virtual double update( double x ) {

			X.push( x );

			if ( X.size() < N ) {

				// if number of samples is smaller than window size
				// perform a cumulative average

				return uhd::cma::update( x );

			}

			// perform the simple moving average

			avg += ( X.front()  - X.back() ) / N;

			X.pop();

			return avg;
		}

		void set_window_size( double N ) {
			this->N = N;
		}

	protected:

		// would make this const, but some of our code calls an init() function inside of a constructor,
		// and for the queue, copy-constructors (or move ctors??) are deleted.
		size_t N;
		std::queue<double> X;
	};

} // namespace uhd

#endif /* INCLUDED_UHD_UTILS_SMA_HPP */
