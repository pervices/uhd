#ifndef INCLUDED_UHD_UTILS_SMA_HPP
#define INCLUDED_UHD_UTILS_SMA_HPP

#include <queue>

namespace uhd {

	// Simple, Moving Average
	// See https://en.wikipedia.org/wiki/Moving_average#Simple_moving_average

	class sma {

	public:

		sma( size_t N = 16 )
		:
			N( N ),
			sma( 0 )
		{
		}

		virtual ~sma() {}

		// NOT thread-safe (subclass, add mutex, and override for thread-safety)
		virtual double update_sma( double x ) {

			if ( X.empty() ) {
				// fill the buffer with the first sample
				// return first sample

				std::vector<double> tmp( N );
				std::fill( tmp.begin(), tmp.end(), x );

				X = std::queue( tmp );

				return x;
			}

			X.push( x );
			sma += ( X.front()  - X.back() ) / N;
			X.pop();

			return sma;
		}

	protected:

		const size_t N;
		std::queue<double> X;
		double sma;
	};

} // namespace uhd

#endif /* INCLUDED_UHD_UTILS_SMA_HPP */
