#ifndef INCLUDED_UHD_UTILS_CMA_HPP
#define INCLUDED_UHD_UTILS_CMA_HPP

#include <queue>

namespace uhd {

	// Cumulative Moving Average
	// See https://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average

	class cma {

	public:

		cma()
		:
			avg( 0 ),
			n( 0 )
		{
		}

		virtual ~cma() {}

		virtual double update( double x ) {

			n++;
			avg += ( x - avg ) / n;

			return avg;
		}

		double get_average() {
			return avg;
		}

		size_t get_num_samples() {
			return size_t( n );
		}

	protected:

		double avg;
		double n;
	};

} // namespace uhd

#endif /* INCLUDED_UHD_UTILS_CMA_HPP */
