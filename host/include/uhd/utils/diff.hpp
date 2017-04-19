#ifndef INCLUDED_UHD_UTILS_DIFF_HPP
#define INCLUDED_UHD_UTILS_DIFF_HPP

namespace uhd {

	// Finite difference (numerical differentiation)
	//
	// Note: We use the same approach that Matlab uses, which is Newton's Difference Quotient
	// or First-Order Divided Difference
	// See https://en.wikipedia.org/wiki/Numerical_differentiation#Finite_difference_formulas
	//
	// Also note, to avoid copying (i.e. to be fast), we do not operator on vectors. We only
	// store 1 previous sample + 1 previous sample time.
	class diff {

	public:

		diff()
		:
			x0( 0 ),
			y0( 0 )
		{
		}

		diff( double x0, double y0 )
		:
			x0( x0 ),
			y0( y0 )
		{
		}

		virtual ~diff() {}

		virtual double update( double x, double y ) {

			double diff;

			diff = ( y - y0 ) / ( x - x0 );
			x0 = x;
			y0 = y;

			return diff;
		}

	protected:
		double x0;
		double y0;
	};

} // namespace uhd

#endif /* INCLUDED_UHD_UTILS_SMA_HPP */
