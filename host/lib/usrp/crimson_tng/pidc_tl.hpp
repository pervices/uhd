#ifndef PIDC_TL_HPP_
#define PIDC_TL_HPP_

#include "pidc.hpp"

namespace uhd {

#define TL_TI( Ku, Pu ) ( 2.2 * Pu )
#define TL_TD( Ku, Pu ) ( Pu / 6.3 )

#define TL_KP( Ku, Pu ) ( Ku / 2.2 )
#define TL_KI( Ku, Pu ) ( TL_KP( Ku, Pu ) / TL_TI( Ku, Pu ) )
#define TL_KD( Ku, Pu ) ( TL_KP( Ku, Pu ) * TL_TD( Ku, Pu ) )

	/**
	 * Tyreus-Luyben Tuned PID Controller
	 *
	 * An alternative, more stable tuning than Ziegler-Nichols.
	 */
	class pidc_tl : public uhd::pidc {

	public:

		pidc_tl()
		:
			pidc_tl( 0.0, 0.0, 0.0, 0.0 )
		{
		}

		pidc_tl( double sp, double Kp, double Ki, double Kd )
		:
			uhd::pidc( sp, Kp, Ki, Kd )
		{
		}

		pidc_tl( double sp, double Ku, double Pu )
		:
			pidc_tl( sp, TL_KP( Ku, Pu ), TL_KI( Ku, Pu ), TL_KD( Ku, Pu ) )
		{
		}

		virtual ~pidc_tl() {}
	};

} // namespace uhd

#endif /* PIDC_TL_HPP_ */
