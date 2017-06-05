#ifndef INCLUDED_UHD_UTILS_PIDC_HPP
#define INCLUDED_UHD_UTILS_PIDC_HPP

#include "uhd/types/time_spec.hpp"

namespace uhd {

	class pidc {

	public:

		typedef enum {
			K_P,
			K_I,
			K_D,
		} k_t;

		pidc()
		:
			pidc( 0.0, 0.0, 0.0, 0.0 )
		{
		}

		pidc( double sp, double Kp, double Ki, double Kd )
		:
			Kp( Kp ),
			Ki( Ki ),
			Kd( Kd ),
			e( 0.0 ),
			i( 0.0 ),
			// initialize the control variable to be equal to the set point, so error is initially zero
			cv( sp ),
			last_time( uhd::time_spec_t::get_system_time().get_real_secs() )
		{
			const std::string s[] = { "Kp", "Ki", "Kd" };
			const double K[] = { Kp, Ki, Kd };
			for( unsigned i = 0; i < sizeof( K ) / sizeof( K[ 0 ] ); i++ ) {
				if ( K[ i ] < 0 ) {
					throw value_error( "PID K-values are, by definition, non-negative" );
				}
			}
		}

		virtual ~pidc() {}

		double update_control_variable( const double sp, const double pv, const double now = uhd::time_spec_t::get_system_time().get_real_secs() ) {
			// XXX: @CF: Use "velocity algorithm" form?
			// https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
			// Possibly better to not use the velocity algorithm form to avoid several opportunities for numerical instability

			double then = last_time;

			double dt = now - then;
			double e_1 = e;
			e = sp - pv;

			// proportional
			double P = Kp * e;

			// integral
			i += e * dt;
			double I = Ki * i;

			// derivative
			// the only possible numerical instability in this format is division by dt
			double D = Kd * (e - e_1) / dt;

			// ouput
			cv = P + I + D;
			last_time = now;

			return cv;
		}

		double get_last_time() {
			return last_time;
		}

		// XXX: @CF: should only be used in the case where last time is not necessarily when the pidc constructor was called
		void set_last_time( double t ) {
			last_time = t;
		}

		double get_control_variable() {
			return cv;
		}

		double get_k( k_t k ) {
			switch( k ) {
			case K_P: return Kp;
			case K_I: return Ki;
			case K_D: return Kd;
			default: return 0;
			}
		}

	protected:
		double Kp, Ki, Kd;

		double e; // error memory
		double i; // integral memory
		double cv; // output memory

		double last_time;
	};

} // namespace uhd

#endif /* INCLUDED_UHD_UTILS_PIDC_HPP */
