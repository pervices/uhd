#ifndef PIDC_HPP_
#define PIDC_HPP_

#if 0
 #ifndef DEBUG_PIDC
 #define DEBUG_PIDC 1
 #endif
#endif

#define DEBUG_PIDC//remove this define when done
#ifdef DEBUG_PIDC
#include <iostream>
#include <iomanip>
#endif
#include <cmath>

#include <uhd/types/time_spec.hpp>
#include "sma.hpp"

namespace uhd {

	class pidc {

	public:

		static constexpr double DEFAULT_MAX_ERROR_FOR_DIVERGENCE = 20e-6;

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
			e( DEFAULT_MAX_ERROR_FOR_DIVERGENCE ),
			i( 0.0 ),
			// initialize the control variable to be equal to the set point, so error is initially zero
			cv( 0.0 ),
			sp( sp ),
			offset( 0.0 ),
			last_time( 0 ),
			last_status_time( 0 ),
			converged( false ),
			max_error_for_divergence( DEFAULT_MAX_ERROR_FOR_DIVERGENCE )
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

		double update_control_variable( const double sp, const double pv, double now ) {
			// XXX: @CF: Use "velocity algorithm" form?
			// https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
			// Possibly better to not use the velocity algorithm form to avoid several opportunities for numerical instability

			double then = last_time;

			double dt = now - then;
			double e_1 = e;

			if ( std::abs( dt ) < 1e-9 || dt < 0 ) {

                std::cout << "No changes to pid controller made" << std::endl;
				// when dt is incredibly small (or negative) do not perform any updates
				return cv;
			}

			std::cout << "PID controller is being updated" << std::endl;

			this->sp = sp;
			e = sp - pv;
			error_filter.update(  e  );

			// proportional
			double P = Kp * e;

			// integral
			i += e * dt;
			double I = Ki * i;

			// derivative
			// the only possible numerical instability in this format is division by dt
			double D = Kd * (e - e_1) / dt;

			is_converged( now );

			// ouput
			cv = P + I + D;

			last_time = now;

			return cv - offset;
		}

		double get_last_time() {
			return last_time;
		}

		// XXX: @CF: should only be used in the case where last time is not necessarily when the pidc constructor was called
		void set_last_time( double t ) {
			last_time = t;
		}

		double get_control_variable() {
			return cv - offset;
		}

		double get_k( k_t k ) {
			switch( k ) {
			case K_P: return Kp;
			case K_I: return Ki;
			case K_D: return Kd;
			default: return 0;
			}
		}

		void reset( const double sp, const double time ) {
			cv = sp;
			last_time = time;
			e = max_error_for_divergence;
			i = 0;
			converged = false;
		}

		bool is_converged( const double time ) {
            std::cout << "Checking if converged" << std::endl;

			double filtered_error;

			filtered_error = abs(error_filter.get_average());

            std::cout << "Filter error: " << filtered_error << std::endl;

			/*if ( filtered_error >= 1000000 ) {//commented out by Doug 2021-06-17
				if ( time - last_status_time >= 1 ) {
					print_pid_diverged();
					print_pid_status( time, cv, filtered_error );
				}
				reset( sp, time );
				return false;
			}*/

            print_pid_status( time, cv, filtered_error )//remove when done debugging

			if ( time - last_status_time >= 1 ) {
				print_pid_status( time, cv, filtered_error );
				last_status_time = time;
			}

			if ( ! converged ) {
				if ( filtered_error < max_error_for_divergence * 0.9 ) {
					converged = true;
					print_pid_status( time, cv, filtered_error );
					print_pid_converged();
				}
			} else {
				if ( filtered_error >= max_error_for_divergence ) {
					converged = false;
					print_pid_diverged();
					print_pid_status( time, cv, filtered_error );
				}
			}

			return converged;
		}

		double get_max_error_for_convergence() {
			return max_error_for_divergence;
		}
		void set_max_error_for_convergence( const double error ) {
			max_error_for_divergence = std::abs( error );
		}
		void set_error_filter_length( size_t len ) {
			double avg = error_filter.get_average();
			error_filter.set_window_size( len );
			error_filter.reset();
			error_filter.update( avg );
		}

		void set_offset( const double timeOffset ) {
			offset =  timeOffset;
		}
		double get_offset(){
			return offset;
		}
	protected:
		double Kp, Ki, Kd;

		double e; // error memory
		double i; // integral memory
		double cv; // output memory
		double sp;

		double offset; //time offset

		double last_time;
		double last_status_time;

		bool converged;
		double max_error_for_divergence;
		uhd::sma error_filter;

		static void print_pid_status( double t, double cv, double pv ) {
#ifdef DEBUG_PIDC
			std::cerr
				<< "t: " << std::fixed << std::setprecision(6) << t << ", "
				<< "cv: " << std::fixed << std::setprecision( 20 ) << cv << ", "
				<< "pv: " << std::fixed << std::setprecision( 20 ) << pv << ", "
				<< std::endl;
#else
			(void)t;
			(void)cv;
			(void)pv;
#endif
		}
		static void print_pid_converged() {
#ifdef DEBUG_PIDC
			std::cerr
				<< "PID Converged"
				<< std::endl;
#endif
		}
		static void print_pid_diverged() {
#ifdef DEBUG_PIDC
			std::cerr
				<< "PID Diverged"
				<< std::endl;
#endif
		}
		static void print_pid_reset() {
#ifdef DEBUG_PIDC
			std::cerr
				<< "PID Reset"
				<< std::endl;
#endif
		}
	};

} // namespace uhd

#include "pidc_tl.hpp"

#endif /* PIDC_HPP_ */
