#ifndef PIDC_HPP_
#define PIDC_HPP_

#if 0
 #ifndef DEBUG_PIDC
 #define DEBUG_PIDC 1
 #endif
#endif

#ifdef DEBUG_PIDC
#include <iostream>
#include <iomanip>
#endif
#include <cmath>

#include <uhd/types/time_spec.hpp>
#include <uhdlib/utils/sma.hpp>

namespace uhd {

	class pidc {

	public:

        // Threshold for considering times as synced for filtered error
		static constexpr double DEFAULT_MAX_ERROR_FOR_DIVERGENCE = 20e-6;
        // Threshold for considering times as desynced given a single value exceeding this error
        static constexpr double INSTANTENOUS_MAX_ERROR_FOR_DIVERGENCE = 1;
        // Threshold for reseting clock sync instead of trying to continue
        static constexpr uint64_t RESET_THRESHOLD = 1000000;

		typedef enum {
			K_P,
			K_I,
			K_D,
		} k_t;

		pidc()
		:
			pidc( 0.0, 0.0, 0.0, 0.0, 0.0 )
		{
		}

		pidc( double sp, double Kp, double Ki, double Kd, double derivative_filter_frequency )
		:
			Kp( Kp ),
			Ki( Ki ),
			Kd( Kd ),
			e( DEFAULT_MAX_ERROR_FOR_DIVERGENCE ),
			i( 0.0 ),
			// initialize the control variable to be equal to the set point, so error is initially zero
			cv( 0.0 ),
			sp( sp ),
            DERIVATE_MIN_FREQUENCY(derivative_filter_frequency),
			offset( 0.0 ),
			last_time( 0.0 ),
			last_status_time( 0.0 ),
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

		double update_control_variable( const double sp, const double pv, const uhd::time_spec_t &now ) {
			// XXX: @CF: Use "velocity algorithm" form?
			// https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
			// Possibly better to not use the velocity algorithm form to avoid several opportunities for numerical instability

			double dt = (now - last_time).get_real_secs();
			double e_1 = e;

			if ( std::abs( dt ) < 1e-9 || dt < 0 ) {
				// when dt is incredibly small (or negative) do not perform any updates
				return cv;
			}

			this->sp = sp;
			e = sp - pv;

            if(e > INSTANTENOUS_MAX_ERROR_FOR_DIVERGENCE) {
                converged = false;
            }
			error_filter.update(  e  );

			// proportional
			double P = Kp * e;

			// integral
			i += e * dt;
			double I = Ki * i;

			// derivative
            // Noise can cause the derivative component to go haywire, to mitigate this a low pass filter is apllied
            double alpha = 1 - exp(-DERIVATE_MIN_FREQUENCY*dt*2*M_PI);
            double derivative = (e - e_1) / dt;
            // f(n) = (1-a) * f(n-1) + a * f(n)
            double filtered_derivative = (( 1 - alpha ) * previous_filtered_derivated) + (alpha * derivative);
			double D = Kd * filtered_derivative;
            previous_filtered_derivated = filtered_derivative;

			// ouput
			cv = P + I + D;

			last_time = now;

			return cv - offset;
		}

		uhd::time_spec_t get_last_time() {
			return last_time;
		}

		// XXX: @CF: should only be used in the case where last time is not necessarily when the pidc constructor was called
		void set_last_time( const uhd::time_spec_t &t ) {
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

		void reset( const uhd::time_spec_t &time, const double offset ) {
            i = 0;
            cv = 0;
            this->offset = offset;
			last_time = time;
			e = max_error_for_divergence;
			i = 0;
			converged = false;
		}

		// Reset offset is the new offset to use if time diff exceeds the error limit
		// time: current time on host system
		// reset_advised: flag indicating that clocks are diverged to the point where it is is better to reset clock sync than continue
		bool is_converged( const uhd::time_spec_t &time, bool *reset_advised ) {

			double filtered_error;

			filtered_error = abs(error_filter.get_average());

            if ( filtered_error >= RESET_THRESHOLD ) {
                if ( (time - last_status_time).get_real_secs() >= 1 ) {
                    print_pid_diverged();
                    print_pid_status( time, cv, filtered_error );
                }
                *reset_advised = true;
                return false;
            }

			if ( (time - last_status_time).get_real_secs() >= 1 ) {
				print_pid_status( time, cv, filtered_error );
				last_status_time = time;
			}

			// Updates whether or not it is converged (the * 0.9 is to apply hysteresis)
            if ( filtered_error < max_error_for_divergence * 0.9 ) {
                converged = true;
                print_pid_status( time, cv, filtered_error );
                print_pid_converged();
            }
            if ( filtered_error >= max_error_for_divergence ) {
                converged = false;
                print_pid_diverged();
                print_pid_status( time, cv, filtered_error );
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
	private:
		double Kp, Ki, Kd;

		double e; // error memory
		double i; // integral memory
		double cv; // output memory
		double sp; // setpoint
        double previous_filtered_derivated = 0; // derivative memory
        double DERIVATE_MIN_FREQUENCY; // Cutoff frequency of the derivative gain

		double offset; //time offset

		uhd::time_spec_t last_time;
		uhd::time_spec_t last_status_time;

		bool converged;
		double max_error_for_divergence;
		uhd::sma error_filter;

		static void print_pid_status( uhd::time_spec_t t, double cv, double pv ) {
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
