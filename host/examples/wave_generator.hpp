//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2019-2020 Ettus Research, A National Instruments Brand
// Copyright 2023 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//


#include <cmath>
#include <complex>

#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <math.h>
#include <numeric>


// TODO remove this once debugging is done
#include <iostream>

// Datatype of the samples to be include (float for fc32, short for sc16...)
template<typename T = float>
class wave_generator
{
private:
    static constexpr std::complex<double> J = std::complex<double>(0,1);
    const double _sample_rate;
    const double _wave_freq;

    T _wave_max;
    std::complex<T> (*get_function)(double, T);

    // TODO: make wave type enum
    std::string _wave_type;

    // Vector to contain the waves that are combined to make more complex waves
    std::vector<wave_generator<double>> constituent_waves;
public:
    /**
     * @param wave_type The waveform to generate
     * @param ampl The amplitude of the wave
     * @param sample_rate The sample rate
     * @param wave_freq The frequency of the wave for most waves. The comb spacing for COMB waves. Ignored for constant wave
     */
    wave_generator(const std::string& wave_type, const double ampl, double sample_rate, double wave_freq)
    : _sample_rate(sample_rate),
    _wave_freq(wave_freq),
    _wave_type(wave_type)
    {
        T type_max;
        if(std::is_floating_point<T>::value) {
            type_max = 1;
        } else {
            type_max = std::numeric_limits<T>::max();
        }

        _wave_max = (T) (ampl * type_max);

        // Note: CONST, SQUARE, and RAMP only fill the I portion, since they are
        // amplitude-modulating signals, not phase-modulating.

        // Wave freq of 0 is equivalent to CONST, regardless of anything else
        if (wave_type == "CONST" || wave_freq == 0) {
            get_function = &get_const;
        } else if (wave_type == "SQUARE") {
            get_function = &get_square;
        } else if (wave_type == "RAMP") {
            get_function = &get_ramp;
        } else if (wave_type == "SINE") {
            get_function = &get_sine;
        } else if (wave_type == "SINE_NO_Q") {
            get_function = &get_sine_no_q;
        } else if (wave_type == "COMB") {
            // TODO: implement get for comb wave
            get_function = &get_sine;
        } else {
            throw std::runtime_error("unknown waveform type: " + wave_type);
        }
    }

    /**
     *\param index Sample number
     */
    inline std::complex<T> operator()(const size_t index) const
    {
        double revolutions = index * _wave_freq / _sample_rate;
        double whole_revoltuions;
        double angle = 2* M_PI * std::modf(revolutions, &whole_revoltuions);
        return get_function(angle, _wave_max);
    }

    // Calculates the fundamental period of the sampled wave
    // Using a lookup table that is as long as the fundamental period prevent discontinuities when it loops around
    // The fundamental period of the sampled wave is different from the fundamental period of the theoretical continuous wave
    size_t get_fundamental_period() {
        if(_wave_type == "COMB") {
            // TODO: clean up
            size_t num_positive_frequencies = (size_t) std::ceil((0.5 * _sample_rate/_wave_freq) - 1);

            // Calculate all the positive frequencies in the output (negatives can be skipped because their periods are the same)
            std::vector<double> frequencies(num_positive_frequencies);
            for(size_t n = 0; n < num_positive_frequencies; n++) {
                frequencies[n] = ( n + 1 ) * _wave_freq;
            }
            std::cout << "Lowest freq for fundamental: " <<  frequencies.front() << std::endl;
            std::cout << "Highest freq for fundamental: " <<  frequencies.back() << std::endl;

            // Calculate the period in samples
            std::vector<double> period(num_positive_frequencies);
            for(size_t n = 0; n < num_positive_frequencies; n++) {
                period[n] = _sample_rate / frequencies[n];
            }

            // Calculate how many periods are required of each wave a required for a continuous lookup table
            std::vector<size_t> num_samples_for_continuous(num_positive_frequencies);
            for(size_t n = 0; n < num_positive_frequencies; n++) {
                double full_period;
                double frac_period = std::modf(period[n], &full_period);
                if(frac_period > 0.000000001) {
                    num_samples_for_continuous[n] = (size_t) std::round( (1 / frac_period) * period[n] );
                } else {
                    num_samples_for_continuous[n] = (size_t) std::round( period[n] );
                }
            }

            // Find the fundamental which is the lcm of the number of samples for a contiuous signal for each individual wave
            size_t fundamental_period = 1;
            for(size_t n = 0; n < num_positive_frequencies; n++) {
                fundamental_period = std::lcm(fundamental_period, num_samples_for_continuous[n]);
            }

            // TODO: properly handle long fundamental period waves
            // Maximum lookup table size
            // Limited to avoid absurd startup times and RAM requirements
            const size_t MAX_LUT_SIZE = 100000;
            if(fundamental_period > MAX_LUT_SIZE) {
                std::cout << "fundamental period of comb wave to long. Limiting it to " + std::to_string(MAX_LUT_SIZE) + " samples\n";
                fundamental_period = MAX_LUT_SIZE;
            }

            std::cout << "fundamental_period: " << fundamental_period << std::endl;

            return fundamental_period;
        } else if (_wave_type == "CONST") {
            // Const only has 1 value so it's fundamental period is 1
            return 1;
        } else {
            double period;
            if(_wave_freq != 0) {
                period = _sample_rate/_wave_freq;
            } else {
                period = 0;
            }
            double full_period;
            double frac_period = std::modf(period, &full_period);
            // Length of the period of the sampled signal, to take into account mismatch between period and sample rate
            size_t fundamental_period;

            // If there is no fractional part of the period we can use the period as the super period
            // Also if the fractional part is very close to 0 treat it as close enough
            if(frac_period < 0.000000001) {
                fundamental_period = period;
            } else {
                double extra_cycles;
                if(frac_period < 0.5) {
                    extra_cycles = 1.0/frac_period;
                } else {
                    extra_cycles = 1.0/(1.0-frac_period);
                }

                fundamental_period = (size_t) ::round(period * extra_cycles);
            }
            return fundamental_period;
        }
    }

private:
    static std::complex<T>get_const(const double angle, const T max) {
        (void) angle;
        return std::complex<T>(max, 0);
    }

    static std::complex<T>get_square(const double angle, const T max) {
        if(angle < M_PI) {
            return std::complex<T>(0, 0);
        } else {
            return std::complex<T>(max);
        }
    }

    static std::complex<T>get_ramp(const double angle, const T max) {
        return {(T)(((2.0 * angle / (2.0 * M_PI)) - 1.0) * max), 0};
    }

    static std::complex<T>get_sine(const double angle, const T max) {
        return std::complex<T>((double)max * std::exp(J * angle));
    }

    static std::complex<T>get_sine_no_q(const double angle, const T max) {
        auto result = std::complex<T>((double)max * std::exp(J * angle));
        result.imag(0);
        return result;
    }
};

