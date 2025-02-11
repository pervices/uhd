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
#include <uhd/utils/log.hpp>

// Datatype of the samples to be include (float for fc32, short for sc16...)
template<typename T = float>
class wave_generator
{
private:
    static constexpr std::complex<double> J = std::complex<double>(0,1);
    const double _sample_rate;
    const double _wave_freq;

    T _type_max;
    T _wave_max;
    std::complex<T> (*get_function)(wave_generator*, double);

    std::string _wave_type;

    // Vector to contain the waves that are combined to make more complex waves
    std::vector<wave_generator<double>> _constituent_waves;
    // Value used to normal the amplitude of composite wave
    double _normalization_factor = 0;
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
        if(std::is_floating_point<T>::value) {
            _type_max = 1;
        } else {
            _type_max = std::numeric_limits<T>::max();
        }

        _wave_max = (T) (ampl * _type_max);

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
            size_t num_positive_frequencies = calc_num_positive_frequencies();

            _constituent_waves.emplace_back("SINE", ampl, _sample_rate, 0);
            _normalization_factor += 1;

            for(size_t n = 1; n <= num_positive_frequencies; n++) {
                // Amplitude adjusted such that every consituent wave has the same energy
                double adjusted_ampl = std::sqrt( std::pow(ampl, 2) * 1 / n);

                _constituent_waves.emplace_back("SINE", adjusted_ampl, _sample_rate, _wave_freq * n);

                // Normalization factor is counted twice to account for the positive and negative sinewave at the frequency
                _normalization_factor += 2 * (adjusted_ampl / ampl);
            }

            get_function = &get_comb;
        } else {
            throw std::runtime_error("unknown waveform type: " + wave_type);
        }
    }

    /**
     *\param index Sample number
     */
    inline std::complex<T> operator()(const size_t index)
    {
        if(_wave_type == "COMB") {
            return get_function(this, index);
        } else {
            double revolutions = index * _wave_freq / _sample_rate;
            double whole_revoltuions;
            double angle = 2* M_PI * std::modf(revolutions, &whole_revoltuions);
            return get_function(this, angle);
        }
    }

    // Calculates the fundamental period of the sampled wave
    // Using a lookup table that is as long as the fundamental period prevent discontinuities when it loops around
    // The fundamental period of the sampled wave is different from the fundamental period of the theoretical continuous wave
    size_t get_fundamental_period() {
        if(_wave_type == "COMB") {
            if(_wave_freq == 0) {
                throw std::invalid_argument("Comb space of 0 requested");
            }
            size_t num_positive_frequencies = calc_num_positive_frequencies();

            if(num_positive_frequencies == 0) {
                UHD_LOG_WARNING("WAVE_GENERATOR", "The requested comb does not contain non 0 frequencies. Are your comb spacing and sample rate correct?")
            }

            // Calculate all the positive frequencies in the output (negatives can be skipped because their periods are the same)
            std::vector<double> frequencies(num_positive_frequencies);
            for(size_t n = 0; n < num_positive_frequencies; n++) {
                frequencies[n] = ( n + 1 ) * _wave_freq;
            }

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

            return fundamental_period;
        } else if (_wave_type == "CONST") {
            // Const only has 1 value so it's fundamental period is 1
            return 1;
        } else {
            double period;
            if(_wave_freq != 0) {
                period = _sample_rate/_wave_freq;
            } else {
                period = 1;
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
    inline size_t calc_num_positive_frequencies() {
        return (size_t) std::ceil((0.5 * _sample_rate/_wave_freq) - 1);
    }

    static std::complex<T>get_const(wave_generator<T> *self, const double angle) {
        (void) angle;
        return std::complex<T>(self->_wave_max, 0);
    }

    static std::complex<T>get_square(wave_generator<T> *self, const double angle) {
        if(angle < M_PI) {
            return std::complex<T>(0, 0);
        } else {
            return std::complex<T>(self->_wave_max);
        }
    }

    static std::complex<T>get_ramp(wave_generator<T> *self, const double angle) {
        return {(T)(((2.0 * angle / (2.0 * M_PI)) - 1.0) * self->_wave_max), 0};
    }

    static std::complex<T>get_sine(wave_generator<T> *self, const double angle) {
        return std::complex<T>((double)self->_wave_max * std::exp(J * angle));
    }

    static std::complex<T>get_sine_no_q(wave_generator<T> *self, const double angle) {
        auto result = std::complex<T>((double)self->_wave_max * std::exp(J * angle));
        result.imag(0);
        return result;
    }

    static std::complex<T>get_comb(wave_generator<T> *self, const double findex) {
        // Unlike other get functions comb takes the sample number
        // It takes a double to allow this function signature to be the same as the otehr functions that take angles
        size_t index = (size_t) findex;

        // Used to sum all every constituent wave
        std::complex<double> sum(0, 0);

        // Add the first wave (0Hz
        sum += self->_constituent_waves[0](index) / self->_normalization_factor;

        // For every non 0 frequency
        for(size_t i = 0; i < self->_constituent_waves.size(); i++) {
            // Add the positive frequency
            std::complex<double> positive_sample = self->_constituent_waves[i](index);
            sum += positive_sample / self->_normalization_factor;

            // Add the negative frequency (I and Q swapped)
            std::complex<double> negative_sample(std::imag(positive_sample), std::real(positive_sample));
            sum += negative_sample / self->_normalization_factor;
        }

        // Convert from range of std::complex<double> (range -1 to 1) to the disired type
        return sum * (double) self->_type_max;
    }
};

