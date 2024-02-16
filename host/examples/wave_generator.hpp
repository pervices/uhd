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
public:
    wave_generator(const std::string& wave_type, const double ampl, double sample_rate, double wave_freq)
    : _sample_rate(sample_rate),
    _wave_freq(wave_freq)
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

