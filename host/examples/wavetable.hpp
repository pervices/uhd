//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2019-2020 Ettus Research, A National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <algorithm>
#include <cmath>
#include <complex>

#include <stdexcept>
#include <string>
#include <vector>
#include <limits>
#include <cstdint>

static const size_t wave_table_len = 65536;

// Datatype of the samples to be include (float for fc32, short for sc16...)
template<typename T = float>
class wave_table_multitype
{
public:
    wave_table_multitype(const std::string& wave_type, const float ampl)
        : _wave_table(wave_table_len, {0, 0})
    {
        T type_max;
        if(std::is_floating_point<T>::value) {
            type_max = 1;
        } else {
            type_max = std::numeric_limits<T>::max();
        }

        T wave_max =(T) (ampl * type_max);

        // Note: CONST, SQUARE, and RAMP only fill the I portion, since they are
        // amplitude-modulating signals, not phase-modulating.

        if (wave_type == "CONST") {
            // Fill with I == ampl, Q == 0
            std::fill(_wave_table.begin(), _wave_table.end(), std::complex<T>{wave_max, 0});

            _power_dbfs = static_cast<double>(20 * std::log10(ampl));

        } else if (wave_type == "SQUARE") {
            // Fill the second half of the table with ampl, first half with
            // zeros
            std::fill(_wave_table.begin() + wave_table_len / 2,
                _wave_table.end(),
                std::complex<T>{wave_max, 0});

            _power_dbfs = static_cast<double>(20 * std::log10(ampl)) - static_cast<double>(10 * std::log10(2.0));

        } else if (wave_type == "RAMP") {
            // Fill I values with ramp from -1 to 1, Q with zero
            float energy_acc = 0.0f;
            for (size_t i = 0; i < wave_table_len; i++) {
                _wave_table[i] = {(T)((2.0f * i / (wave_table_len - 1) - 1.0f) * wave_max), 0};

                energy_acc += std::norm(_wave_table[i]/wave_max);
            }
            _power_dbfs = static_cast<double>(energy_acc / wave_table_len);
            // Note: The closed-form solution to the average sum of squares of
            // the ramp is:
            // 1.0 / 3 + 2.0 / (3 * N) + 1.0 / (3 * N) + 4.0 / (6 * N^2))
            // where N == wave_table_len, but it turns out be be less code if we
            // just calculate the power on the fly.
        } else if (wave_type == "SINE") {
            static const double tau = 2 * std::acos(-1.0);
            static const std::complex<double> J(0, 1);
            // Careful: i is the loop counter, not the imaginary unit

            for (size_t i = 0; i < wave_table_len; i++) {
                // Directly generate complex sinusoid (a*e^{j 2\pi i/N}). We
                // create a single rotation. The call site will sub-sample
                // appropriately to create a sine wave of it's desired frequency
                _wave_table[i] =
                    std::complex<T>((double)wave_max * std::exp(J * static_cast<double>(tau * i / wave_table_len)));
            }
            _power_dbfs = static_cast<double>(20 * std::log10(ampl));
        } else if (wave_type == "SINE_NO_Q") {
            static const double tau = 2 * std::acos(-1.0);
            static const std::complex<double> J(0, 1);
            // Careful: i is the loop counter, not the imaginary unit

            for (size_t i = 0; i < wave_table_len; i++) {
                // Directly generate complex sinusoid (a*e^{j 2\pi i/N}). We
                // create a single rotation. The call site will sub-sample
                // appropriately to create a sine wave of it's desired frequency
                _wave_table[i] =
                    std::complex<T>((double)wave_max * std::exp(J * static_cast<double>(tau * i / wave_table_len)));
                _wave_table[i] = std::complex<T>(std::real(_wave_table[i]));
            }

            _power_dbfs = static_cast<double>(20 * std::log10(ampl));

        } else {
            throw std::runtime_error("unknown waveform type: " + wave_type);
        }
    }

    inline std::complex<T> operator()(const size_t index) const
    {
        return _wave_table[index % wave_table_len];
    }

    //! Return the signal power in dBFS
    inline double get_power() const
    {
        return _power_dbfs;
    }

private:
    std::vector<std::complex<T>> _wave_table;
    double _power_dbfs;
};

class wave_table_class : public wave_table_multitype<float> {
public:
    wave_table_class(const std::string& wave_type, const float ampl)
    : wave_table_multitype<float>(wave_type, ampl)
    {
    }
};
