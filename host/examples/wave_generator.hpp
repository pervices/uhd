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
#include <uhd/exception.hpp>
#include <fstream>

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

    // The frequency to a apply an adjustment to improve output strength consistency
    // _frequency_brackets contains the frequency in fraction between 0 and + nyquist limit
    std::vector<double> _frequency_brackets;
    // _amplitude_multiplier contains the value to multiply the amplitude with linear interpolation used between them
    std::vector<double> _amplitude_multiplier;
    // Indicates whether or not a calibration file was provided
    bool _calibration_enabled;

    double _power_dbfs = 0;
public:
    /**
     * @param wave_type The waveform to generate
     * @param ampl The amplitude of the wave
     * @param sample_rate The sample rate
     * @param wave_freq The frequency of the wave for most waves. The comb spacing for COMB waves. Ignored for constant wave
     * @param ampl_calibration_path Path to a calibration file to improve linearity in comb waves
     */
    wave_generator(const std::string& wave_type, const double ampl, double sample_rate, double wave_freq, std::string ampl_calibration_path = "")
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

        _calibration_enabled = ampl_calibration_path != "";

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
            if(_calibration_enabled) {
                parse_config(ampl_calibration_path);
            }

            _constituent_waves.emplace_back("SINE", ampl, _sample_rate, 0);
            _normalization_factor += 1;

            for(size_t n = 1; n <= num_positive_frequencies; n++) {
                // Amplitude adjusted such that every consituent wave has the same theoretical energy
                double adjusted_ampl = std::sqrt( std::pow(ampl, 2) * 1 / n);

                // The wave frequency of this tooth of the comb
                double frequency = _wave_freq * n;
                // Which fraction from the center to the positive nyquist limit the frequency is. Used for adjusting amplitude for linearity
                double bandwidth_fraction = frequency / (_sample_rate/2);

                size_t num_frequency_brackets = _frequency_brackets.size() - 1;

                if(_calibration_enabled) {
                    // Find which adjustment bracket this frequency belongs to
                    size_t bracket;
                    for(bracket = 0; bracket < num_frequency_brackets; bracket++) {
                        if(bandwidth_fraction < _frequency_brackets[bracket + 1]) {
                            break;
                        }
                    }
                    if(bracket >= num_frequency_brackets) {
                        // Error if the request frequency is not in the table. This should be impossible
                        throw std::runtime_error("Requested frequency not in adjustment table");
                    }
                    // Linearly interpolate how much to adjust the amplitude within the band provided
                    double x = (bandwidth_fraction - _frequency_brackets[bracket]) / (_frequency_brackets[bracket + 1] - _frequency_brackets[bracket]);
                    double y = x * (_amplitude_multiplier[bracket + 1] - _amplitude_multiplier[bracket]) + _amplitude_multiplier[bracket];

                    // Adjust the amplitude according to the config file provided to improve linearity across the input
                    adjusted_ampl *= y;
                }

                _constituent_waves.emplace_back("SINE", adjusted_ampl, _sample_rate, frequency);

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
            // Wave freq of 0 behaves like CONST
            if(_wave_freq == 0) {
                return 1;
            }

            // The exact number of samples per cycle of the wave
            double period = _sample_rate / _wave_freq;

            // The fundamental period (in samples) is the numerator of period expressed as a
            // fraction in lowest terms. Find it via a continued fraction expansion of period,
            // which generates successively better rational approximations (convergents).
            // If the true next convergent's numerator would exceed the 1e9 cap, consider the best
            // semiconvergent (mediant) that still fits. A semiconvergent with a small partial
            // quotient can be a *worse* approximation than the previous full convergent, so the two
            // are compared directly (by how close candidate_lut_length*wave_freq/sample_rate lands to an integer number
            // of cycles) rather than assuming the larger numerator always wins.
            constexpr int64_t max_fundamental_period = 1000000000;

            // Distance from candidate_lut_length*wave_freq/sample_rate (i.e. candidate_lut_length/period) to the nearest integer number of
            // cycles: 0 means no discontinuity at the lookup table wrap point, 0.5 is the worst case
            auto phase_error = [period](int64_t candidate_lut_length) {
                double cycles = (double) candidate_lut_length / period;
                double whole;
                double frac = std::fabs(std::modf(cycles, &whole));
                return std::min(frac, 1.0 - frac);
            };

            double remainder = period;
            // The two most recently accepted convergents, expressed as numerator/denominator pairs.
            // Seeded with the conventional 0th "convergent" 1/0 (a placeholder meaning "no cycles yet")
            // so the very first iteration is handled by the same logic as every later one.
            int64_t prev_convergent_numerator = 1, prev_prev_convergent_numerator = 0;
            int64_t prev_convergent_denominator = 0, prev_prev_convergent_denominator = 1;
            // Safe fallback if not even a single cycle fits under the cap (e.g. wave_freq > sample_rate)
            int64_t fundamental_period = 1;

            // Limits the number of iterations to avoid an infinite loop due to floating point rounding
            for(size_t iteration = 0; iteration < 64; iteration++) {
                double whole;
                double frac = std::modf(remainder, &whole);
                // The integer part of this step of the continued fraction expansion of period
                int64_t partial_quotient = (int64_t) whole;

                // Largest partial quotient <= partial_quotient for which
                // partial_quotient*prev_convergent_numerator + prev_prev_convergent_numerator still
                // fits under the cap. Using the full partial_quotient gives the true next convergent;
                // using less gives the best semiconvergent achievable under the cap. Computed via
                // division before any multiplication so the multiplication below can never overflow,
                // even when partial_quotient itself is huge.
                int64_t largest_partial_quotient_under_cap = partial_quotient;
                if(prev_convergent_numerator > 0) {
                    int64_t max_partial_quotient_for_cap =
                        (max_fundamental_period - prev_prev_convergent_numerator) / prev_convergent_numerator;
                    if(max_partial_quotient_for_cap < largest_partial_quotient_under_cap) {
                        largest_partial_quotient_under_cap = max_partial_quotient_for_cap;
                    }
                }

                if(largest_partial_quotient_under_cap < 1) {
                    // Not even one more cycle fits under the cap; keep the last accepted convergent
                    break;
                }

                int64_t candidate_numerator =
                    largest_partial_quotient_under_cap * prev_convergent_numerator + prev_prev_convergent_numerator;
                int64_t candidate_denominator =
                    largest_partial_quotient_under_cap * prev_convergent_denominator + prev_prev_convergent_denominator;

                if(largest_partial_quotient_under_cap < partial_quotient) {
                    // True next convergent would have overflowed the cap. Only switch to the
                    // semiconvergent if it's actually a better approximation than the previous
                    // full convergent; otherwise keep fundamental_period as it already is.
                    double err_semi = phase_error(candidate_numerator);
                    double err_prev = (prev_convergent_denominator == 0)
                        ? std::numeric_limits<double>::infinity()
                        : phase_error(prev_convergent_numerator);
                    if(err_semi < err_prev) {
                        fundamental_period = candidate_numerator;
                    }
                    break;
                }

                fundamental_period = candidate_numerator;

                // frac is essentially 0: period is (to floating point precision) exactly
                // candidate_numerator/candidate_denominator
                if(frac < 0.000000001) {
                    break;
                }

                prev_prev_convergent_numerator = prev_convergent_numerator;
                prev_convergent_numerator = candidate_numerator;
                prev_prev_convergent_denominator = prev_convergent_denominator;
                prev_convergent_denominator = candidate_denominator;

                remainder = 1 / frac;
            }

            std::cout << "fundamental_period: " << fundamental_period << std::endl;

            return (size_t) fundamental_period;
        }
    }

    //! Return the signal power in dBFS
    inline double get_power() const
    {
        // TODO: implement
        throw uhd::not_implemented_error("Getting wave power not implemented yet");
        return _power_dbfs;
    }

private:
    inline size_t calc_num_positive_frequencies() {
        return (size_t) std::ceil((0.5 * _sample_rate/_wave_freq) - 1);
    }

    void parse_config(std::string config_path) {
        // Format:
        // # At the start of a line indicates a comment
        // The first non comment line is a list of frequency fractions
        // The second non comment line is a list of their respective gains
        std::ifstream file(config_path);
        if(!file.is_open()) {
            std::cout << "Error when opening file\n";
        }

        std::string line;
        do {
            if(file.eof()) {
                throw std::runtime_error("Reached end of file before finding frequency location");
            }
            std::getline(file, line);

            // Loop until the first non blank line that doesn't start with #
        } while(line[0] == '#' || line == "");

        // Replace , with space for easier parsing
        std::replace( line.begin(), line.end(), ',', ' ');
        std::string trimmed_line = line;

        while(trimmed_line.size() > 0) {
            size_t pos = 0;
            double value;
            try {
                value = std::stod(trimmed_line, &pos);
            } catch (std::invalid_argument &e) {
                // Provides an error message to the user there is an error while parsing then rethrows the error
                UHD_LOG_ERROR("WAVE_GENERATOR", "Error parsing double for frequency fraction from line: \"" + trimmed_line + "\"");
                throw;
            }
            // No more parsable characters
            if(pos == 0) {
                break;
            }

            // Add the bracket size to the list
            _frequency_brackets.emplace_back(value);

            trimmed_line.erase(0, pos);
        }

        do {
            if(file.eof()) {
                throw std::runtime_error("Reached end of file before finding amplitude");
            }
            std::getline(file, line);

            // Loop until the first non blank line that doesn't start with #
        } while(line[0] == '#' || line == "");

        std::replace( line.begin(), line.end(), ',', ' ');
        trimmed_line = line;

        while(trimmed_line.size() > 0) {
            size_t pos = 0;
            double value;
            try {
                value = std::stod(trimmed_line, &pos);
            } catch (std::invalid_argument &e) {
                // Provides an error message to the user there is an error while parsing then rethrows the error
                UHD_LOG_ERROR("WAVE_GENERATOR", "Error parsing double for frequency fraction from line: \"" + trimmed_line + "\"");
                throw;
            }
            // No more parsable characters
            if(pos == 0) {
                break;
            }

            // Add the bracket size to the list
            _amplitude_multiplier.emplace_back(value);

            trimmed_line.erase(0, pos);
        }

        if(_frequency_brackets.size() != _amplitude_multiplier.size()) {
            throw std::runtime_error("Mismatch between the size of the calibration frequency and multiplier list");
        }

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
        // It takes a double to allow this function signature to be the same as the other functions that take angles
        size_t index = (size_t) findex;

        // Used to sum all every constituent wave
        std::complex<double> sum(0, 0);

        // Add the first wave (0Hz
        sum += self->_constituent_waves[0](index) / self->_normalization_factor;

        // For every non 0 frequency
        for(size_t i = 1; i < self->_constituent_waves.size(); i++) {
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

