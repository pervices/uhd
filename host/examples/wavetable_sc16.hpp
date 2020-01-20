//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <string>
#include <cmath>
#include <complex>
#include <vector>
#include <stdexcept>

static const size_t wave_table_len = 8192;

class wave_table_class_sc16{
public:
    wave_table_class_sc16(const std::string &wave_type, const float ampl):
        _wave_table(wave_table_len)
    {
        //compute real wave table with 1.0 amplitude
        std::vector<float> real_wave_table(wave_table_len);
        if (wave_type == "CONST"){
            for (size_t i = 0; i < wave_table_len; i++)
                real_wave_table[i] = 1;
        }
        else if (wave_type == "SQUARE"){
            for (size_t i = 0; i < wave_table_len; i++)
                real_wave_table[i] = (i < wave_table_len/2)? 0 : 1;
        }
        else if (wave_type == "RAMP"){
            for (size_t i = 0; i < wave_table_len; i++)
                real_wave_table[i] = 2*i/(wave_table_len-1) - 1;
        }
        else if (wave_type == "SINE"){
            static const double tau = 2*std::acos(-1.0);
            double scale_factor = 32767.;
            for (size_t i = 0; i < wave_table_len; i++)
                real_wave_table[i] = std::sin((tau*i)/wave_table_len) * scale_factor * ampl;
        }
        else throw std::runtime_error("unknown waveform type: " + wave_type);

        // compute i and q pairs with 90% offset and scale to amplitude
        for (size_t i = 0; i < wave_table_len; i++){
            const size_t q = (i+(3*wave_table_len)/4)%wave_table_len;

            uint32_t real = int16_t(real_wave_table[i]);
            uint32_t imag = int16_t(real_wave_table[q]);

            uint32_t val = (real << 16) + imag;
            // Byte swap
            val = ((val & 0xff000000) >> 24 ) | ((val & 0x00ff0000) >> 8 ) | ((val & 0x0000ff00) << 8 ) | ((val & 0x000000ff) << 24 );
            _wave_table[i] = val;
        }
    }

    inline uint32_t operator()(const size_t index) const{
        return _wave_table[index % wave_table_len];
    }

private:
    std::vector<uint32_t> _wave_table;
};


