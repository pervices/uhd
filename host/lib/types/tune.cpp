//
// Copyright 2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/types/tune_request.hpp>
#include <uhd/types/tune_result.hpp>
#include <boost/format.hpp>

using namespace uhd;

tune_request_t::tune_request_t(double target_freq):
    target_freq(target_freq),
    rf_freq_policy(POLICY_AUTO),
    rf_freq(0.0),
    rf_ch_freq(0.0),
    rf_dp_freq(0.0),
    lo_freq(0.0),
    dsp_freq_policy(POLICY_AUTO),
    dsp_freq(0.0)
{
    /* NOP */
}

tune_request_t::tune_request_t(double target_freq, double lo_off):
    target_freq(target_freq),
    rf_freq_policy(POLICY_MANUAL),
    rf_freq(target_freq + lo_off),
    rf_ch_freq(0.0),
    rf_dp_freq(0.0),
    lo_freq(0.0),
    dsp_freq_policy(POLICY_AUTO),
    dsp_freq(0.0)
{
    /* NOP */
}

//dummy is there to avoid an overload conflict with the target_freq and lo_off version
tune_request_t::tune_request_t(double target_dsp_freq, double target_lo_freq, bool dummy):
    target_freq(target_dsp_freq, target_lo_freq),
    rf_freq_policy(POLICY_MANUAL),
    rf_freq(target_freq),
    rf_ch_freq(0.0),
    rf_dp_freq(0.0),
    lo_freq(target_lo_freq),
    dsp_freq_policy(POLICY_MANUAL),
    dsp_freq(target_dsp_freq)
{
    /* NOP */
}

tune_request_t::tune_request_t(double target_freq, double rf_ch_freq , double rf_dp_freq, double lo_off, double dsp_freq ):
    target_freq(target_freq),
    rf_freq_policy(POLICY_MANUAL),
    rf_freq(rf_ch_freq + rf_dp_freq),
    rf_ch_freq(rf_ch_freq),
    rf_dp_freq(rf_dp_freq),
    lo_freq(lo_off),
    dsp_freq_policy(POLICY_MANUAL),
    dsp_freq(dsp_freq)
{
    /* NOP */
}

std::string tune_result_t::to_pp_string(void) const{
    return str(boost::format(
        "Tune Result:\n"
        "    Target RF  Freq: %f (MHz)\n"
        "    Actual RF  Freq: %f (MHz)\n"
        "    Target DSP Freq: %f (MHz)\n"
        "    Actual DSP Freq: %f (MHz)\n"
    )
        % (target_rf_freq/1e6)  % (actual_rf_freq/1e6)
        % (target_dsp_freq/1e6) % (actual_dsp_freq/1e6)
    );
}
