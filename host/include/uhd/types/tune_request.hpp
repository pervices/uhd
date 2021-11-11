//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_UHD_TYPES_TUNE_REQUEST_HPP
#define INCLUDED_UHD_TYPES_TUNE_REQUEST_HPP

#include <uhd/config.hpp>
#include <uhd/types/device_addr.hpp>

namespace uhd{

    /*!
     * A tune request instructs the implementation how to tune the RF chain.
     * The policies can be used to select automatic tuning or
     * fined control over the daughterboard IF and DSP tuning.
     * Not all combinations of policies are applicable.
     * Convenience constructors are supplied for most use cases.
     *
     * See also \ref general_tuning
     */
    struct UHD_API tune_request_t{
        /*!
         * Make a new tune request for a particular center frequency.
         * Use an automatic policy for the RF and DSP frequency
         * to tune the chain as close as possible to the target frequency.
         * \param target_freq the target frequency in Hz
         */
        tune_request_t(double target_freq = 0);

        /*!
         * Make a new tune request for a particular center frequency.
         * Use a manual policy for the RF frequency,
         * and an automatic policy for the DSP frequency,
         * to tune the chain as close as possible to the target frequency.
         * \param target_freq the target frequency in Hz
         * \param lo_off the LO offset frequency in Hz
         */
        tune_request_t(double target_freq, double lo_off);

        /*!
         * Fully manual tune request which supports multiple center frequencies.
         * \param target_freq the target frequency in Hz
         * \param rf_ch_freq the target frequency of DAC channelizer
         * \param rf_dp_freq the target frequency of DAC Datapath
         * \param lo_off the LO offset frequency in Hz
         * \param dsp_freq the target frequency of FPGA DSP
         */
        tune_request_t(int band, double dsp_freq, double lo_off);
        /*!
         * Tune request for manually setting dsp nco and lo
         * \param band the target frequency in Hz
         * \param dsp_freq the target frequency of FPGA DSP
         * \param lo_off the LO offset frequency in Hz
         */

        tune_request_t(double target_freq, double rf_ch_freq, double rf_dp_freq, double lo_off, double dsp_freq);

        //! Policy options for tunable elements in the RF chain.
        enum policy_t {
            //! Do not set this argument, use current setting.
            POLICY_NONE   = int('N'),
            //! Automatically determine the argument's value.
            POLICY_AUTO   = int('A'),
            //! Use the argument's value for the setting.
            POLICY_MANUAL = int('M')
        };

        /*!
         * The target frequency of the overall chain in Hz.
         * Set this even if all policies are set to manual.
         */
        double target_freq;

        /*!
         * The policy for the RF frequency.
         * Automatic behavior: the target frequency + default LO offset.
         */
        policy_t rf_freq_policy;

        /*!
         * The RF frequency in Hz.
         * This is DAD Datapath plus DAC Channelizer NCO tuning word.
         * Set when the policy is set to manual.
         *
         * Note: We must reuse the same name for compatibility with UHD.
         *       A proper name for this variable would be rf_dp_freq.
         */
        double rf_freq;

        /*!
         * The RF frequency in Hz. This is DAD Datapath NCO tuning word.
         * Set when the policy is set to manual.
         */
        double rf_dp_freq;

        /*!
         * The RF frequency in Hz. This is DAD Channel NCO tuning word.
         * Set when the policy is set to manual.
         */
        double rf_ch_freq;

        /*!
         * The External LO frequency in Hz.
         * Set when the policy is set to manual.
         */
        double lo_freq;

        /*!
         * The policy for the DSP frequency.
         * Automatic behavior: the difference between the target and IF.
         */
        policy_t dsp_freq_policy;

        /*!
         * The DSP frequency in Hz.
         * Set when the policy is set to manual.
         *
         * Note that the meaning of the DSP frequency's sign differs between
         * TX and RX operations. The target frequency is the result of
         * `target_freq = rf_freq + sign * dsp_freq`. For TX, `sign` is
         * negative, and for RX, `sign` is positive.
         * Example: If both RF and DSP tuning policies are set to manual, and
         * `rf_freq` is set to 1 GHz, and `dsp_freq` is set to 10 MHz, the
         * actual target frequency is 990 MHz for a TX tune request, and
         * 1010 MHz for an RX tune request.
         */
        double dsp_freq;

        /*!
         * The args parameter is used to pass arbitrary key/value pairs.
         * Possible keys used by args (depends on implementation):
         *
         * - mode_n: Allows the user to tell the daughterboard tune code
         * to choose between an integer N divider or fractional N divider.
         * Default is fractional N on boards that support fractional N tuning.
         * Fractional N provides greater tuning accuracy at the expense of spurs.
         * Possible options for this key: "integer" or "fractional".
         */
        device_addr_t args;

        policy_t band_policy;
        /*
         * The policy used when selecting band
         */

        int band;
        /*
         * The numerical constants corresponding to each band are in the fw_commmon.h for each device,
         * in and enum named BAND_DICT
         * In the fw_common.h file for the device, then manual band slection has not been implemented
         */
    };

} //namespace uhd

#endif /* INCLUDED_UHD_TYPES_TUNE_REQUEST_HPP */
