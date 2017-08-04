//
// Copyright 2014 Per Vices Corporation
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <iostream>
#include <cmath>
#include <list>
#include <numeric>
#include <vector>

#include <uhd/property_tree.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp/multi_crimson_tng.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/gain_group.hpp>
#include <uhd/usrp/dboard_id.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/convert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>

#include "crimson_tng/crimson_tng_fw_common.h"
#include "crimson_tng/crimson_tng_impl.hpp"

#define CRIMSON_MASTER_CLOCK_RATE	322265625
#define CRIMSON_RX_CHANNELS 4
#define CRIMSON_TX_CHANNELS 4

using namespace uhd;
using namespace uhd::usrp;

static const double TX_SIGN = -1;
static const double RX_SIGN = 1;

static const double DSP_NCO_SHIFT_HZ = 25e6;

/***********************************************************************
 * Helper Functions
 **********************************************************************/
// TODO Need to check if: the sample rate, channels enabled, and bytes per I/Q sample,
// will overflow, or under flow the data stream
bool _check_tng_link_rate(const stream_args_t &args) {
	// insert logic to calculate bytes_per_sample

	// insert logic to compare link_max_rate > 0 && sample_rate * bytes_per_sample
	// <> link_max_rate

	// assume no under/overflow
	return true;
}

bool is_high_band( const meta_range_t &dsp_range, const double freq, double bw ) {
	return freq + bw / 2.0 >= dsp_range.stop();
}

// no longer used. at one point was used for deciding whether or not to use DSP for frequency shift
static bool is_low_sample_rate( const double dsp_rate ) {
	return dsp_rate < ( CRIMSON_MASTER_CLOCK_RATE / 9 );
}

// return true if b is a (not necessarily strict) subset of a
bool range_contains( const meta_range_t & a, const meta_range_t & b ) {
	return b.start() >= a.start() && b.stop() <= a.stop();
}

double choose_dsp_nco_shift( double target_freq, double sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree ) {

	/*
	 * Scenario 1) Channels A and B
	 *
	 * We want a shift such that the full bandwidth fits inside of one of the
	 * dashed regions.
	 *
	 * Our margin around each sensitive area is 1 MHz on either side.
	 *
	 * In order of increasing bandwidth & minimal interference, our
	 * preferences are
	 *
	 * Region A
	 * Region B
	 * Region F
	 * Region G
	 * Region H
	 *
	 * Region A is preferred because it exhibits the least attenuation. B is
	 * preferred over C for that reason (and because it has a more bandwidth
	 * than C). F is the next largest band and is preferred over E because
	 * it avoids the LO fundamental, but it contains FM. G is the next-to-last
	 * preference because it includes the LO and FM but has a very large
	 * bandwidth. Finally, H is the catch-all. It suffers at high frequencies
	 * due to the ADC filterbank, but includes the entirety of the spectrum.
	 *
	[--]-------------------[+]-----------------------+-------------------------+-----------[///////////+////////]---------------+------------[\\\\\\\\\\\+\\\\\\\\\\\\\\>
	 | |                    |                                                              |                    |                            |                      |    f (MHz)
	 0 2                    25                                                           87.9                  107.9                        137                    162.5
	DC                 LO fundamental                                                               FM                                   ADC Cutoff            Max Samp Rate
           A (21 MHz)                            B (60.9 MHz)                                                             C (27.1 Mhz)                 D (26.5 MHz)
                           E = A + B (includes LO)                                                                     F = B + C (includes FM)
					                                           G = A + B + C
					                                           H = A + B + C + D
	 */
	static const std::vector<freq_range_t> AB_regions {
		freq_range_t( 3e6, 24e6 ), // A
		freq_range_t( 26e6, 86.9e6 ), // B
		freq_range_t( 26e6, 136e6 ), // F = B + C
		freq_range_t( 3e6, 136e6 ), // G = A + B + C
		freq_range_t( 3e6, 162.5e6 ), // H = A + B + C + D (Catch All)
	};
	/*
	 * Scenario 2) Channels C and D
	 *
	 * Channels C & D only provide 1/2 the bandwidth of A & B due to silicon
	 * errata. This should be corrected in subsequent hardware revisions of
	 * Crimson.
	 *
	 * In order of increasing bandwidth & minimal interference, our
	 * preferences are
	 *
	 * Region A
	 * Region B
	 * Region C
	[--]-------------------[+]-----------------------+-------------------------+---->
	 | |                    |                                                  |    f (MHz)
	 0 2                    25                                               81.25
	DC                 LO fundamental                                      Max Samp Rate
           A (21 MHz)                            B (55.25 MHz)
                           C = A + B (includes LO)
	 */
	static const std::vector<freq_range_t> CD_regions {
		freq_range_t( 3e6, 24e6 ), // A
		freq_range_t( 26e6, 81.25e6 ), // B
		freq_range_t( 3e6, 81.25e6 ), // C = A + B (Catch All)
	};
	// XXX: @CF: TODO: Dynamically construct data structure upon init when KB #3926 is addressed

	static const double lo_step = 25e6;

	const meta_range_t dsp_range = dsp_subtree->access<meta_range_t>( "/freq/range" ).get();
	const char channel = ( dsp_range.stop() - dsp_range.start() ) > 81.25e6 ? 'A' : 'C';
	const double bw = dsp_subtree->access<double>("/rate/value").get();
	const std::vector<freq_range_t> & regions =
		( 'A' == channel || 'B' == channel )
		? AB_regions
		: CD_regions
	;
	const int K = (int) floor( ( dsp_range.stop() - dsp_range.start() ) / lo_step );

	for( int k = 0; k <= K; k++ ) {
		for( double sign: { +1, -1 } ) {

			double candidate_lo = target_freq;
			if ( sign > 0 ) {
				// If sign > 0 we set the LO sequentially higher multiples of LO STEP
				// above the target frequency
				candidate_lo += lo_step - fmod( target_freq, lo_step );
			} else {
				// If sign < 0 we set the LO sequentially lower multiples of LO STEP
				// above the target frequency
				candidate_lo -= fmod( target_freq, lo_step );
			}
			candidate_lo += k * sign * lo_step;

			const double candidate_nco = target_freq - candidate_lo;
			const double bb_ft = target_freq - candidate_lo + candidate_nco;
			const meta_range_t candidate_range( bb_ft - bw / 2, bb_ft + bw / 2 );

			for( const freq_range_t & _range: regions ) {
				if ( range_contains( _range, candidate_range ) ) {
					return candidate_nco;
				}
			}
		}
	}

	// Under normal operating parameters, this should never happen because
	// the last-choice _range in each of AB_regions and CD_regions is
	// a catch-all for the entire DSP bandwidth. Hitting this scenario is
	// equivalent to saying that the LO is incapable of up / down mixing
	// the RF signal into the baseband domain.
	throw runtime_error(
		(
			boost::format( "No suitable baseband region found: target_freq: %f, bw: %f, dsp_range: %s" )
			% target_freq
			% bw
			% dsp_range.to_pp_string()
		).str()
	);
}

// See multi_usrp.cpp::tune_xx_subdev_and_dsp()
tune_result_t tune_lo_and_dsp( const double xx_sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, const tune_request_t &tune_request ) {

	enum {
		LOW_BAND,
		HIGH_BAND,
	};

	freq_range_t dsp_range = dsp_subtree->access<meta_range_t>("freq/range").get();
	freq_range_t rf_range = rf_fe_subtree->access<meta_range_t>("freq/range").get();
	freq_range_t adc_range( dsp_range.start(), 137e6, 0.0001 );
	freq_range_t & min_range = dsp_range.stop() < adc_range.stop() ? dsp_range : adc_range;

	double clipped_requested_freq = rf_range.clip( tune_request.target_freq );
	double bw = dsp_subtree->access<double>( "/rate/value" ).get();

	int band = is_high_band( min_range, clipped_requested_freq, bw ) ? HIGH_BAND : LOW_BAND;

	//------------------------------------------------------------------
	//-- set the RF frequency depending upon the policy
	//------------------------------------------------------------------
	double target_rf_freq = 0.0;
	double dsp_nco_shift = 0;

	// kb #3689, for phase coherency, we must set the DAC NCO to 0
	if ( TX_SIGN == xx_sign ) {
		rf_fe_subtree->access<double>("nco").set( 0.0 );
	}

	rf_fe_subtree->access<int>( "freq/band" ).set( band );

	switch (tune_request.rf_freq_policy){
		case tune_request_t::POLICY_AUTO:
			switch( band ) {
			case LOW_BAND:
				// in low band, we only use the DSP to tune
				target_rf_freq = 0;
				break;
			case HIGH_BAND:
				dsp_nco_shift = choose_dsp_nco_shift( clipped_requested_freq, xx_sign, dsp_subtree, rf_fe_subtree );
				// in high band, we use the LO for most of the shift, and use the DSP for the difference
				target_rf_freq = rf_range.clip( clipped_requested_freq - dsp_nco_shift );
				break;
			}
		break;

		case tune_request_t::POLICY_MANUAL:
			target_rf_freq = rf_range.clip( tune_request.rf_freq );
			break;

		case tune_request_t::POLICY_NONE:
			break; //does not set
	}

	//------------------------------------------------------------------
	//-- Tune the RF frontend
	//------------------------------------------------------------------
	rf_fe_subtree->access<double>("freq/value").set( target_rf_freq );
	const double actual_rf_freq = rf_fe_subtree->access<double>("freq/value").get();

	//------------------------------------------------------------------
	//-- Set the DSP frequency depending upon the DSP frequency policy.
	//------------------------------------------------------------------
	double target_dsp_freq = 0.0;
	switch (tune_request.dsp_freq_policy) {
		case tune_request_t::POLICY_AUTO:
			target_dsp_freq = actual_rf_freq - clipped_requested_freq;

			//invert the sign on the dsp freq for transmit (spinning up vs down)
			target_dsp_freq *= xx_sign;

			break;

		case tune_request_t::POLICY_MANUAL:
			target_dsp_freq = tune_request.dsp_freq;
			break;

		case tune_request_t::POLICY_NONE:
			break; //does not set
	}

	//------------------------------------------------------------------
	//-- Tune the DSP
	//------------------------------------------------------------------
	dsp_subtree->access<double>("freq/value").set(target_dsp_freq);
	const double actual_dsp_freq = dsp_subtree->access<double>("freq/value").get();

	//------------------------------------------------------------------
	//-- Load and return the tune result
	//------------------------------------------------------------------
	tune_result_t tune_result;
	tune_result.clipped_rf_freq = clipped_requested_freq;
	tune_result.target_rf_freq = target_rf_freq;
	tune_result.actual_rf_freq = actual_rf_freq;
	tune_result.target_dsp_freq = target_dsp_freq;
	tune_result.actual_dsp_freq = actual_dsp_freq;
	return tune_result;
}

//do_tune_freq_results_message(tune_request, result, get_tx_freq(chan), "TX");
static void do_tune_freq_results_message( tune_request_t &req, tune_result_t &res, double freq, std::string rx_or_tx ) {
/*
	std::string results_string;

	// XXX: @CF: We should really change these messages..

	// results of tuning RF LO
	if (res.target_rf_freq != req.target_freq) {
		boost::format rf_lo_message(
			"  The RF LO does not support the requested %s frequency:\n"
			"	Requested RF LO Frequency: %f MHz\n"
			"	RF LO Result: %f MHz\n");
		rf_lo_message % rx_or_tx % (req.target_freq / 1e6) % (res.actual_rf_freq / 1e6);
		results_string += rf_lo_message.str();
	} else {
		boost::format rf_lo_message(
			"  The RF LO supports the requested %s frequency:\n"
			"	Requested RF LO Frequency: %f MHz\n"
			"	RF LO Result: %f MHz\n");
		rf_lo_message % rx_or_tx % (req.target_freq / 1e6) % (res.actual_rf_freq / 1e6);
		results_string += rf_lo_message.str();
	}

	// results of tuning DSP
	if (res.target_dsp_freq != req.dsp_freq) {
		boost::format dsp_message(
			"  The DSP does not support the requested %s frequency:\n"
			"	Requested DSP Frequency: %f MHz\n"
			"	DSP Result: %f MHz\n");
		dsp_message % rx_or_tx % (req.dsp_freq / 1e6) % (res.actual_dsp_freq / 1e6);
		results_string += dsp_message.str();
	} else {
		boost::format dsp_message(
			"  The DSP supports the requested %s frequency:\n"
			"	Requested DSP Frequency: %f MHz\n"
			"	DSP Result: %f MHz\n");
		dsp_message % rx_or_tx % (req.target_freq / 1e6) % (res.actual_dsp_freq / 1e6);
		results_string += dsp_message.str();
	}
	UHD_MSG( status ) << results_string;
*/
}

/***********************************************************************
 * Multi Crimson Implementation
 **********************************************************************/
multi_crimson_tng::multi_crimson_tng(const device_addr_t &addr) {
    // this make will invoke the correct inherited crimson device class
    _dev  = device::make(addr, device::CRIMSON_TNG);
    crimson_tng_impl::sptr dev_impl = boost::static_pointer_cast<crimson_tng_impl>( _dev );
    _tree = _dev  -> get_tree();
}

multi_crimson_tng::~multi_crimson_tng(void) {
    // do nothing
}

device::sptr multi_crimson_tng::get_device(void){
    return _dev;
}

// ID = unique ID set for the device
// NAME = Name of the board (digital board will be crimson)
// SERIAL = manufacturer serial no.
// FW VERSION = verilog version, if NA, will be same as SW version
// HW VERSION = PCB version
// SW VERSION = software version

dict<std::string, std::string> multi_crimson_tng::get_usrp_rx_info(size_t chan) {
    dict<std::string, std::string> crimson_info;

    crimson_info["mboard_id"]       = _tree->access<std::string>(mb_root(0) / "id").get();
    crimson_info["mboard_name"]     = _tree->access<std::string>(mb_root(0) / "name").get();
    crimson_info["mboard_serial"]   = _tree->access<std::string>(mb_root(0) / "serial").get();

    crimson_info["clock_id"]        = _tree->access<std::string>(mb_root(0) / "time" / "id").get();
    crimson_info["clock_name"]      = _tree->access<std::string>(mb_root(0) / "time" / "name").get();
    crimson_info["clock_serial"]    = _tree->access<std::string>(mb_root(0) / "time" / "serial").get();

    crimson_info["rx_id"]           = _tree->access<std::string>(mb_root(0) / "rx" / "id").get();
    crimson_info["rx_subdev_name"]  = _tree->access<std::string>(mb_root(0) / "rx" / "name").get();
    crimson_info["rx_subdev_spec"]  = _tree->access<std::string>(mb_root(0) / "rx" / "spec").get();
    crimson_info["rx_serial"]       = _tree->access<std::string>(mb_root(0) / "rx" / "serial").get();
    //crimson_info["rx_antenna"]      = _tree->access<std::string>(mb_root(0) / "rx" / "antenna").get();

    return crimson_info;
}

dict<std::string, std::string> multi_crimson_tng::get_usrp_tx_info(size_t chan) {
    dict<std::string, std::string> crimson_info;

    crimson_info["mboard_id"]       = _tree->access<std::string>(mb_root(0) / "id").get();
    crimson_info["mboard_name"]     = _tree->access<std::string>(mb_root(0) / "name").get();
    crimson_info["mboard_serial"]   = _tree->access<std::string>(mb_root(0) / "serial").get();

    crimson_info["clock_id"]        = _tree->access<std::string>(mb_root(0) / "time" / "id").get();
    crimson_info["clock_name"]      = _tree->access<std::string>(mb_root(0) / "time" / "name").get();
    crimson_info["clock_serial"]    = _tree->access<std::string>(mb_root(0) / "time" / "serial").get();

    crimson_info["tx_id"]           = _tree->access<std::string>(mb_root(0) / "tx" / "id").get();
    crimson_info["tx_subdev_name"]  = _tree->access<std::string>(mb_root(0) / "tx" / "name").get();
    crimson_info["tx_subdev_spec"]  = _tree->access<std::string>(mb_root(0) / "tx" / "spec").get();
    crimson_info["tx_serial"]       = _tree->access<std::string>(mb_root(0) / "tx" / "serial").get();
    //crimson_info["tx_antenna"]      = _tree->access<std::string>(mb_root(0) / "tx" / "antenna").get();

    return crimson_info;
}

/*******************************************************************
 * Mboard methods
 ******************************************************************/
// Master clock rate is fixed at 322.265625 MHz
void multi_crimson_tng::set_master_clock_rate(double rate, size_t mboard){
    return;
}

double multi_crimson_tng::get_master_clock_rate(size_t mboard){
    return _tree->access<double>(mb_root(0) / "tick_rate").get();
}

std::string multi_crimson_tng::get_pp_string(void){
    std::string buff = str(boost::format(
        "%s Crimson:\n"
        "  Device: %s\n"
    )
        % ((get_num_mboards() > 1)? "Multi" : "Single")
        % (_tree->access<std::string>("/name").get())
    );
    for (size_t m = 0; m < get_num_mboards(); m++){
        buff += str(boost::format(
            "  Mboard %d: %s\n"
        ) % m
            % (_tree->access<std::string>(mb_root(m) / "name").get())
        );
    }

    //----------- rx side of life ----------------------------------
    for (size_t m = 0, chan = 0; m < get_num_mboards(); m++){
        for (; chan < (m + 1)*get_rx_subdev_spec(m).size(); chan++){
            buff += str(boost::format(
                "  RX Channel: %u\n"
                "    RX DSP: %s\n"
                "    RX Dboard: %s\n"
                "    RX Subdev: %s\n"
            ) % chan
                % rx_dsp_root(chan).leaf()
                % rx_rf_fe_root(chan).branch_path().branch_path().leaf()
                % (_tree->access<std::string>(rx_rf_fe_root(chan) / "name").get())
            );
        }
    }

    //----------- tx side of life ----------------------------------
    for (size_t m = 0, chan = 0; m < get_num_mboards(); m++){
        for (; chan < (m + 1)*get_tx_subdev_spec(m).size(); chan++){
            buff += str(boost::format(
                "  TX Channel: %u\n"
                "    TX DSP: %s\n"
                "    TX Dboard: %s\n"
                "    TX Subdev: %s\n"
            ) % chan
                % tx_dsp_root(chan).leaf()
                % tx_rf_fe_root(chan).branch_path().branch_path().leaf()
                % (_tree->access<std::string>(tx_rf_fe_root(chan) / "name").get())
            );
        }
    }
    return buff;
}

// Get the Digital board name
std::string multi_crimson_tng::get_mboard_name(size_t mboard){
    return _tree->access<std::string>(mb_root(0) / "name").get();
}

static bool printed_get_time_now;

// Get the current time on Crimson
time_spec_t multi_crimson_tng::get_time_now(size_t mboard){
	crimson_tng_impl::sptr dev = boost::static_pointer_cast<crimson_tng_impl>( _dev );
	double diff = NULL == dev.get() ? 0 : dev.get()->time_diff_get();
	if ( ! printed_get_time_now ) {
		if ( NULL != dev.get() ) {
			std::cout << "time diff is " << diff << std::endl;
			printed_get_time_now = true;
		}
	}
	return time_spec_t::get_system_time() + diff;
}

// Get the time of the last PPS (pulse per second)
time_spec_t multi_crimson_tng::get_time_last_pps(size_t mboard){
    return _tree->access<time_spec_t>(mb_root(0) / "time/pps").get();
}

// Set the current time on Crimson
void multi_crimson_tng::set_time_now(const time_spec_t &time_spec, size_t mboard){
    _tree->access<time_spec_t>(mb_root(0) / "time/now").set(time_spec);
    return;
}

// Set the time for the next PPS (pulse per second)
void multi_crimson_tng::set_time_next_pps(const time_spec_t &time_spec, size_t mboard){
    _tree->access<time_spec_t>(mb_root(0) / "time/pps").set(time_spec);
    return;
}

void multi_crimson_tng::set_time_unknown_pps(const time_spec_t &time_spec){
    // Not implemented
    throw uhd::not_implemented_error("timed command feature not implemented on this hardware");
}

bool multi_crimson_tng::get_time_synchronized(void){
    // Not implemented
    throw uhd::not_implemented_error("timed command feature not implemented on this hardware");
    return true;
}
void multi_crimson_tng::set_command_time(const time_spec_t &time_spec, size_t mboard){
    // Not implemented
    throw uhd::not_implemented_error("timed command feature not implemented on this hardware");
    return;
}
void multi_crimson_tng::clear_command_time(size_t mboard){
    // Not implemented
    throw uhd::not_implemented_error("timed command feature not implemented on this hardware");
    return;
}

void multi_crimson_tng::issue_stream_cmd(const stream_cmd_t &stream_cmd, size_t chan){
//    switch( stream_cmd.stream_mode ) {
//    case stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
//    case stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE:
//    case stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE:
//    	// currently we do not differentiate between any kind of stream-enable
//    	_tree->access<std::string>(rx_link_root(chan) / "stream").set("1");
//    	break;
//    case stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS:
//    default:
//    	_tree->access<std::string>(rx_link_root(chan) / "stream").set("0");
//    	break;
//    }
}

void multi_crimson_tng::set_clock_config(const clock_config_t &clock_config, size_t mboard) {
    //set the reference source...
    std::string clock_source;
    switch(clock_config.ref_source){
    case clock_config_t::REF_INT: clock_source = "internal"; break;
    case clock_config_t::REF_SMA: clock_source = "external"; break;
    default: clock_source = "internal";
    }
    _tree->access<std::string>(mb_root(0) / "clock_source" / "value").set(clock_source);

    //set the time source
    std::string time_source;
    switch(clock_config.pps_source){
    case clock_config_t::PPS_INT: time_source = "internal"; break;
    case clock_config_t::PPS_SMA: time_source = "external"; break;
    default: time_source = "internal";
    }
    _tree->access<std::string>(mb_root(0) / "time_source" / "value").set(clock_source);

    return;
}

// set the current time source
void multi_crimson_tng::set_time_source(const std::string &source, const size_t mboard){
    _tree->access<std::string>(mb_root(0) / "time_source" / "value").set(source);
    return;
}

// get the current time source
std::string multi_crimson_tng::get_time_source(const size_t mboard){
    return _tree->access<std::string>(mb_root(0) / "time_source" / "value").get();
}

// get all possible time sources
std::vector<std::string> multi_crimson_tng::get_time_sources(const size_t mboard){
    return _tree->access<std::vector<std::string> >(mb_root(0) / "time_source" / "options").get();
}

// set the current clock source (Crimson only uses internal reference for now, because it is a really good clock already)
void multi_crimson_tng::set_clock_source(const std::string &source, const size_t mboard){
    //_tree->access<std::string>(mb_root(0) / "clock_source" / "value").set(source);
    return;
}

// get the current clock source
std::string multi_crimson_tng::get_clock_source(const size_t mboard){
    return _tree->access<std::string>(mb_root(0) / "clock_source" / "value").get();
}

// get all possible clock sources
std::vector<std::string> multi_crimson_tng::get_clock_sources(const size_t mboard){
    return _tree->access<std::vector<std::string> >(mb_root(0) / "clock_source" / "options").get();
}

// Set the clock source output, Crimson doesn't have an SMA for clock out
void multi_crimson_tng::set_clock_source_out(const bool enb, const size_t mboard){
    // Not supported
    throw uhd::runtime_error("multi_crimson_tng::set_clock_source_out - not supported on this device");
    return;
}

// Set the time source output, Crimson doesn't have an SMA for time out
void multi_crimson_tng::set_time_source_out(const bool enb, const size_t mboard){
    // Not supported
    throw uhd::runtime_error("multi_crimson_tng::set_time_source_out - not supported on this device");
    return;
}

// Crimson only has support for 1 Digital (mboard) board.
size_t multi_crimson_tng::get_num_mboards(void){
    return 1;
}

sensor_value_t multi_crimson_tng::get_mboard_sensor(const std::string &name, size_t mboard){
    return _tree->access<sensor_value_t>(mb_root(0) / "sensors" / name).get();
}

std::vector<std::string> multi_crimson_tng::get_mboard_sensor_names(size_t mboard){
    return _tree->list(mb_root(0) / "sensors");
}

void multi_crimson_tng::set_user_register(const boost::uint8_t addr, const boost::uint32_t data, size_t mboard){
    // Not implemented
    throw uhd::not_implemented_error("timed command feature not implemented on this hardware");
}

/*******************************************************************
 * RX methods
 ******************************************************************/
rx_streamer::sptr multi_crimson_tng::get_rx_stream(const stream_args_t &args) {
    _check_tng_link_rate(args);
    return this->get_device()->get_rx_stream(args);
}

// Crimson does not support changing subdev properties because you can't add daughter boards
void multi_crimson_tng::set_rx_subdev_spec(const subdev_spec_t &spec, size_t mboard){
    throw uhd::runtime_error("multi_crimson_tng::set_tx_subdev_spec - not supported on this device");
}

// Get the current RX chain subdev properties, Crimson only has one mboard
subdev_spec_t multi_crimson_tng::get_rx_subdev_spec(size_t mboard){
    return subdev_spec_t("Slot_1:RX_Chain_1 Slot_2:RX_Chain_2 Slot_3:RX_Chain_3 Slot_4:RX_Chain_4");
}

// Get number of RX channels, Crimson has 4
size_t multi_crimson_tng::get_rx_num_channels(void){
    return 4;
}

// Get the name of the Crimson subdevice on specified channel
std::string multi_crimson_tng::get_rx_subdev_name(size_t chan){
    return "Channel " + boost::lexical_cast<std::string>(chan);
}

// Set the current RX sampling rate on specified channel
void multi_crimson_tng::set_rx_rate(double rate, size_t chan){

	std::list<size_t> _chan(
		ALL_CHANS == chan
		? CRIMSON_TNG_RX_CHANNELS
		: 0
	);

	if ( ALL_CHANS == chan ) {
		std::iota( std::begin( _chan ), std::end( _chan ), 0 );
	} else {
		_chan.push_back( chan );
	}

	for( auto &ch: _chan ) {

		_tree->access<double>(rx_dsp_root(ch) / "rate" / "value").set(rate);

		double actual_rate = _tree->access<double>(rx_dsp_root(ch) / "rate" / "value").get();

//		boost::format base_message (
//				"RX Sample Rate Request:\n"
//				"  Requested sample rate: %f MSps\n"
//				"  Actual sample rate: %f MSps\n");
//		base_message % (rate/1e6) % (actual_rate/1e6);
//		std::string results_string = base_message.str();
//
//		UHD_MSG(status) << results_string;
	}
}

// Get the current RX sampling rate on specified channel
double multi_crimson_tng::get_rx_rate(size_t chan){
    return _tree->access<double>(rx_dsp_root(chan) / "rate" / "value").get();
}

// get the range of possible RX rates on specified channel
meta_range_t multi_crimson_tng::get_rx_rates(size_t chan){
    return _tree->access<meta_range_t>(rx_dsp_root(chan) / "rate" / "range").get();
}

// set the RX frequency on specified channel
tune_result_t multi_crimson_tng::set_rx_freq( const tune_request_t &tune_request, size_t chan ) {

	tune_result_t result;

	double gain = get_rx_gain( chan );

	result =
		tune_lo_and_dsp(
			RX_SIGN,
			_tree->subtree( rx_dsp_root( chan ) ),
			_tree->subtree( rx_rf_fe_root( chan ) ),
			tune_request
		);

	do_tune_freq_results_message( (tune_request_t &) tune_request, result, get_rx_freq( chan ), "RX" );

	set_rx_gain( gain, chan );

	return result;
}

// get the RX frequency on specified channel
double multi_crimson_tng::get_rx_freq(size_t chan){
    double cur_dsp_nco = _tree->access<double>(rx_dsp_root(chan) / "nco").get();
    double cur_lo_freq = 0;
    if (_tree->access<int>(rx_rf_fe_root(chan) / "freq" / "band").get() == 1) {
        cur_lo_freq = _tree->access<double>(rx_rf_fe_root(chan) / "freq" / "value").get();
    }
    return cur_lo_freq - cur_dsp_nco;
}

// get the RX frequency range on specified channel
freq_range_t multi_crimson_tng::get_rx_freq_range(size_t chan){
    return _tree->access<meta_range_t>(rx_dsp_root(chan) / "freq" / "range").get();
}

// get front end RX frequency on specified channel
freq_range_t multi_crimson_tng::get_fe_rx_freq_range(size_t chan){
    return _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "freq" / "range").get();
}

// set RX frontend gain on specified channel, name specifies which IC to configure the gain for
void multi_crimson_tng::set_rx_gain(double gain, const std::string &name, size_t chan) {

	double atten_val = 0;
	double gain_val = 0;
	double lna_val = 0;

	gain = gain < CRIMSON_TNG_RF_RX_GAIN_RANGE_START ? CRIMSON_TNG_RF_RX_GAIN_RANGE_START : gain;
	gain = gain > CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP ? CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP : gain;

	if ( 0 == _tree->access<int>(rx_rf_fe_root(chan) / "freq" / "band").get() ) {
		// Low-Band

		double low_band_gain = gain > 31.5 ? 31.5 : gain;

		if ( low_band_gain != gain ) {
            boost::format rf_lo_message(
                "  The RF Low Band does not support the requested gain:\n"
                "    Requested RF Low Band gain: %f dB\n"
                "    Actual RF Low Band gain: %f dB\n"
            );
            rf_lo_message % gain % low_band_gain;
            std::string results_string = rf_lo_message.str();
        	UHD_MSG(status) << results_string;
		}

		// PMA is off (+0dB)
		lna_val = 0;
		// BFP is off (+0dB)
		// PE437 fully attenuates the BFP (-20 dB) AND THEN SOME
		atten_val = 31.75;
		// LMH is adjusted from 0dB to 31.5dB
		gain_val = low_band_gain;

	} else {
		// High-Band

		if ( false ) {
		} else if ( CRIMSON_TNG_RF_RX_GAIN_RANGE_START <= gain && gain <= 31.5 ) {
			// PMA is off (+0dB)
			lna_val = 0;
			// BFP is on (+20dB)
			// PE437 fully attenuates BFP (-20dB) AND THEN SOME (e.g. to attenuate interferers)
			atten_val = 31.75;
			// LMH is adjusted from 0dB to 31.5dB
			gain_val = gain;
		} else if ( 31.5 < gain && gain <= 63.25 ) {
			// PMA is off (+0dB)
			lna_val = 0;
			// BFP is on (+20dB)
			// PE437 is adjusted from -31.75 dB to 0dB
			atten_val = 63.25 - gain;
			// LMH is maxed (+31.5dB)
			gain_val = 31.5;
		} else if ( 63.25 < gain && gain <= CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP ) {
			// PMA is on (+20dB)
			lna_val = 20;
			// BFP is on (+20dB)
			// PE437 is adjusted from -20 dB to 0dB
			atten_val = CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP - gain;
			// LMH is maxed (+31.5dB)
			gain_val = 31.5;
		}
	}

	int lna_bypass_enable = 0 == lna_val ? 1 : 0;
	_tree->access<int>( rx_rf_fe_root(chan) / "freq" / "lna" ).set( lna_bypass_enable );

    if ( 0 == _tree->access<int>( cm_root() / "chanmask-rx" ).get() ) {
		_tree->access<double>( rx_rf_fe_root(chan) / "atten" / "value" ).set( atten_val * 4 );
		_tree->access<double>( rx_rf_fe_root(chan) / "gain" / "value" ).set( gain_val * 4 );
    } else {
		_tree->access<double>( cm_root() / "rx/atten/val" ).set( atten_val * 4 );
		_tree->access<double>( cm_root() / "rx/gain/val" ).set( gain_val * 4 );
    }
}

// get RX frontend gain on specified channel
double multi_crimson_tng::get_rx_gain(const std::string &name, size_t chan){

	double r;

	bool lna_bypass_enable = 0 == _tree->access<int>(rx_rf_fe_root(chan) / "freq" / "lna").get() ? false : true;
    double lna_val = lna_bypass_enable ? 0 : 20;
    double gain_val  = _tree->access<double>(rx_rf_fe_root(chan) / "gain"  / "value").get() / 4;
    double atten_val = _tree->access<double>(rx_rf_fe_root(chan) / "atten" / "value").get() / 4;

	if ( 0 == _tree->access<int>(rx_rf_fe_root(chan) / "freq" / "band").get() ) {
		r = gain_val;
	} else {
		r = 31.75 - atten_val + lna_val + gain_val; // maximum is 83.25
	}

    return r;
}

// get RX frontend gain range on specified channel
gain_range_t multi_crimson_tng::get_rx_gain_range(const std::string &name, size_t chan){
    return _tree->access<meta_range_t>(rx_rf_fe_root(chan) / "gain" / "range").get();
}

// get RX frontend gain names/options. There is only one configurable gain on the RX rf chain.
std::vector<std::string> multi_crimson_tng::get_rx_gain_names(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_rx_gain_names - not supported on this device");
}

// Crimson does not cater to antenna specifications, this is up to the user to accomodate for
void multi_crimson_tng::set_rx_antenna(const std::string &ant, size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::set_rx_antenna - not supported on this device");
}

// Crimson does not cater to antenna specifications, this is up to the user to accomodate for
std::string multi_crimson_tng::get_rx_antenna(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_rx_antenna - not supported on this device");
}

// Crimson does not cater to antenna specifications, this is up to the user to accomodate for
std::vector<std::string> multi_crimson_tng::get_rx_antennas(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_rx_antennas - not supported on this device");
}

// Set the RX bandwidth on specified channel
void multi_crimson_tng::set_rx_bandwidth(double bandwidth, size_t chan){
    _tree->access<double>(rx_dsp_root(chan) / "bw" / "value").set(bandwidth);
}
// Get the RX bandwidth on specified channel
double multi_crimson_tng::get_rx_bandwidth(size_t chan){
    return _tree->access<double>(rx_dsp_root(chan) / "bw" / "value").get();
}
// Get the RX bandwidth range on specified channel
meta_range_t multi_crimson_tng::get_rx_bandwidth_range(size_t chan){
    return _tree->access<meta_range_t>(rx_dsp_root(chan) / "bw" / "range").get();
}

// There is no dboard interface available for Crimson. Everything is communicated through the mboard (Digital board)
dboard_iface::sptr multi_crimson_tng::get_rx_dboard_iface(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_rx_dboard_iface - not supported on this device");
}

sensor_value_t multi_crimson_tng::get_rx_sensor(const std::string &name, size_t chan){
    return _tree->access<sensor_value_t>(rx_rf_fe_root(0) / "sensors" / name).get();
}

std::vector<std::string> multi_crimson_tng::get_rx_sensor_names(size_t chan){
    return _tree->list(rx_rf_fe_root(0) / "sensors");
}

// Enable dc offset on specified channel
void multi_crimson_tng::set_rx_dc_offset(const bool enb, size_t chan){
    _tree->access<bool>(mb_root(0) / "rx_frontends" / chan_to_string(chan) / "dc_offset" / "enable").set(enb);
}
// Set dc offset on specified channel
void multi_crimson_tng::set_rx_dc_offset(const std::complex<double> &offset, size_t chan){
    _tree->access< std::complex<double> >(mb_root(0) / "rx_frontends" / chan_to_string(chan) / "dc_offset" / "value").set(offset);
}
// set iq balance on specified channel
void multi_crimson_tng::set_rx_iq_balance(const std::complex<double> &offset, size_t chan){
    _tree->access< std::complex<double> >(mb_root(0) / "rx_frontends" / chan_to_string(chan) / "iq_balance" / "value").set(offset);
}

/*******************************************************************
 * TX methods
 ******************************************************************/
tx_streamer::sptr multi_crimson_tng::get_tx_stream(const stream_args_t &args) {
    _check_tng_link_rate(args);
    return this->get_device()->get_tx_stream(args);
}

// Crimson does not support changing subdev properties because you can't add daughter boards
void multi_crimson_tng::set_tx_subdev_spec(const subdev_spec_t &spec, size_t mboard){
    throw uhd::runtime_error("multi_crimson_tng::set_tx_subdev_spec - not supported on this device");
}

// Get the current TX chain subdev properties, Crimson only has one mboard
subdev_spec_t multi_crimson_tng::get_tx_subdev_spec(size_t mboard){
    return subdev_spec_t("Slot_1:TX_Chain_1 Slot_2:TX_Chain_2 Slot_3:TX_Chain_3 Slot_4:TX_Chain_4");
}

// Get number of TX channels, Crimson has 4
size_t multi_crimson_tng::get_tx_num_channels(void){
    return 4;
}

// Get the name of the Crimson subdevice on specified channel
std::string multi_crimson_tng::get_tx_subdev_name(size_t chan){
    return "Channel " + boost::lexical_cast<std::string>(chan);
}

// Set the current TX sampling rate on specified channel
void multi_crimson_tng::set_tx_rate(double rate, size_t chan){

	std::list<size_t> _chan(
		ALL_CHANS == chan
		? CRIMSON_TNG_TX_CHANNELS
		: 0
	);

	if ( ALL_CHANS == chan ) {
		std::iota( std::begin( _chan ), std::end( _chan ), 0 );
	} else {
		_chan.push_back( chan );
	}

	for( auto &ch: _chan ) {

		_tree->access<double>(tx_dsp_root(ch) / "rate" / "value").set(rate);

		double actual_rate = _tree->access<double>(tx_dsp_root(ch) / "rate" / "value").get();

		boost::format base_message (
				"TX Sample Rate Request:\n"
				"  Requested sample rate: %f MSps\n"
				"  Actual sample rate: %f MSps\n");
		base_message % (rate/1e6) % (actual_rate/1e6);
		std::string results_string = base_message.str();

		UHD_MSG(status) << results_string;
	}
}

// Get the current TX sampling rate on specified channel
double multi_crimson_tng::get_tx_rate(size_t chan){
    return _tree->access<double>(tx_dsp_root(chan) / "rate" / "value").get();
}

// get the range of possible TX rates on specified channel
meta_range_t multi_crimson_tng::get_tx_rates(size_t chan){
    return _tree->access<meta_range_t>(tx_dsp_root(chan) / "rate" / "range").get();
}

// set the TX frequency on specified channel
tune_result_t multi_crimson_tng::set_tx_freq(const tune_request_t & tune_request, size_t chan) {

	tune_result_t result;

	double gain = get_tx_gain( chan );

	result =
		tune_lo_and_dsp(
			TX_SIGN,
			_tree->subtree( tx_dsp_root( chan ) ),
			_tree->subtree( tx_rf_fe_root( chan ) ),
			tune_request
		);

	do_tune_freq_results_message( (tune_request_t &) tune_request, result, get_rx_freq( chan ), "TX" );

	set_tx_gain( gain, chan );

	return result;
}

// get the TX frequency on specified channel
double multi_crimson_tng::get_tx_freq(size_t chan){
    double cur_dac_nco = _tree->access<double>(tx_rf_fe_root(chan) / "nco").get();
    double cur_dsp_nco = _tree->access<double>(tx_dsp_root(chan) / "nco").get();
    double cur_lo_freq = 0;
    if (_tree->access<int>(tx_rf_fe_root(chan) / "freq" / "band").get() == 1) {
    	cur_lo_freq = _tree->access<double>(tx_rf_fe_root(chan) / "freq" / "value").get();
    }
    return cur_lo_freq + cur_dac_nco + cur_dsp_nco;
}

// get the TX frequency on specified channel
freq_range_t multi_crimson_tng::get_tx_freq_range(size_t chan){
    return _tree->access<meta_range_t>(tx_dsp_root(chan) / "freq" / "range").get();
}

// get the TX frequency range on specified channel
freq_range_t multi_crimson_tng::get_fe_tx_freq_range(size_t chan){
    return _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "freq" / "range").get();
}

// set TX frontend gain on specified channel, name specifies which IC to configure the gain for
void multi_crimson_tng::set_tx_gain(double gain, const std::string &name, size_t chan){
	double MAX_GAIN = 31.75;
	double MIN_GAIN = 0;

	if 		(gain > MAX_GAIN) gain = MAX_GAIN;
	else if (gain < MIN_GAIN) gain = MIN_GAIN;

	gain = round(gain / 0.25);

    if ( 0 == _tree->access<int>( cm_root() / "chanmask-tx" ).get() ) {
    	_tree->access<double>(tx_rf_fe_root(chan) / "gain" / "value").set(gain);
    } else {
    	_tree->access<double>( cm_root() / "tx/gain/val").set(gain);
    }
}

// get TX frontend gain on specified channel
double multi_crimson_tng::get_tx_gain(const std::string &name, size_t chan){
    double gain_val = _tree->access<double>(tx_rf_fe_root(chan) / "gain" / "value").get();
    return gain_val * 0.25;
}

// get TX frontend gain range on specified channel
gain_range_t multi_crimson_tng::get_tx_gain_range(const std::string &name, size_t chan){
    return _tree->access<meta_range_t>(tx_rf_fe_root(chan) / "gain" / "range").get();
}

// get TX frontend gain names/options. There is only one configurable gain on the TX rf chain.
std::vector<std::string> multi_crimson_tng::get_tx_gain_names(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_tx_gain_names - not supported on this device");
}

// Crimson does not cater to antenna specifications, this is up to the user to accomodate for
void multi_crimson_tng::set_tx_antenna(const std::string &ant, size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::set_tx_antenna - not supported on this device");
}

// Crimson does not cater to antenna specifications, this is up to the user to accomodate for
std::string multi_crimson_tng::get_tx_antenna(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_tx_antenna - not supported on this device");
}

// Crimson does not cater to antenna specifications, this is up to the user to accomodate for
std::vector<std::string> multi_crimson_tng::get_tx_antennas(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_tx_antennas - not supported on this device");
}

// Set the TX bandwidth on specified channel
void multi_crimson_tng::set_tx_bandwidth(double bandwidth, size_t chan){
    _tree->access<double>(tx_dsp_root(chan) / "bw" / "value").set(bandwidth);
}

// Get the TX bandwidth on specified channel
double multi_crimson_tng::get_tx_bandwidth(size_t chan){
    return _tree->access<double>(tx_dsp_root(chan) / "bw" / "value").get();
}

// Get the TX bandwidth range on specified channel
meta_range_t multi_crimson_tng::get_tx_bandwidth_range(size_t chan){
    return _tree->access<meta_range_t>(tx_dsp_root(chan) / "bw" / "range").get();
}

// There is no dboard interface available for Crimson. Everything is communicated through the mboard (Digital board)
dboard_iface::sptr multi_crimson_tng::get_tx_dboard_iface(size_t chan){
    throw uhd::runtime_error("multi_crimson_tng::get_tx_dboard_iface - not supported on this device");
}

sensor_value_t multi_crimson_tng::get_tx_sensor(const std::string &name, size_t chan){
    return _tree->access<sensor_value_t>(tx_rf_fe_root(chan) / "sensors" / name).get();
}

std::vector<std::string> multi_crimson_tng::get_tx_sensor_names(size_t chan){
    return _tree->list(tx_rf_fe_root(chan) / "sensors");
}

// Set dc offset on specified channel
void multi_crimson_tng::set_tx_dc_offset(const std::complex<double> &offset, size_t chan){
    _tree->access< std::complex<double> >(tx_rf_fe_root(chan) / "dc_offset" / "value").set(offset);
}

// set iq balance on specified channel
void multi_crimson_tng::set_tx_iq_balance(const std::complex<double> &offset, size_t chan){
    _tree->access< std::complex<double> >(tx_rf_fe_root(chan) / "iq_balance" / "value").set(offset);
}

/*******************************************************************
 * GPIO methods
 ******************************************************************/
std::vector<std::string> multi_crimson_tng::get_gpio_banks(const size_t mboard){
    // Not supported
    throw uhd::runtime_error("multi_crimson_tng::get_gpio_banks - not supported on this device");
}

void multi_crimson_tng::set_gpio_attr(const std::string &bank, const std::string &attr,
    const boost::uint32_t value, const boost::uint32_t mask, const size_t mboard){
    // Not supported
    throw uhd::runtime_error("multi_crimson_tng::set_gpio_attr - not supported on this device");
}

boost::uint32_t multi_crimson_tng::get_gpio_attr(const std::string &bank, const std::string &attr, const size_t mboard){
    // Not supported
    throw uhd::runtime_error("multi_crimson_tng::get_gpio_attr - not supported on this device");
}

/*******************************************************************
 * Helper methods
 ******************************************************************/
// Getting FS paths
fs_path multi_crimson_tng::mb_root(const size_t mboard)
{
    try
    {
        const std::string name = _tree->list("/mboards").at(mboard);
        return "/mboards/" + name;
    }
    catch(const std::exception &e)
    {
        throw uhd::index_error(str(boost::format("multi_usrp::mb_root(%u) - %s") % mboard % e.what()));
    }
}

fs_path multi_crimson_tng::rx_rf_fe_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_RX_CHANNELS) 	channel = 0;
    else				channel = chan;

    return mb_root(0) / "dboards" / chan_to_alph(channel) / "rx_frontends" / chan_to_string(channel);
}

fs_path multi_crimson_tng::rx_dsp_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_RX_CHANNELS) 	channel = 0;
    else				channel = chan;
    return mb_root(0) / "rx_dsps" / chan_to_string(channel);
}

fs_path multi_crimson_tng::rx_link_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_RX_CHANNELS) 	channel = 0;
    else				channel = chan;
    return mb_root(0) / "rx_link" / chan_to_string(channel);
}

fs_path multi_crimson_tng::tx_rf_fe_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_TX_CHANNELS) 	channel = 0;
    else				channel = chan;

    return mb_root(0) / "dboards" / chan_to_alph(channel) / "tx_frontends" / chan_to_string(channel);
}

fs_path multi_crimson_tng::tx_dsp_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_TX_CHANNELS) 	channel = 0;
    else				channel = chan;
    return mb_root(0) / "tx_dsps" / chan_to_string(channel);
}

fs_path multi_crimson_tng::tx_link_root(const size_t chan) {
    size_t channel;
    if (chan > CRIMSON_TX_CHANNELS) 	channel = 0;
    else				channel = chan;
    return mb_root(0) / "tx_link" / chan_to_string(channel);
}

fs_path multi_crimson_tng::cm_root() {
    return mb_root(0) / "cm";
}

// Channel string handling
std::string multi_crimson_tng::chan_to_string(size_t chan) {
    return "Channel_" + boost::lexical_cast<std::string>((char)(chan + 65));
}

std::string multi_crimson_tng::chan_to_alph(size_t chan) {
    return boost::lexical_cast<std::string>((char)(chan + 65));
}
