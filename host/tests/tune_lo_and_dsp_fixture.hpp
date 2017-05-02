#ifndef HOST_TESTS_TUNE_LO_AND_DSP_FIXTURE_HPP_
#define HOST_TESTS_TUNE_LO_AND_DSP_FIXTURE_HPP_

#include <map>
#include <string>

#include <boost/test/unit_test.hpp>

#include <uhd/types/ranges.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/types/tune_result.hpp>
#include <uhd/property_tree.hpp>

#include <boost/bind.hpp>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"

#define TREE_CREATE_ST(PATH, TYPE, VAL) 	{ _tree->create<TYPE>(PATH).set(VAL); TYPE ## _vars[ PATH ] = VAL;  }
#define TREE_CREATE_RW(PATH, PROP, TYPE, HANDLER)						\
	do { \
		TYPE ## _vars[ PATH ] = TYPE(); \
		_tree->create<TYPE> (PATH)								\
    		.set( get_ ## HANDLER (PATH))							\
		.subscribe(boost::bind(&tune_lo_and_dsp_fixture::set_ ## HANDLER, this, (PATH), _1))	\
		.publish  (boost::bind(&tune_lo_and_dsp_fixture::get_ ## HANDLER, this, (PATH)    ));	\
	} while(0)

using namespace std;
using namespace uhd;

static const double TX_SIGN = -1;
static const double RX_SIGN = 1;

// XXX: due to the default in g++ now being -fvisibility=hidden, tune_lo_and_dsp must have __attribute__ ((visibility ("default")))
// That C++ "feature" breaks all unit tests that require being linked to libuhd :(
extern tune_result_t tune_lo_and_dsp( const double xx_sign, property_tree::sptr dsp_subtree, property_tree::sptr rf_fe_subtree, const tune_request_t &tune_request );

struct tune_lo_and_dsp_fixture {

	bool tx;
	char channel;

	double fc;
	double bw;

	bool _true;

	tune_request_t req;
	tune_result_t res;

	property_tree::sptr dsp_subtree;
	property_tree::sptr rf_subtree;

	std::map<string,double> double_vars;
	std::map<string,int> int_vars;
	std::map<string,meta_range_t> meta_range_t_vars;

	tune_lo_and_dsp_fixture( char channel, bool tx, double fc, double bw )
	:
		tx( tx ),
		channel( channel ),
		fc( fc ),
		bw( bw ),
		_true( true )
	{
		const fs_path tx_fe_path    = "";
		const fs_path rx_dsp_path = "";
		const fs_path chan = "";

		// mock-out the dsp subtree and rf subtree
		// XXX: @CF: would be great if the code to populate this data wasn't necessarily bound to the crimson_tng_impl constructor
		dsp_subtree = property_tree::make();
		rf_subtree = property_tree::make();

		property_tree::sptr _tree;

		_tree = rf_subtree;
		if ( tx ) {
			TREE_CREATE_RW(tx_fe_path / "nco", "tx_"+lc_num+"/rf/dac/nco", double, double);
		}
		TREE_CREATE_ST(tx_fe_path / "freq" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_FREQ_RANGE_START, CRIMSON_TNG_FREQ_RANGE_STOP, CRIMSON_TNG_FREQ_RANGE_STEP));
		TREE_CREATE_RW(tx_fe_path / "freq" / "band", "rx_"+lc_num+"/rf/freq/band", int, int);
		TREE_CREATE_RW(tx_fe_path / chan / "freq" / "value", "rx_"+lc_num+"/rf/freq/val", double, double);

		_tree = dsp_subtree;
		TREE_CREATE_ST(rx_dsp_path / "rate" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_RATE_RANGE_START, CRIMSON_TNG_RATE_RANGE_STOP, CRIMSON_TNG_RATE_RANGE_STEP));
		TREE_CREATE_RW(rx_dsp_path / "rate" / "value", "rx_"+lc_num+"/dsp/rate",    double, double);
		TREE_CREATE_ST(rx_dsp_path / "freq" / "range", meta_range_t,
			meta_range_t(CRIMSON_TNG_DSP_FREQ_RANGE_START, CRIMSON_TNG_DSP_FREQ_RANGE_STOP, CRIMSON_TNG_DSP_FREQ_RANGE_STEP));
		TREE_CREATE_RW(rx_dsp_path / "freq" / "value", "rx_"+lc_num+"/dsp/nco_adj", double, double);

		req.target_freq = fc;
		dsp_subtree->access<double>( "/rate/value" ).set( bw );
	}

	double get_double( const std::string pre ) {
		if ( double_vars.end() == double_vars.find( pre ) ) {
			throw uhd::runtime_error( ( boost::format( "no property named %s" ) % pre ).str() );
		}
		return double_vars[ pre ];
	}
	void set_double( const std::string pre, double data ) {
		if ( double_vars.end() == double_vars.find( pre ) ) {
			throw uhd::runtime_error( ( boost::format( "no property named %s" ) % pre ).str() );
		}
		double_vars[ pre ] = data;
	}

	int get_int( const std::string pre ) {
		if ( int_vars.end() == int_vars.find( pre ) ) {
			throw uhd::runtime_error( ( boost::format( "no property named %s" ) % pre ).str() );
		}
		return int_vars[ pre ];
	}
	void set_int( const std::string pre, int data ) {
		if ( int_vars.end() == int_vars.find( pre ) ) {
			throw uhd::runtime_error( ( boost::format( "no property named %s" ) % pre ).str() );
		}
		int_vars[ pre ] = data;
	}
};


#endif /* HOST_TESTS_TUNE_LO_AND_DSP_FIXTURE_HPP_ */
