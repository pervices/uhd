#include <iostream>

#include <uhd/usrp/multi_usrp.hpp>

using namespace std;

int main() {

	uhd::fs_path tx_fe_path = "/mboards/0/dboards/A/tx_frontends/Channel_A";
	uhd::fs_path tx_dsp_path = "/mboards/0/tx_dsps/Channel_A";

	double expected_target_freq = 945 * 1e6;
	//double actual_target_freq;

	double expected_lo_freq = 930 * 1e6;
	double actual_lo_freq;

	double expected_dac_nco_freq = 0;
	double actual_dac_nco_freq;

	double expected_dsp_nco_freq = 15 * 1e6;
	double actual_dsp_nco_freq;

    //create a usrp device
    uhd::usrp::multi_usrp::sptr tx_usrp = uhd::usrp::multi_usrp::make( uhd::device_addr_t() );

    uhd::tune_request_t expected_tune( expected_target_freq );

	tx_usrp->set_tx_freq( expected_tune, 0 );

	uhd::property_tree::sptr tree = tx_usrp->get_device()->get_tree();

	actual_lo_freq = tree->access<double>( tx_fe_path / "freq" / "value" ).get();
	actual_dac_nco_freq = tree->access<double>( tx_fe_path / "nco" ).get();
	actual_dsp_nco_freq = tree->access<double>( tx_dsp_path / "nco" ).get();

	std::cout << "Actual LO: " << actual_lo_freq << ", Expected LO: " << expected_lo_freq << std::endl;
	std::cout << "Actual DAC NCO: " << actual_dac_nco_freq << ", Expected DAC NCO: " << expected_dac_nco_freq << std::endl;
	std::cout << "Actual DSP NCO: " << actual_dsp_nco_freq << ", Expected DSP NCO: " << expected_dsp_nco_freq << std::endl;

    //finished
    return EXIT_SUCCESS;
}
