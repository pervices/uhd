//
// Copyright 2010-2012,2014 Ettus Research LLC
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
#include <csignal>
#include <thread>

#include "wavetable.hpp"
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

struct thread_ctx {
	thread_ctx()
	: should_exit( false ), buff( NULL )
	{
	}
	bool should_exit;
	const std::vector<std::complex<int16_t>> *buff;
	uhd::tx_streamer::sptr tx_stream;
	std::thread th;
	uhd::tx_metadata_t md;
};

static void thread_fn( thread_ctx *ctx ) {

	std::vector<std::complex<int16_t> *> buffs( 1, (std::complex<int16_t> *) & ctx->buff->front() );

	while( ! ctx->should_exit ) {

	    ctx->tx_stream->send( buffs, ctx->buff->size(), ctx->md );

	    ctx->md.start_of_burst = false;
	    ctx->md.has_time_spec = false;
	}

    //send a mini EOB packet
	ctx->md.end_of_burst = true;
	ctx->tx_stream->send("", 0, ctx->md);
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, wave_type, ant, subdev, ref, otw, channel_list;
    size_t spb;
    double rate, freq, gain, wave_freq;
    double sob;
    float ampl;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
		("sob", po::value<double>(&sob)->default_value(0.5), "start of burst in N seconds, 0 to disable")
        ("rate", po::value<double>(&rate)->default_value(10e6), "rate of outgoing samples")
        ("freq", po::value<double>(&freq)->default_value(2.4e9), "RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(1500)), "amplitude of the waveform [0 to 32767]")
        ("gain", po::value<double>(&gain)->default_value(20), "gain for the RF chain")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("SINE"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value( 5e6 ), "waveform frequency in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo)")
        ("channels", po::value<std::string>(&channel_list)->default_value("0,1,2,3"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels())
            throw std::runtime_error("Invalid channel(s) specified.");
        else
            channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }


    //Lock mboard clocks
    usrp->set_clock_source(ref);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;

    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        usrp->set_tx_freq(tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        //set the rf gain
        if (vm.count("gain")){
            std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
            usrp->set_tx_gain(gain, channel_nums[ch]);
            std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;
        }
    }

    boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

    //for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST"){
        wave_freq = usrp->get_tx_rate()/2;
    }

    //error when the waveform is not possible to generate
    if (std::abs(wave_freq) > usrp->get_tx_rate()/2){
        throw std::runtime_error("wave freq out of Nyquist zone");
    }
    if (usrp->get_tx_rate()/std::abs(wave_freq) > wave_table_len/2){
        throw std::runtime_error("wave freq too small for table");
    }

    //pre-compute the waveform values
    const wave_table_class_sc16 wave_table(wave_type, ampl);
    const size_t step = boost::math::iround(wave_freq/usrp->get_tx_rate() * wave_table_len);
    size_t index = 0;

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    std::vector<thread_ctx> ctx( channel_nums.size() );
    uhd::stream_args_t stream_args("sc16", "sc16");
    for( size_t i = 0; i < channel_nums.size(); i++ ) {
        stream_args.channels = std::vector<size_t>{ channel_nums[ i ] };
        std::cout << "Getting TX Streamer for Channel " << (char)('A' + channel_nums[ i ]) << std::endl;
        ctx[ i ].tx_stream = usrp->get_tx_stream(stream_args);
        std::cout << "Got TX Streamer for Channel " << (char)('A' + channel_nums[ i ]) << std::endl;
    }

    //allocate a buffer which we re-use for each channel
    if (spb == 0) spb = ctx[ 0 ].tx_stream->get_max_num_samps()*10;
    std::vector<std::complex<int16_t> > buff(spb);

    uhd::time_spec_t sob_time( usrp->get_time_now().get_real_secs() + sob );

    for( auto &_ctx: ctx ) {
    	_ctx.buff = & buff;

        //setup the metadata flags
        _ctx.md.start_of_burst = true;
        _ctx.md.end_of_burst   = false;

        if ( 0 == sob ) {
        	_ctx.md.has_time_spec = false;
        } else {
        	_ctx.md.has_time_spec = true;
        	_ctx.md.time_spec = sob_time;
        }

    }

    //std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    //usrp->set_time_now(uhd::time_spec_t(0.0));

    //Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked",0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked",0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked",0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    // kb 3974: there is absolutely no need to call this inside the for loop, as in tx_waveforms.cpp
    //fill the buffer with the waveform
    for (size_t n = 0; n < buff.size(); n++){
        buff[n] = wave_table(index += step);
    }

    for ( size_t i = 0; i < ctx.size(); i++ ) {
    	ctx[ i ].th = std::thread( thread_fn, & ctx[ i ] );
    }

    //send data until the signal handler gets called
    while(not stop_signal_called){
    	usleep( 1000000 );
    }

    for ( auto & _ctx: ctx ) {
    	_ctx.should_exit = true;
    	_ctx.th.join();
    }

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}

