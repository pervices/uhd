//
// Copyright 2010-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef UC16_DATA
#define UC16_DATA 1
#endif

#if (UC16_DATA == 1)
#include "wavetable_sc16.hpp"
#else
#include "wavetable.hpp"
#endif

#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <iostream>
#include <csignal>
#include <string>
#include <chrono>
#include <thread>

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}
uhd::usrp::multi_usrp::sptr usrp;

/***********************************************************************
 * Helper functions
 **********************************************************************/
namespace gpio
{
	std::thread gpio_thread;

    void write(uhd::usrp::multi_usrp::sptr& usrp, const uint64_t pins [], const uint64_t mask [], const double time)
    {
        usrp->set_command_time(uhd::time_spec_t(time));
        // NOTE: We expect set_user_register to be called sequentially for these registers
        //       and the last register (3 for vaunt and 7 for tate) will trigger the
        //       gpio packet to be sent
        usrp->set_user_register(0, (uint32_t) (pins[0] >> 0x00)); // GPIO 31:0
        usrp->set_user_register(1, (uint32_t) (mask[0] >> 0x00)); // MASK for 31:0
        usrp->set_user_register(2, (uint32_t) (pins[0] >> 0x20)); // GPIO 63:32
        usrp->set_user_register(3, (uint32_t) (mask[0] >> 0x20)); // MASK for 63:32
#ifdef PV_TATE
        usrp->set_user_register(4, (uint32_t) (pins[1] >> 0x00)); // GPIO 95:64
        usrp->set_user_register(5, (uint32_t) (mask[1] >> 0x00)); // MASK for 95:64
        usrp->set_user_register(6, (uint32_t) (pins[1] >> 0x20)); // GPIO 128:96
        usrp->set_user_register(7, (uint32_t) (mask[1] >> 0x20)); // MASK for 128:96 (Also writes packet).
#endif
    }

    void gpio_main()
    {
#ifdef PV_TATE
        std::cout << "GPIO example for Tate" << std::endl;
        // Note that Tate has 80 GPIO pins
        // The following is the mapping of the GPIO pins to the registers
        //
        //    pwr_en        : Power on the HDR board
        //    hi_pwr_en     : Enable the high power branch
        //    atten64..1    : Amount of attenuation (all will be summed together).
        //                      9          8          7          6          5          4          3          2          1          0
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL A:   9 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |   0
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL B:  19 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  10
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL C:  29 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  20
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL D:  39 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  30
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL E:  49 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  40
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL F:  59 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  50
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL G:  69 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  60
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL H:  79 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  70
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        
        // default is to set pwr_en and enable hi_pwr branch and set attenuation to minimum (0).
        uint64_t pins_default [2] = {0x0601806018060180, 0x6018};
        uint64_t pins [2] = {0x0601806018060180, 0x6018};
        uint64_t mask [2] = {0xFFFFFFFFFFFFFFFF, 0xFFFF};
        double time = 0.0;
        uint64_t attenuation = 0;
        while (!stop_signal_called) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            time += 0.5;
            // Ramp up the attenuation every half second
            attenuation++;
            if (attenuation == 128) {
                attenuation = 0;
            }
            pins[0] = pins_default[0]   | 
                      (attenuation<< 0) | 
                      (attenuation<<10) |
                      (attenuation<<20) |
                      (attenuation<<30) |
                      (attenuation<<40) |
                      (attenuation<<50) |
                      (attenuation<<60);

            // Watch out for crossing the 64-bit boundary in the previous array entry
            pins[1] = pins_default[1]   |
                      (attenuation>> 4) |
                      (attenuation<< 6);


            gpio::write(usrp, pins, mask, time);
        }

        // Turn off the HDR boards when the program is terminated.
        time += 0.5;
        pins[0] = 0;
        pins[1] = 0;
        gpio::write(usrp, pins, mask, time);
#else
        std::cout << "GPIO example for Vaunt" << std::endl;
        // Note that Vaunt has 48 GPIO pins
        uint64_t pins = 0x0;
        const uint64_t mask = 0xFFFFFFFFFF;
        for(double time = 0.0; time < 64.0; time++) {
            // Toggle the pins for the next 64 seconds
            pins ^= mask;
            gpio::write(usrp, pins, mask, time);
            if (stop_signal_called) {
                break;
            }
        }
#endif

    }

    void start_gpio_thread()
    {
		gpio_thread = std::thread(gpio_main);
    }

}
/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, wave_type, ant, subdev, ref, pps, otw, channel_list;
    uint64_t total_num_samps;
    size_t spb;
    double rate, ch_freq, dp_freq, dsp_freq, gain, wave_freq, bw;
    float ampl;

    double first, last, increment;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("nsamps", po::value<uint64_t>(&total_num_samps)->default_value(0), "total number of samples to transmit")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("ch-freq", po::value<double>(&ch_freq), "DAC Channel RF center frequency in Hz")
        ("dp-freq", po::value<double>(&dp_freq), "DAC Datapath RF center frequency in Hz")
        ("dsp-freq", po::value<double>(&dsp_freq), "FPGA DSP center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("CONST"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo, gpsdo)")
        ("pps", po::value<std::string>(&pps)->default_value("internal"), "PPS source (internal, external, mimo, gpsdo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("int-n", "tune USRP with integer-N tuning")
        ("first", po::value<double>(&first)->default_value(5), "Time for first stacked command")
        ("last", po::value<double>(&last)->default_value(5), "Time for last stacked command")
        ("increment", po::value<double>(&increment)->default_value(1), "Increment for stack commands between <first> and <last> times")
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
    usrp = uhd::usrp::multi_usrp::make(args);

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_tx_subdev_spec(subdev);

    //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = std::stoi(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels())
            throw std::runtime_error("Invalid channel(s) specified.");
        else
            channel_nums.push_back(std::stoi(channel_strings[ch]));
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
    if (not vm.count("dp-freq")){
        std::cerr << "Please specify the DAC Datapath center frequency with --dp-freq" << std::endl;
        return ~0;
    }

    //set the center frequency
    if (not vm.count("ch-freq")){
        std::cerr << "Please specify the DAC Channel center frequency with --ch-freq" << std::endl;
        return ~0;
    }

    //set the center frequency
    if (not vm.count("dsp-freq")){
        std::cerr << "Please specify the DAC Channel center frequency with --dsp-freq" << std::endl;
        return ~0;
    }

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        double total_freq = ch_freq + dp_freq + dsp_freq;
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (total_freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(total_freq, ch_freq, dp_freq, 0.0, dsp_freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_tx_freq(tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        //set the rf gain
        if (vm.count("gain")){
            std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
            usrp->set_tx_gain(gain, channel_nums[ch]);
            std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;
        }

        //set the analog frontend filter bandwidth
        if (vm.count("bw")){
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % bw << std::endl;
            usrp->set_tx_bandwidth(bw, channel_nums[ch]);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...") % usrp->get_tx_bandwidth(channel_nums[ch]) << std::endl << std::endl;
        }

        //set the antenna
        if (vm.count("ant")) usrp->set_tx_antenna(ant, channel_nums[ch]);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1)); //allow for some setup time

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

#if (UC16_DATA == 1)
    const wave_table_class_sc16 wave_table(wave_type, ampl);
#else
    const wave_table_class wave_table(wave_type, ampl);
#endif
    const size_t step = boost::math::iround(wave_freq/usrp->get_tx_rate() * wave_table_len);
    std::cout << "Step size is : " << step << std::endl;
    size_t index = 0;

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    //
    // Note: sc16_item32_be means that the input is 16-bit complex numbers packed
    //       into a 32-bit uint32_t variable with the upper two bytes being the I
    //       sample and the lower two bytes being the Q sample
    //       I = item32_be[31:16]
    //       Q = item32_be[15:0]

#if (UC16_DATA == 1)
    otw = "uc16";
    uhd::stream_args_t stream_args("uc16_item32", otw);
#else
    otw = "sc16";
    uhd::stream_args_t stream_args("fc32", otw);
#endif

    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    //allocate a buffer which we re-use for each channel
    if (spb == 0) {
        spb = tx_stream->get_max_num_samps()*10;
    }

#if (UC16_DATA == 1)
    std::vector<uint32_t > buff(spb);
    std::vector<uint32_t *> buffs(channel_nums.size(), &buff.front());
#else
    std::vector<std::complex<float> > buff(spb);
    std::vector<std::complex<float> *> buffs(channel_nums.size(), &buff.front());
#endif

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    if (channel_nums.size() > 1)
    {
        // Sync times
        if (pps == "mimo")
        {
            UHD_ASSERT_THROW(usrp->get_num_mboards() == 2);

            //make mboard 1 a slave over the MIMO Cable
            usrp->set_time_source("mimo", 1);

            //set time on the master (mboard 0)
            usrp->set_time_now(uhd::time_spec_t(0.0), 0);

            //sleep a bit while the slave locks its time to the master
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else
        {
            if (pps == "internal" or pps == "external" or pps == "gpsdo")
                usrp->set_time_source(pps);
            usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
            std::this_thread::sleep_for(std::chrono::seconds(1)); //wait for pps sync pulse
        }
    }
    else
    {
        usrp->set_time_now(0.0);
    }

    //Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    const size_t tx_sensor_chan = channel_nums.empty() ? 0 : channel_nums[0];
    sensor_names = usrp->get_tx_sensor_names(tx_sensor_chan);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", tx_sensor_chan);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    const size_t mboard_sensor_idx = 0;
    sensor_names = usrp->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    usrp->set_time_now(0.0);

    gpio::start_gpio_thread();


    for(double time = first; time <= last; time += increment)
    {
        // Set up metadata. We start streaming a bit in the future
        // to allow MIMO operation:
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst   = false;
        md.has_time_spec  = true;
        md.time_spec = uhd::time_spec_t(time);

        //send data until the signal handler gets called
        //or if we accumulate the number of samples specified (unless it's 0)
        uint64_t num_acc_samps = 0;
        while(true){

            if (stop_signal_called)
                break;

            if (total_num_samps > 0 and num_acc_samps >= total_num_samps) {
                break;
            }

            //fill the buffer with the waveform
            for (size_t n = 0; n < buff.size(); n++){
                buff[n] = wave_table(index += step);
            }

            //send the entire contents of the buffer
            num_acc_samps += tx_stream->send(buffs, buff.size(), md);

            md.start_of_burst = false;
            md.has_time_spec = false;
        }

        //send a mini EOB packet
        md.end_of_burst = true;
        tx_stream->send("", 0, md);
    }
    gpio::gpio_thread.join();
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
