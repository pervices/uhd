//
// Copyright 2011-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

#include <filesystem>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

// Manages
template <typename samp_type>
class SampleManager {
public:
    size_t buffers_per_file; // Number of buffers containing spb required per file
    size_t total_samples; // Number of samples in the file

    // max_buffered_samples: maximum number of samples to be stored in memory
    // spb: number of samples to send at a time
    SampleManager(std::string filepath, size_t max_buffered_samples, size_t spb)
    :
    _max_buffered_samples((max_buffered_samples/spb) * spb),
    _spb(spb)
    {
        size_t file_size = std::filesystem::file_size(filepath);

        // Makes sure the file is the correct size
        if(file_size % sizeof(samp_type) != 0) {
            UHD_LOG_ERROR("TX_SAMPLES_FROM_FILE", "File does not contain a whole number of samples");
            throw std::invalid_argument("File does not contain a whole number of samples");
        }

        total_samples = file_size / sizeof(samp_type);

        buffers_per_file = std::ceil((double)total_samples/spb);

        // Maximum number of buffers that can fit spb samples within size max_buffered_samples
        size_t max_buffers_storable = max_buffered_samples / spb;

        if(buffers_per_file <= max_buffers_storable) {
            _large_file = false;
            buffers_storable = buffers_per_file;
        } else {
            _large_file = true;
            buffers_storable = max_buffers_storable;
        }

        sample_buffer = std::vector<samp_type>(buffers_storable * spb, 0);

        buffer_status = std::vector<std::atomic<int8_t>>(buffers_storable);
        for(auto& s : buffer_status) {
            s = 0;
        }

        sample_fd = open(filepath.c_str(), O_RDONLY);
        if(sample_fd == -1) {
            UHD_LOG_ERROR("TX_SAMPLES_FROM_FILE", "Unable to open file: " + filepath + ". Failed with error code: " + strerror(errno));
            throw std::runtime_error("Sample file not found");
        }

        // Fills the buffer
        for(size_t n = 0; n < buffers_storable; n++) {
            load();
        }

    }

    SampleManager() {
        if(sample_fd != -1) {
            close(sample_fd);
        }
    }

    // Releases samples from the buffer after they are no longer immediatly needed
    void release() {
        if(_large_file) {
            // Sets the flag indicating that this buffer is finished
            buffer_status[real_buffer(buffers_consumed)] = 0;
            buffers_consumed++;
            // Load samples into the freed up space
            load();
        } else {
            // Do not release the samples, the entire file can fit in memory
            buffers_consumed++;
        }
    }

    // Gets a pointer to a buffer containing the next nsamps
    // nsamps: number of samples obtained is written here
    // return: pointer for a buffer containing the number of samples written to nsamps. Will be _spb for everything but the last send buffer per pass through the file
    samp_type* get_samples(size_t* nsamps) {

        *nsamps = get_num_samples_in_buffer(buffers_requested);

        samp_type* buffer_start = &sample_buffer[(buffers_requested % buffers_storable) * _spb];

        // Waits until buffer to get returned is loaded
        // Loading should be initiated on startup/when clearing
        while(!buffer_status[real_buffer(buffers_requested)]) {
            // Busy wait
        }

        buffers_requested++;

        return buffer_start;
    }

private:
    bool _large_file; // True if unable to fit all samples into the buffer
    const size_t _max_buffered_samples; // Maximum number of samples that can be stored in memory
    const size_t _spb; // Samples per send command

    int sample_fd = -1;

    size_t buffers_storable; // Maximum number of buffers than can be stored
    size_t buffers_consumed = 0;
    size_t buffers_requested = 0;
    size_t buffers_loaded = 0;
    size_t samples_loaded = 0;

    // Contains samples to be sent, divded up to be used as a ring buffer of buffers
    std::vector<samp_type> sample_buffer;

    // 1 = the corresponding sub-buffer in samples_buffer is loaded
    std::vector<std::atomic<int8_t>> buffer_status;

    // Returns which real buffer in the ring buffer of buffers is being used
    inline size_t real_buffer(size_t buffer) {
        return buffer % buffers_storable;
    }

    // Loads samples into the buffer
    // TODO: perform this asynchronously
    void load() {
        if(buffer_status[real_buffer(buffers_loaded)]) {
            printf("buffers_loaded: %lu\n", buffers_loaded);
            throw std::runtime_error("Attempting to load to section ring buffer that has not been unloaded");
        }

        size_t samples_to_load = get_num_samples_in_buffer(buffers_loaded);
        // Offset in the file for where to load samples from
        size_t sample_offset = (samples_loaded % sample_buffer.size()) * _spb;

        ssize_t ret = lseek(sample_fd, sample_offset * sizeof(samp_type), SEEK_SET);
        if(ret == -1) {
            UHD_LOG_ERROR("TX_SAMPLES_FROM_FILE", "Unable to seek file, failed with error code: " + std::string(strerror(errno)));
            throw std::runtime_error("Sample file not found");
        }

        ret = read(sample_fd, &sample_buffer[(buffers_loaded % buffers_storable) * _spb], samples_to_load * sizeof(samp_type));

        if(ret == -1) {
            UHD_LOG_ERROR("TX_SAMPLES_FROM_FILE", "Unable to read file, failed with error code: " + std::string(strerror(errno)));
            throw std::runtime_error("Sample file not found");
        } else if((size_t) ret != samples_to_load * sizeof(samp_type)) {
            UHD_LOG_ERROR("TX_SAMPLES_FROM_FILE", "Unexpected number of samples read");
            throw std::runtime_error("Sample file not found");
        }

        buffer_status[real_buffer(buffers_loaded)] = 1;
        buffers_loaded++;
        samples_loaded+=sample_offset;
    }

    // buffer_num: The number of the buffer in the sequence (starting at 0)
    // return: Number of samples that should be in that buffer
    size_t get_num_samples_in_buffer(size_t buffer_num) {
        // Which buffer in the cycle this is
        size_t cycle_point = buffer_num % buffers_per_file;

        size_t remaining_samples_in_file = total_samples - (cycle_point * _spb);

        return std::min(remaining_samples_in_file, _spb);
    }
};

template <typename samp_type>
void send_from_file(
    // sample_manager: Manages loading samples
    // start_time: time to start the burst, ignored if include_sob not set
    uhd::tx_streamer::sptr tx_stream, SampleManager<samp_type>* sample_manager, bool include_sob, bool include_eob, uhd::time_spec_t start_time)
{
    uhd::tx_metadata_t md;
    // If there is no delay send as one burst, otherwise have a start and end of burst for each send
    md.start_of_burst = include_sob;
    md.end_of_burst   = false;
    md.has_time_spec  = include_sob;
    md.time_spec = start_time;

    // loop until the entire file has been read
    size_t total_samples_sent = 0;
    for(size_t n = 0; n < sample_manager->buffers_per_file && !stop_signal_called; n++) {

        size_t samples_to_send;
        samp_type* sample_buffer = sample_manager->get_samples(&samples_to_send);

        std::vector<samp_type*> send_buff_ptr(tx_stream->get_num_channels(), sample_buffer);

        const size_t samples_sent = tx_stream->send(send_buff_ptr, samples_to_send, md, 30);

        total_samples_sent += samples_sent;

        if (samples_sent != samples_to_send) {
            UHD_LOG_ERROR("TX-STREAM",
                "The tx_stream timed out sending " << sample_manager->total_samples << " samples ("
                                                   << total_samples_sent << " sent).");
            return;
        }

        // Released the samples just sent and begins pre-loading future samples
        sample_manager->release();

        // Clear start_of_burst once any samples have been sent
        md.start_of_burst = md.start_of_burst && (samples_sent == 0);
        md.has_time_spec = false;
    }

    // Sends eob packet if requests
    if(include_eob) {
        md.end_of_burst = true;
        tx_stream->send("", 0, md);
    }

}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, file, type, ant, subdev, ref, wirefmt, channels;
    size_t spb, max_buffer, single_channel = 0;
    double rate, freq, gain, power, bw, delay, lo_offset = 0;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to read binary samples from")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
        ("max_buffer", po::value<size_t>(&max_buffer)->default_value(1e9), "maximum number of samples that may be stored in memory")
        ("spb", po::value<size_t>(&spb), "samples per send command. Useful for performance tuning")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("lo-offset", po::value<double>(&lo_offset), "Offset for frontend LO in Hz (optional)")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("power", po::value<double>(&power), "transmit power")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref), "clock reference (internal, external, mimo, gpsdo)")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8 or sc16)")
        ("delay", po::value<double>(&delay)->default_value(0.0), "specify a delay between repeated transmission of file (in seconds)")
        ("channel", po::value<size_t>(&single_channel), "which channel to use")
        ("channels", po::value<std::string>(&channels), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("repeat", "repeatedly transmit file")
        ("int-n", "tune USRP with integer-n tuning")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD TX samples from file %s") % desc << std::endl;
        return ~0;
    }

    bool repeat = vm.count("repeat") > 0;

    bool delay_used = delay != 0;

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args
              << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    // Channels
    std::vector<size_t> channel_nums;
    std::vector<std::string> channels_split;
    if (vm.count("channel")) {
        if (vm.count("channels")) {
            std::cout << "ERROR: Cannot specify 'channel' and 'channels'!" << std::endl;
            return EXIT_FAILURE;
        }
        if (single_channel >= usrp->get_tx_num_channels())
            throw std::runtime_error("Invalid channel specified.");
        channel_nums.push_back(single_channel);
    } else {
        // Provide default
        if (!vm.count("channels"))
            channels = "0";
        // Split string into 1 or more channels
        boost::split(channels_split, channels, boost::is_any_of("\"',"));
        for (std::string channel : channels_split) {
            if (boost::lexical_cast<size_t>(channel) >= usrp->get_tx_num_channels())
                throw std::runtime_error("Invalid channel(s) specified.");
            channel_nums.push_back(boost::lexical_cast<size_t>(channel));
        }
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        usrp->set_clock_source(ref);
    }

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev"))
        usrp->set_tx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    // set the sample rate
    if (not vm.count("rate")) {
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    for (std::size_t channel : channel_nums) {
        usrp->set_tx_rate(rate, channel);
        rate = usrp->get_tx_rate();
        std::cout << boost::format("Actual TX Rate: %f Msps...")
                         % (usrp->get_tx_rate(channel) / 1e6)
                  << std::endl
                  << std::endl;
    }

    // set the center frequency
    if (not vm.count("freq")) {
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

    bool manual_lo = vm.count("lo-offset");

    uhd::tune_request_t tune_request;
    if(manual_lo) {
        std::cout << boost::format("Setting TX Freq %f using LO Offset: %f MHz...") % (lo_offset / 1e6) % (lo_offset / 1e6) << std::endl;
        tune_request = uhd::tune_request_t(freq, lo_offset);
    } else {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq / 1e6) << std::endl;
        tune_request = uhd::tune_request_t(freq);
    }

    if (vm.count("int-n"))
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    for (std::size_t channel : channel_nums) {
        uhd::tune_result_t tune_result = usrp->set_tx_freq(tune_request, channel);
            if(manual_lo) {
                std::cout << boost::format("Actual TX LO Offset: %f MHz...") % (tune_result.actual_rf_freq / 1e6)
                    << std::endl;
            }
            std::cout << boost::format("Actual TX Freq: %f MHz...") % ((tune_result.actual_rf_freq + tune_result.actual_dsp_freq) / 1e6)
                    << std::endl
                    << std::endl;
    }

    // set the rf gain
    if (vm.count("power")) {
        for (std::size_t channel : channel_nums) {
            if (!usrp->has_tx_power_reference(channel)) {
                std::cout << "ERROR: USRP does not have a reference power API on channel "
                          << channel << "!" << std::endl;
                return EXIT_FAILURE;
            }
            std::cout << "Setting TX output power: " << power << " dBm..." << std::endl;
            usrp->set_tx_power_reference(power, channel);
            std::cout << "Actual TX output power: "
                      << usrp->get_tx_power_reference(channel) << " dBm..." << std::endl;
        }

        if (vm.count("gain")) {
            std::cout << "WARNING: If you specify both --power and --gain, "
                         " the latter will be ignored."
                      << std::endl;
        }
    } else if (vm.count("gain")) {
        for (std::size_t channel : channel_nums) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
            usrp->set_tx_gain(gain, channel);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % usrp->get_tx_gain(channel)
                      << std::endl
                      << std::endl;
        }
    }

    // set the analog frontend filter bandwidth
    if (vm.count("bw")) {
        std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % (bw / 1e6)
                  << std::endl;
        for (std::size_t channel : channel_nums) {
            usrp->set_tx_bandwidth(bw, channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % (usrp->get_tx_bandwidth(channel) / 1e6)
                      << std::endl
                      << std::endl;
        }
    }

    // set the antenna
    if (vm.count("ant")) {
        for (std::size_t channel : channel_nums) {
            usrp->set_tx_antenna(ant, channel);
        }
    }
    // allow for some setup time:
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    for (std::size_t channel : channel_nums) {
        sensor_names = usrp->get_tx_sensor_names(channel);
        if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
            != sensor_names.end()) {
            uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", channel);
            std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                      << std::endl;
            UHD_ASSERT_THROW(lo_locked.to_bool());
        }
    }
    sensor_names = usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked")
             != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
             != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    // set sigint if user wants to receive
    if (repeat) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // create a transmit streamer
    std::string cpu_format;
    if (type == "double")
        cpu_format = "fc64";
    else if (type == "float")
        cpu_format = "fc32";
    else if (type == "short")
        cpu_format = "sc16";

    uhd::stream_args_t stream_args(cpu_format, wirefmt);
    stream_args.channels             = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    if(!vm.count("spb")) {
        spb = tx_stream->get_max_num_samps()*10;
    }

    void* sample_manager;
    uhd::time_spec_t burst_duration;
    if (type == "double") {
        sample_manager = new SampleManager<std::complex<double>>(file, max_buffer, spb);
        burst_duration = uhd::time_spec_t::from_ticks(((SampleManager<std::complex<double>>*)sample_manager)->total_samples, rate);
    } else if (type == "float") {
        sample_manager = new SampleManager<std::complex<float>>(file, max_buffer, spb);
        burst_duration = uhd::time_spec_t::from_ticks(((SampleManager<std::complex<float>>*)sample_manager)->total_samples, rate);
    } else if (type == "short") {
        sample_manager = new SampleManager<std::complex<short>>(file, max_buffer, spb);
        burst_duration = uhd::time_spec_t::from_ticks(((SampleManager<std::complex<short>>*)sample_manager)->total_samples, rate);
    } else {
        throw std::runtime_error("Unknown type " + type);
    }

    // TODO: add check to verify delay is valid

    uhd::time_spec_t start_time = usrp->get_time_now() + 0.01;

    bool include_sob = true;
    // Include eob if either only 1 pulse is being sent or no delay is used
    bool include_eob = delay_used || !repeat;

    // send from file
    do {

        if (type == "double")
            send_from_file<std::complex<double>>(tx_stream, (SampleManager<std::complex<double>>*) sample_manager, include_sob, include_eob, start_time);
        else if (type == "float")
            send_from_file<std::complex<float>>(tx_stream, (SampleManager<std::complex<float>>*) sample_manager, include_sob, include_eob, start_time);
        else if (type == "short")
            send_from_file<std::complex<short>>(tx_stream, (SampleManager<std::complex<short>>*) sample_manager, include_sob, include_eob, start_time);
        else
            throw std::runtime_error("Unknown type " + type);

        // If there is no delay between bursts only the first burst contains a sob
        include_sob = delay_used;

        start_time+= burst_duration + delay;
    } while (repeat and not stop_signal_called);

    std::cout << "Streaming complete" << std::endl;

    if (type == "double")
        delete (SampleManager<std::complex<double>>*) sample_manager;
    else if (type == "float")
        delete (SampleManager<std::complex<float>>*) sample_manager;
    else if (type == "short")
        delete (SampleManager<std::complex<short>>*) sample_manager;
    else {
        throw std::runtime_error("Unknown type " + type);
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
