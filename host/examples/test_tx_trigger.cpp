//
// Copyright 2018 - 2019 Per Vices Corporation GPL 3
//

#include <boost/program_options.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/endian/conversion.hpp>
#include <boost/algorithm/string.hpp>
#include <uhd/utils/log.hpp>

#include <fstream>
#include <thread>
#include <mutex>

#include <csignal>
#undef NDEBUG
#include <cassert>

namespace Exit
{
    bool now {false};

    std::mutex mutex;

    void interrupt(int)
    {
        std::lock_guard<std::mutex> guard(mutex);
        now = true;
        std::cout << "\nInterrupt caught" << std::endl;
    }

    bool get()
    {
        std::lock_guard<std::mutex> guard(mutex);
        return now;
    }

    //
    // Ctrl-C will exit this program mid stream.
    //
    void wait_interrupt()
    {
        signal(SIGINT, interrupt);
    }
}

class Fifo
{
    uhd::transport::udp_simple::sptr link;

    uhd::usrp::multi_usrp::sptr usrp;
    const size_t channel {0};

public:
    Fifo(uhd::usrp::multi_usrp::sptr usrp, const size_t channel)
    :
    usrp {usrp},
    channel {channel}
    {
        std::string ip = usrp->get_tx_ip(channel);
        std::string port = std::to_string(usrp->get_tx_fc_port(channel));
        link = uhd::transport::udp_simple::make_connected(ip, port);
    }

    uint64_t get_level()
    {
        poke();
        uint64_t lvl = peek().header & 0xFFFF;
        lvl = lvl * usrp->get_tx_buff_scale();
        return lvl;
    }

private:
    struct Request
    {
        uint64_t header;

        //
        // FPGA endianess is reversed.
        //

        void flip()
        {
            boost::endian::big_to_native_inplace(header);
        }
    };

    struct Response
    {
        uint64_t header;
        uint64_t overflow;
        uint64_t underflow;
        uint64_t seconds;
        uint64_t ticks;

        //
        // FPGA endianess is reversed.
        //

        void flip()
        {
            boost::endian::big_to_native_inplace(header);
            boost::endian::big_to_native_inplace(overflow);
            boost::endian::big_to_native_inplace(underflow);
            boost::endian::big_to_native_inplace(seconds);
            boost::endian::big_to_native_inplace(ticks);
        }
    };

    void poke()
    {
        Request request = { 0x10001UL << 16 | (channel & 0xFFFF) };
        request.flip();
        link->send(boost::asio::mutable_buffer(&request, sizeof(request)));
    }

    Response peek()
    {
        Response response = {};
        link->recv(boost::asio::mutable_buffer(&response, sizeof(response)));
        response.flip();
        return response;
    }
};

class Trigger
{
    const std::vector<size_t> channels;

    const uhd::usrp::multi_usrp::sptr usrp;

    const std::string gating;

public:
    Trigger(uhd::usrp::multi_usrp::sptr usrp, const std::vector<size_t> channels, const int samples, const int edge_debounce, const std::string gating)
    :
    channels {channels},
    usrp {usrp},
    gating {gating}
    {
        for(const auto ch : channels)
            apply(sma(ch, samples, edge_debounce));
    }

    ~Trigger()
    {
        for(const auto ch : channels)
            apply(sma(ch, 0, 0));
    }

private:
    struct Set
    {
        const std::string path;
        const std::string string_value;
        const bool use_double;
        const double double_value;
        //verify if the porperty set in the state tree matches the value set, certain state tree values are not expected to match
        const bool verify;

        Set(std::string path, std::string value, bool verify) :
            path(path),
            string_value(value),
            use_double(false),
            double_value(0),
            verify(verify)
        {}

        Set(std::string path, double value, bool verify) :
            path(path),
            string_value(""),
            use_double(true),
            double_value(value),
            verify(verify)
        {}

        void print() const
        {
            if(use_double) {
                std::cout << path << " = " << string_value << std::endl;
            } else {
                std::cout << path << " = " << string_value << std::endl;
            }
        }
    };

    std::vector<Set> sma(const size_t channel, const int samples, const int edge_debounce) const
    {
        const std::string root { "/mboards/0/tx/" + std::to_string(channel) + "/" };
        const std::string dsp_root { "/mboards/0/tx_dsps/" + std::to_string(channel) + "/" };
        const std::vector<Set> sets {
            { root + "trigger/sma_mode"       , "edge"                          , true},
            { root + "trigger/trig_sel"       , samples > 0 ? "1" : "0"         , true},
            { root + "trigger/edge_backoff"   , std::to_string(edge_debounce)   , true},
            { root + "trigger/edge_sample_num", std::to_string(samples)         , true},
            { root + "trigger/gating"         , gating                          , true},
            { "/mboards/0/trigger/sma_dir"    , "in"                            , true},
            { "/mboards/0/trigger/sma_pol"    , "positive"                      , true},
            { dsp_root + "rstreq"             , 1.0                             , false},
        };
        return sets;
    }

    void apply(const std::vector<Set> sets) const
    {
        for(const auto &set : sets)
        {
            if(set.use_double) {
                usrp->set_tree_value(set.path, set.double_value);
                double actual_value;
                usrp->get_tree_value(set.path, actual_value);
                std::cout << set.path << "=" << actual_value << std::endl;

                if(set.verify && set.double_value != actual_value) {
                    UHD_LOG_ERROR("TEST_TX_TRIGGER", "Desired value " + std::to_string(set.double_value) + " does not equal the actual value " + std::to_string(actual_value) + " for path: " + set.path);
                    assert(set.double_value == actual_value);
                }

            } else {
                usrp->set_tree_value(set.path, set.string_value);
                std::string actual_value;
                usrp->get_tree_value(set.path, actual_value);

                if (set.verify && set.string_value != actual_value) {
                    UHD_LOG_ERROR("TEST_TX_TRIGGER", "Desired value " + set.string_value + " does not equal the actual value " + actual_value + " for path: " + set.path);
                    assert(set.string_value == actual_value);
                }
            }
            set.print();
        }
    }
};

class Uhd
{
public:
    uhd::usrp::multi_usrp::sptr usrp;

    Uhd(const std::vector<size_t> channels, const double tx_rate, const double tx_center_freq, const double tx_gain)
    {

        for(const auto ch : channels)
        {
            usrp = uhd::usrp::multi_usrp::make(std::string(""));
            usrp->set_clock_source("internal");
            usrp->set_tx_rate(tx_rate, ch);
            usrp->set_tx_freq(uhd::tune_request_t(tx_center_freq), ch);
            usrp->set_tx_gain(tx_gain, ch);
        }
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }
};

class Buffer
{
    const std::vector<size_t> channels;
    const bool iq_swap;

public:
    std::vector<std::complex<float> > buffer;

    std::vector<std::complex<float>*> mirrors;

    Buffer(const std::vector<size_t> channels, const std::string path, const bool iq_swap)
    :
    channels {channels},
    iq_swap {iq_swap}
    {
        load(path);
        mirror();
    }

    //
    // Mirrors are of equal size.
    //

    int size() const
    {
        return buffer.size();
    }

private:
    void load(const std::string path)
    {
        std::ifstream file(path);

        if(file.fail())
        {
            std::cout
                << "File '" << path << "' not found..."
                << std::endl
                << "Create this file with one column of floating point data within range [-1.0, 1.0]"
                << std::endl
                << "IMPORTANT: This signal will be applied to all channels."
                << std::endl;

            std::exit(1);
        }

        for(std::string line; std::getline(file, line);)
        {
            std::stringstream stream(line);

            float val = 0.0f;
            stream >> val;

            if(iq_swap) {
                buffer.push_back(std::complex<float>(0.0f, val));
            } else {
                buffer.push_back(val);
            }
        }
    }

    void mirror()
    {
        for(const auto ch : channels)
        {
            (void) ch;
            mirrors.push_back(&buffer.front());
        }
    }
};

class Streamer
{
    uhd::tx_streamer::sptr tx;
    uhd::usrp::multi_usrp::sptr usrp;

    const std::vector<size_t> channels;

public:
    Streamer(uhd::usrp::multi_usrp::sptr usrp, const std::vector<size_t> channels, uint64_t setpoint)
    :
    channels {channels}
    {
        uhd::stream_args_t stream_args("fc32", "sc16");
        stream_args.channels = channels;
        this->usrp = usrp;
        tx = usrp->get_tx_stream(stream_args);
        // Switches buffer management to blocking mode
        // The default mode attempts to predict the buffer level, which is impossible in trigger mode
        tx->enable_blocking_fc(setpoint);
    }

    void stream(const Buffer buffer, const double start_time, const double period, const uint64_t samples, const uint64_t num_trigger) const
    {
        //
        // Prime the FPGA FIFO buffer.
        //
        
        uhd::tx_metadata_t md;
        md.end_of_burst = false;
        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(start_time);
        double timeout = start_time + 1.5;


        //
        // Transmission will start at <start_time>.
        //
        // Fuzzy flow control begins now. Hit Enter or Ctrl-C to exit and cleanup.
        //
        uint64_t total_sent = 0;

        Exit::wait_interrupt();

        for(uint64_t pulses_sent = 0; ((pulses_sent < num_trigger || num_trigger == 0) && !Exit::get()); pulses_sent++)
        {
            std::vector<int> levels;

            for(const auto ch : channels)
                levels.push_back(Fifo(usrp, ch).get_level());

            const int max = *std::max_element(std::begin(levels), std::end(levels));
            const int min = *std::min_element(std::begin(levels), std::end(levels));

            //
            // The fuzzy bit.
            //

            md.start_of_burst = true;
            md.end_of_burst = false;
            uint64_t sent = tx->send(buffer.mirrors, samples, md, timeout);
            if(sent != samples) {
                throw uhd::runtime_error("timeout, the time between triggers is to long");
            }
            total_sent = sent + total_sent;
            md.has_time_spec = false;

            //send a mini EOB packet
            md.start_of_burst = false;
            md.end_of_burst = true;
            tx->send("", 0, md);

            //
            // Print FIFO levels.
            //
            
            for(const auto level : levels)
                std::cout << level << "\t";
            std::cout << max - min << std::endl;

            //
            // Loop control rate must be faster than SMA trigger rate.
            //

            usleep(1.0e6 / period);
        }   

        md.end_of_burst = true;
        tx->send("", 0, md);
    }

    size_t get_max_num_samps() const
    {
        return tx->get_max_num_samps();
    }
};

class Args
{
public:
    double start_time {0.0};

    double period {0.0};

    double tx_rate {0.0};
    
    double tx_center_freq {0.0};

    double tx_gain {0.0};

    uint64_t num_trigger {0};

    int setpoint {0};

    uint64_t samples {0};

    std::string path;

    std::string gating;

    int edge_debounce {0};

    std::vector<size_t> channels;

    bool iq_swap;

    Args(int argc, char* argv[])
    {
        std::string channel_list;

        namespace po = boost::program_options;

        po::options_description description("Command line options");

        description.add_options()
            ("help", "This help screen")
            ("start_time"    , po::value<double     >(&start_time    )->default_value(       5.0), "(Seconds) Transmitter will enable after this many seconds")
            ("period"        , po::value<double     >(&period        )->default_value(      20.0), "(Hz     ) Closed loop control frequency updates at this rate")
            ("tx_rate"       , po::value<double     >(&tx_rate       )->default_value(     100e3), "(Hz     ) Transmitter sample rate")
            ("tx_center_freq", po::value<double     >(&tx_center_freq)->default_value(     123e6), "(Hz     ) Transmitter center frequency")
            ("tx_gain"       , po::value<double     >(&tx_gain       )->default_value(      10.0), "(Scalar ) Transmitter gain")
            ("num_trigger"   , po::value<uint64_t   >(&num_trigger   )->default_value(         0), "(Scalar ) Number of trigger event results to record. Set to 0 for continuous")
            ("setpoint"      , po::value<int        >(&setpoint      )->default_value(      5000), "(Samples) Closed loop control will maintain this sample count as the setpoint")
            ("samples"       , po::value<uint64_t        >(&samples       )->default_value(       400), "(Samples) Number of samples to send per trigger event")
            ("path"          , po::value<std::string>(&path          )->default_value("data.txt"), "(Path   ) File path of single column floating point data (Just I, not Q) in range [-1.0, 1.0] to be applied to all device channels")
            ("gating"        , po::value<std::string>(&gating        )->default_value(     "dsp"), "(String ) Gating mode [\"dsp\" | \"output\"]")
            ("edge_debounce" , po::value<int        >(&edge_debounce )->default_value(         0), "(Samples) Number of samples to ignore after first trigger (for debouncing)")
            ("channels", po::value<std::string>(&channel_list)->default_value("0,1,2,3"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
            ("iq_swap", "Swap i and q in that data packets being send. Useful for debugging purposes only")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, description), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << description << std::endl;
            std::exit(1);
        }

        iq_swap = vm.count("iq_swap");

        std::vector<std::string> channel_strings;
        boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
        for(size_t ch = 0; ch < channel_strings.size(); ch++){
            size_t chan = std::stoi(channel_strings[ch]);
            channels.push_back(chan);
        }
        
    }
};

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    Args args(argc, argv);

    Uhd uhd(args.channels, args.tx_rate, args.tx_center_freq, args.tx_gain);

    Streamer streamer(uhd.usrp, args.channels, (uint64_t) args.setpoint);

    Buffer buffer(args.channels, args.path, args.iq_swap);

    //
    // Trigger class will destruct and cleanup SMA settings on Exit signal.
    //

    Trigger trigger(uhd.usrp, args.channels, args.samples, args.edge_debounce, args.gating);

    streamer.stream(buffer, args.start_time, args.period, args.samples, args.num_trigger);

    return 0;
}
