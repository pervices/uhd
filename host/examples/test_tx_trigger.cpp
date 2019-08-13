//
// Copyright 2018 - 2019 Per Vices Corporation GPL 3
//

#include <boost/program_options.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/endian/conversion.hpp>

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

    void set()
    {
        std::lock_guard<std::mutex> guard(mutex);
        now = true;
    }

    void interrupt(int)
    {
        std::lock_guard<std::mutex> guard(mutex);
        now = true;
        std::cout << "\nInterrupt caught: Hit Enter for Crimson cleanup" << std::endl;
    }

    bool get()
    {
        std::lock_guard<std::mutex> guard(mutex);
        return now;
    }

    //
    // Ctrl-C and / or Enter will exit this program mid stream.
    //

    void wait()
    {
        signal(SIGINT, interrupt);
        std::getchar();
        set();
    }
}

class Fifo
{
    uhd::transport::udp_simple::sptr link;

    const size_t channel {0};

public:
    Fifo(const size_t channel)
    :
    channel {channel}
    {
        const std::string port = channel % 2 ? "10.10.11.2" : "10.10.10.2";
        const std::string ip = "42809";
        link = uhd::transport::udp_simple::make_connected(port, ip);
    }

    uint16_t get_level()
    {
        poke();
        return peek().header & 0xFFFF;
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
        const std::string value;

        void print() const
        {
            std::cout << path << " = " << value << std::endl;
        }
    };

    std::vector<Set> sma(const size_t channel, const int samples, const int edge_debounce) const
    {
        const std::string root { "/mboards/0/tx/" + std::to_string(channel) + "/" };
        const std::string dsp_root { "/mboards/0/tx_dsps/" + std::to_string(channel) + "/" };
        const std::vector<Set> sets {
            { root + "trigger/sma_mode"       , "edge"                          },
            { root + "trigger/trig_sel"       , samples > 0 ? "1" : "0"         },
            { root + "trigger/edge_backoff"   , std::to_string(edge_debounce)   },
            { root + "trigger/edge_sample_num", std::to_string(samples)         },
            { root + "trigger/gating"         , gating                          },
            { "/mboards/0/trigger/sma_dir"    , "in"                            },
            { "/mboards/0/trigger/sma_pol"    , "positive"                      },
            { dsp_root + "rstreq"             , "1"                             },
        };
        return sets;
    }

    void apply(const std::vector<Set> sets) const
    {
        set(sets);
        check(sets);
    }

    void set(const std::vector<Set> sets) const
    {
        for(const auto set : sets)
        {
            usrp->set_tree_value(set.path, set.value);
            set.print();
        }
        std::cout << std::endl;
    }

    void check(const std::vector<Set> sets) const
    {
        for(const auto set : sets)
        {
            std::string value;
            // skip checking rstreq, since it is self-clearing
            if (set.path.find("rstreq") == std::string::npos ) {
                usrp->get_tree_value(set.path, value);
                assert(value == set.value);
            }

        }
    }
};

class Uhd
{
public:
    uhd::usrp::multi_usrp::sptr usrp;

    Uhd(const std::vector<size_t> channels, const double tx_rate, const double tx_center_freq, const double tx_gain)
    {
        uhd::set_thread_priority_safe();

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

public:
    std::vector<std::complex<float> > buffer;

    std::vector<std::complex<float>*> mirrors;

    Buffer(const std::vector<size_t> channels, const std::string path)
    :
    channels {channels}
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

            buffer.push_back(val);
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

    const std::vector<size_t> channels;

public:
    Streamer(uhd::usrp::multi_usrp::sptr usrp, const std::vector<size_t> channels)
    :
    channels {channels}
    {
        uhd::stream_args_t stream_args("fc32", "sc16");
        stream_args.channels = channels;
        tx = usrp->get_tx_stream(stream_args);
    }

    void stream(const Buffer buffer, const double start_time, const int setpoint, const double period) const
    {
        //
        // Prime the FPGA FIFO buffer.
        //
        
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst = false;
        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(start_time);

        //
        // Transmission will start at <start_time>.
        //
        // Fuzzy flow control begins now. Hit Enter or Ctrl-C to exit and cleanup.
        //

        std::thread thread(Exit::wait);
        while(!Exit::get())
        {
            std::vector<int> levels;

            for(const auto ch : channels)
                levels.push_back(Fifo(ch).get_level());

            const int max = *std::max_element(std::begin(levels), std::end(levels));
            const int min = *std::min_element(std::begin(levels), std::end(levels));

            //
            // The fuzzy bit.
            //

            if(min < setpoint)
            {
                tx->send(buffer.mirrors, buffer.size(), md);
                md.start_of_burst = false;
                md.has_time_spec = false;
            }

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
        thread.join();

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

    int setpoint {0};

    int samples {0};

    std::string path;

    std::string gating;

    int edge_debounce {0};

    //
    // Channels not to be set from command line.
    //

    const std::vector<size_t> channels = { 0, 1, 2, 3 };

    Args(int argc, char* argv[])
    {
        namespace po = boost::program_options;

        po::options_description description("Command line options");

        description.add_options()
            ("help", "This help screen")
            ("start_time"    , po::value<double     >(&start_time    )->default_value(       5.0), "(Seconds) Transmitter will enable after this many seconds")
            ("period"        , po::value<double     >(&period        )->default_value(      20.0), "(Hz     ) Closed loop control frequency updates at this rate")
            ("tx_rate"       , po::value<double     >(&tx_rate       )->default_value(     100e3), "(Hz     ) Transmitter sample rate")
            ("tx_center_freq", po::value<double     >(&tx_center_freq)->default_value(     123e6), "(Hz     ) Transmitter center frequency")
            ("tx_gain"       , po::value<double     >(&tx_gain       )->default_value(      10.0), "(Scalar ) Transmitter gain")
            ("setpoint"      , po::value<int        >(&setpoint      )->default_value(      5000), "(Samples) Closed loop control will maintain this sample count as the setpoint")
            ("samples"       , po::value<int        >(&samples       )->default_value(       400), "(Samples) Number of samples to send per trigger event")
            ("path"          , po::value<std::string>(&path          )->default_value("data.txt"), "(Path   ) File path of single column floating point data (Just I, not Q) in range [-1.0, 1.0] to be applied to all device channels")
            ("gating"        , po::value<std::string>(&gating        )->default_value(     "dsp"), "(String ) Gating mode [\"dsp\" | \"output\"]")
            ("edge_debounce" , po::value<int        >(&edge_debounce )->default_value(         0), "(Samples) Number of samples to ignore after first trigger (for debouncing)")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, description), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << description << std::endl;
            std::exit(1);
        }
    }
};

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    Args args(argc, argv);

    Uhd uhd(args.channels, args.tx_rate, args.tx_center_freq, args.tx_gain);

    Streamer streamer(uhd.usrp, args.channels);

    Buffer buffer(args.channels, args.path);

    //
    // Trigger class will destruct and cleanup SMA settings on Exit signal.
    //

    Trigger trigger(uhd.usrp, args.channels, args.samples, args.edge_debounce, args.gating);

    streamer.stream(buffer, args.start_time, args.setpoint, args.period);

    return 0;
}
