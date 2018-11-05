//
// Copyright 2018 Per Vices Corporation (GPL-3.0)
//

#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#undef NDEBUG
#include <cassert>

class Trigger
{
private:
    const int channel;
    uhd::usrp::multi_usrp::sptr usrp;

public:
    Trigger(uhd::usrp::multi_usrp::sptr& usrp, const int channel, const int samples)
    :
    channel{channel},
    usrp{usrp}
    {
        apply(sma(samples));
    }

    ~Trigger()
    {
        apply(sma(0));
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

    std::vector<Set> sma(const int samples) const
    {
        const std::string root { "/mboards/0/tx/" + std::to_string(channel) + "/" };
        const std::vector<Set> sets {
            { root + "trigger/sma_mode"       , "edge"                  },
            { root + "trigger/trig_sel"       , samples > 0 ? "1" : "0" },
            { root + "trigger/edge_backoff"   , "0"                     },
            { root + "trigger/edge_sample_num", std::to_string(samples) },
            { root + "trigger/gating"         , "dsp"                   },
            { "/mboards/0/trigger/sma_dir"    , "in"                    },
            { "/mboards/0/trigger/sma_pol"    , "positive"              },
        };
        return sets;
    }

    void apply(const std::vector<Set>& sets) const
    {
        set(sets);
        check(sets);
    }

    void set(const std::vector<Set>& sets) const
    {
        for(const auto& set : sets)
        {
            usrp->set_tree_value(set.path, set.value);
            set.print();
        }
    }

    void check(const std::vector<Set>& sets) const
    {
        for(const auto& set : sets)
        {
            std::string value;
            usrp->get_tree_value(set.path, value);
            assert(value == set.value);
        }
    }
};

class Uhd
{
public:
    uhd::usrp::multi_usrp::sptr usrp;

    Uhd()
    {
        usrp = uhd::usrp::multi_usrp::make(std::string(""));
        usrp->set_clock_source("internal");
        usrp->set_tx_rate(25e6);
        usrp->set_tx_freq(uhd::tune_request_t(0.0));
        usrp->set_tx_gain(10.0);
    }
};

class Streamer
{
public:
    uhd::tx_streamer::sptr tx;

    Streamer(uhd::usrp::multi_usrp::sptr usrp, const size_t channel)
    {
        uhd::stream_args_t stream_args("fc32", "sc16");
        stream_args.channels = { channel };
        tx = usrp->get_tx_stream(stream_args);
    }

    void stream(std::vector<std::complex<float>> values, const size_t packets)
    {
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst = false;
        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(2.0);

        for(size_t i = 0; i < packets; i++)
        {
            tx->send(&values.front(), values.size(), md);
            md.start_of_burst = false;
            md.has_time_spec = false;
        }
        std::cout << "Press any key to stop streaming" << std::endl;
        std::cin.get();

        md.end_of_burst = true;
        tx->send("", 0, md);
    }
};

class Buffer
{
public:
    std::vector<std::complex<float>> values;

    Buffer(const int size)
    :
    values(size)
    {
        for(int n = 0; n < size; n++)
            values[n] = 0.5 * std::sin(2.0 * 3.1416 * 100.0 * n / size);
    }
};

class Args
{
public:
    size_t channel;
    size_t packets;

    Args(int argc, char* argv[])
    {
        if(argc != 3)
        {
            std::cout << "sudo ./test_tx_trigger channel packets" << std::endl;
            std::exit(1);
        }
        channel = std::stoi(argv[1]);
        packets = std::stoi(argv[2]);
    }
};

int main(int argc, char* argv[])
{
    uhd::set_thread_priority_safe();

    const Args args(argc, argv);

    Uhd uhd;

    Streamer streamer(uhd.usrp, args.channel);

    const Buffer buffer(streamer.tx->get_max_num_samps());

    const Trigger trig(uhd.usrp, args.channel, 100);

    streamer.stream(buffer.values, args.packets);
}
