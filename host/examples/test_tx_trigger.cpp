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
    const int channel {0};
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
    }

    void check(const std::vector<Set> sets) const
    {
        for(const auto set : sets)
        {
            std::string value;
            usrp->get_tree_value(set.path, value);
            assert(value == set.value);
        }
    }
};

int main()
{
    const int channel {0};

    uhd::set_thread_priority_safe();

    // Setup USRP.
    auto usrp = uhd::usrp::multi_usrp::make(std::string(""));
    usrp->set_clock_source("internal");
    usrp->set_tx_rate(25e6);
    usrp->set_tx_freq(uhd::tune_request_t(0.0));
    usrp->set_tx_gain(10.0);

    // Setup TX streamer.
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = { channel };
    const uhd::tx_streamer::sptr tx { usrp->get_tx_stream(stream_args) };

    // Setup transfer buffer.
    const int packet_size = tx->get_max_num_samps();
    std::vector<std::complex<float>> buffer(packet_size);
    for(int n = 0; n < packet_size; n++)
        buffer[n] = 0.5 * std::sin(2.0 * 3.1416 * 100.0 * n / packet_size);

    // Trigger disables at end of test with destructor.
    const Trigger trig(usrp, channel, 100);

    // Setup start of burst.
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = uhd::time_spec_t(2.0);

    const int packets = 5;
    for(int i = 0; i < packets; i++)
    {
        // Send packet.
        tx->send(&buffer.front(), packet_size, md);

        // Setup subsequent packets to be middle of burst packets.
        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    // Wait for user input.
    std::getchar();

    // Setup end of burst with empty data and send.
    md.end_of_burst = true;
    tx->send("", 0, md);
}
