//
// Copyright 2018 Per Vices Corporation (GPL-3.0)
//

#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#undef NDEBUG
#include <cassert>

class Trigger
{
private:
    const std::vector<size_t> channels;
    const uhd::usrp::multi_usrp::sptr usrp;

public:
    Trigger(uhd::usrp::multi_usrp::sptr& usrp, const std::vector<size_t>& channels, const int samples)
    :
    channels {channels},
    usrp {usrp}
    {
        for(const auto& ch : channels)
            apply(sma(ch, samples));
    }

    ~Trigger()
    {
        for(const auto& ch : channels)
            apply(sma(ch, 0));
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

    std::vector<Set> sma(const size_t channel, const int samples) const
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
        std::cout << std::endl;
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

    Uhd(const std::vector<size_t>& channels)
    {
        for(const auto& ch : channels)
        {
            usrp = uhd::usrp::multi_usrp::make(std::string(""));
            usrp->set_clock_source("internal");
            usrp->set_tx_rate(25e6, ch);
            usrp->set_tx_freq(uhd::tune_request_t(0.0), ch);
            usrp->set_tx_gain(10.0, ch);
        }
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }
};

class Buffer
{
public:
    std::vector<std::complex<float>> buffer;
    std::vector<std::complex<float>*> mirrors;
    const size_t size;

    void sin()
    {
        const constexpr double pi = acos(-1.0);

        // Frequency per packet. Not overall wave frequency.
        const double freq = 100.0;
        const double ampl = 0.5;

        for(size_t n = 0; n < size; n++)
            buffer[n] = ampl * std::sin(2.0 * pi * freq * n / size);
    }

    void mirror(const std::vector<size_t>& channels)
    {
        for(const auto& ch : channels)
        {
            (void) ch;
            mirrors.push_back(&buffer.front());
        }
    }

    Buffer(const std::vector<size_t>& channels, const size_t size)
    :
    buffer(size),
    size {size}
    {
        sin();
        mirror(channels);
    }
};

class Streamer
{
private:
    uhd::tx_streamer::sptr tx;

public:
    Streamer(uhd::usrp::multi_usrp::sptr usrp, const std::vector<size_t>& channels)
    {
        uhd::stream_args_t stream_args("fc32", "sc16");
        stream_args.channels = channels;
        tx = usrp->get_tx_stream(stream_args);
    }

    void stream(Buffer buffer, const float start_time, const size_t packets) const
    {
        uhd::tx_metadata_t md;

        md.start_of_burst = true;
        md.end_of_burst = false;
        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(start_time);

        for(size_t i = 0; i < packets; i++)
        {
            tx->send(buffer.mirrors, buffer.size, md);
            md.start_of_burst = false;
            md.has_time_spec = false;
        }

        sleep(start_time + 60);

        md.end_of_burst = true;
        tx->send("", 0, md);
    }

    size_t get_max_num_samps() const
    {
        return tx->get_max_num_samps();
    }
};

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    const std::vector<size_t> channels = { 0, 1, 2, 3 };

    // Number of packets with which to fill internal FPGA buffer.
    const int packets = 5;

    // Number of samples with which to transmit per trigger event.
    const int samples = 200;

    uhd::set_thread_priority_safe();

    Uhd uhd(channels);

    Streamer streamer(uhd.usrp, channels);

    const Buffer buffer(channels, streamer.get_max_num_samps());

    Trigger trig(uhd.usrp, channels, samples);

    streamer.stream(buffer, 10.0, packets);

    return 0;
}
