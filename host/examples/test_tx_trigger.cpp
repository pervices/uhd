//
// Copyright 2018 Per Vices Corporation (GPL-3.0)
//

#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>

namespace trigger
{
    struct Setting
    {
        std::string path;
        std::string value;
    };

    void setup(uhd::usrp::multi_usrp::sptr& usrp, const int channel, const int samples)
    {
        const std::string root = "/mboards/0/tx/" + std::to_string(channel) + "/";

        const std::vector<Setting> settings = {
            { root + "trigger/sma_mode"       , "edge"                  },
            { root + "trigger/trig_sel"       , "1"                     },
            { root + "trigger/edge_backoff"   , "0"                     },
            { root + "trigger/edge_sample_num", std::to_string(samples) },
            { root + "trigger/gating"         , "dsp"                   },
            { "/mboards/0/trigger/sma_dir"    , "in"                    },
            { "/mboards/0/trigger/sma_pol"    , "positive"              },
        };
        for(const auto& setting : settings)
        {
            // Set.
            std::cout << setting.path << " = " << setting.value << std::endl;
            usrp->set_tree_value(setting.path, setting.value);

            // Get and check.
            std::string get;
            usrp->get_tree_value(setting.path, get);
            if(setting.value != get)
                std::cout << setting.path << " set/get mismatch" << std::endl;
        }
    }
}

int main()
{
    const int channel = 0;

    uhd::set_thread_priority_safe();

    // Setup USRP.
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));
    usrp->set_clock_source("internal");
    usrp->set_tx_rate(25e6);
    usrp->set_tx_freq(uhd::tune_request_t(0.0));
    usrp->set_tx_gain(10.0);

    // Setup TX streamer.
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = { channel };
    uhd::tx_streamer::sptr tx = usrp->get_tx_stream(stream_args);

    // Setup transfer buffer.
    const int packet_size = tx->get_max_num_samps();
    std::vector<std::complex<float>> buffer(packet_size);
    for(int n = 0; n < packet_size; n++)
        buffer[n] = 0.5;

    // Setup TX trigger.
    trigger::setup(usrp, channel, 100);

    // Setup start of burst.
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = uhd::time_spec_t(2.0);

    const int packets = 3;
    for(int i = 0; i < packets; i++)
    {
        // Send.
        tx->send(&buffer.front(), packet_size, md);

        // Subsequent sends are middle of bursts.
        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    // Wait here.
    std::getchar();

    // Setup end of burst and send.
    md.end_of_burst = true;
    tx->send("", 0, md);
}
