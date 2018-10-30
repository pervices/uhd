//
// Copyright 2018 Per Vices Corporation (GPL-3.0)
//

#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>

int main()
{
    uhd::set_thread_priority_safe();

    // Setup USRP.
    std::cout << "USRP..." << std::endl;
    uhd::usrp::multi_usrp::sptr usrp =
        uhd::usrp::multi_usrp::make(std::string(""));

    usrp->set_clock_source("internal");
    usrp->set_tx_rate(25e6);
    usrp->set_tx_freq(uhd::tune_request_t(0.0));
    usrp->set_tx_gain(10.0);

    // Setup TX streamer.
    std::cout << "TX Streamer..." << std::endl;
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = { 0 };
    uhd::tx_streamer::sptr tx = usrp->get_tx_stream(stream_args);

    // Setup transfer buffer.
    std::cout << "Buffer..." << std::endl;
    const int samples = tx->get_max_num_samps();
    std::cout << samples << std::endl;
    std::vector<std::complex<float>> buffer(samples);
    for(int n = 0; n < samples; n++)
    {
        const double amplitude = 0.5;
        const double pi = acos(-1.0);
        const double freq = 1e6;
        buffer[n] = amplitude * std::sin(2 * pi * freq * n / samples);
    }

    // Setup start of burst.
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = uhd::time_spec_t(1.0);

    std::cout << "Sending..." << std::endl;
    for(int i =  0; true; /*i < 10*/i++)
    {
        // Send.
        tx->send(&buffer.front(), samples, md);

        // Setup subsequent sends as middle of bursts.
        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    // Setup end of burst and send.
    md.end_of_burst = true;
    tx->send("", 0, md);
}
