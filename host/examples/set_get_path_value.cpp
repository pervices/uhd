#include <uhd/usrp/multi_usrp.hpp>

namespace
{
    void test_strings(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        for(const auto& channel : std::vector<std::string> { "0", "1", "2", "3" })
        {
            std::string base = "/mboards/0/rx/";
            std::string field = "/pwr";

            std::string value;

            std::string path = base + channel + field;
            usrp->get(path, value);
            std::cout << value << std::endl;
            usrp->set(path, value);
        }
    }

    void test_ints(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/blink";

        int value;

        usrp->get(path, value);
        std::cout << value << std::endl;
        usrp->set(path, value);
    }

    void test_doubles(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/link_max_rate";

        double value;

        usrp->get(path, value);
        std::cout << value << std::endl;
        usrp->set(path, value);
    }

    void test_bools(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/clock_source/output";

        bool value;

        usrp->get(path, value);
        std::cout << value << std::endl;
        usrp->set(path, value);
    }

    void test_stream_cmd(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        for(const auto& channel : std::vector<std::string> { "0", "1", "2", "3" })
        {
            std::string base = "/mboards/0/rx_dsps/";
            std::string field = "/stream_cmd";

            uhd::stream_cmd_t value(uhd::stream_cmd_t::stream_mode_t::STREAM_MODE_NUM_SAMPS_AND_DONE);

            std::string path = base + channel + field;
            usrp->get(path, value);
            usrp->set(path, value);
        }
    }

    void test_time_specs(uhd::usrp::multi_usrp::sptr& usrp)
    {
        usrp->set_time_now(uhd::time_spec_t(0.0));

        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/time/now";

        uhd::time_spec_t value;

        usrp->get(path, value);
        std::cout << value.get_real_secs() << std::endl;
        usrp->set(path, value);
    }
}

int main()
{
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    usrp->dump();

    test_strings(usrp);
    test_ints(usrp);
    test_doubles(usrp);
    test_bools(usrp);
    test_stream_cmd(usrp);
    test_time_specs(usrp);
}
