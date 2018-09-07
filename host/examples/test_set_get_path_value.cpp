#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>

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
            usrp->get_tree_value(path, value);
            std::cout << value << std::endl;
            usrp->set_tree_value(path, value);
        }
    }

    void test_ints(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/blink";

        int value;

        usrp->get_tree_value(path, value);
        std::cout << value << std::endl;
        usrp->set_tree_value(path, value);
    }

    void test_doubles(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/link_max_rate";

        double value;

        usrp->get_tree_value(path, value);
        std::cout << value << std::endl;
        usrp->set_tree_value(path, value);
    }

    void test_bools(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/clock_source/output";

        bool value;

        usrp->get_tree_value(path, value);
        std::cout << value << std::endl;
        usrp->set_tree_value(path, value);
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
            usrp->get_tree_value(path, value);
            usrp->set_tree_value(path, value);
        }
    }

    void test_time_specs(uhd::usrp::multi_usrp::sptr& usrp)
    {
        usrp->set_time_now(uhd::time_spec_t(0.0));

        std::cout << __FUNCTION__ << std::endl;
        std::string path = "/mboards/0/time/now";

        uhd::time_spec_t value;

        usrp->get_tree_value(path, value);
        std::cout << value.get_real_secs() << std::endl;
        usrp->set_tree_value(path, value);
    }

    void test_sfpa_port_change(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::string path = "/mboards/0/fpga/board/flow_control/sfpa_port";

        const int expected = 12345;

        // Get old value.
        int old;
        usrp->get_tree_value(path, old);

        // Set new value.
        usrp->set_tree_value(path, expected);

        // Get the newly changed value.
        int changed;
        usrp->get_tree_value(path, changed);

        // Ensure the newly changed value is the expected value.
        assert(changed == expected);
    }
}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    usrp->dump_tree("");

    test_strings(usrp);
    test_ints(usrp);
    test_doubles(usrp);
    test_bools(usrp);
    test_stream_cmd(usrp);
    test_time_specs(usrp);
    test_sfpa_port_change(usrp);

    return 0;
}
