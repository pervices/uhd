#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>

namespace
{
    const std::vector<std::string> channels = { "0", "1", "2", "3" };

    void test_strings(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        for(const auto& channel : channels)
        {
            const std::string path = "/mboards/0/rx/" + channel + "/pwr";

            std::string value;

            usrp->get_tree_value(path, value);
            usrp->set_tree_value(path, value);
            std::cout << value << std::endl;
        }
    }

    void test_ints(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        const std::string path = "/mboards/0/blink";

        int value;

        usrp->get_tree_value(path, value);
        usrp->set_tree_value(path, value);
        std::cout << value << std::endl;
    }

    void test_doubles(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        const std::string path = "/mboards/0/link_max_rate";

        double value;

        usrp->get_tree_value(path, value);
        usrp->set_tree_value(path, value);

        std::cout << value << std::endl;
    }

    void test_bools(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        const std::string path = "/mboards/0/clock_source/output";

        bool value;

        usrp->get_tree_value(path, value);
        usrp->set_tree_value(path, value);

        std::cout << value << std::endl;
    }

    void test_stream_cmd(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        for(const auto& channel : channels)
        {
            uhd::stream_cmd_t value(
                uhd::stream_cmd_t::stream_mode_t::STREAM_MODE_NUM_SAMPS_AND_DONE);

            const std::string path = "/mboards/0/rx_dsps/" + channel + "/stream_cmd";
            usrp->get_tree_value(path, value);
            usrp->set_tree_value(path, value);
        }
    }

    void test_time_specs(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;
        const std::string path = "/mboards/0/time/now";

        uhd::time_spec_t value;

        usrp->get_tree_value(path, value);
        usrp->set_tree_value(path, value);

        std::cout << value.get_real_secs() << std::endl;
    }

    void test_sfpa_port_change(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;

        const std::string path = "/mboards/0/fpga/board/flow_control/sfpa_port";

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

        // Reapply the old value.
        usrp->set_tree_value(path, old);

        std::cout << changed << std::endl;
        std::cout << old << std::endl;
    }

    void test_trigger_settings(uhd::usrp::multi_usrp::sptr& usrp)
    {
        std::cout << __FUNCTION__ << std::endl;

        // FPGA values.
        const std::vector<std::string> paths = {
            "/mboards/0/sfpa/ip_addr",
            "/mboards/0/sfpa/mac_addr",
            "/mboards/0/sfpa/pay_len",
            "/mboards/0/sfpb/ip_addr",
            "/mboards/0/sfpb/mac_addr",
            "/mboards/0/sfpb/pay_len",
            "/mboards/0/trigger/sma_dir",
            "/mboards/0/trigger/sma_pol",
        };
        for(const auto& path : paths)
        {
            std::string old;
            usrp->get_tree_value(path, old);
            usrp->set_tree_value(path, old);
            std::cout << old << std::endl;
        }
        // TX Paths.
        for(const auto& channel : channels)
        {
            const std::vector<std::string> paths = {
                "/mboards/0/tx/" + channel + "/trigger/sma_mode",
                "/mboards/0/tx/" + channel + "/trigger/trig_sel",
                "/mboards/0/tx/" + channel + "/trigger/edge_backoff",
                "/mboards/0/tx/" + channel + "/trigger/edge_sample_num",
                "/mboards/0/tx/" + channel + "/trigger/ufl_mode",
                "/mboards/0/tx/" + channel + "/trigger/ufl_dir",
                "/mboards/0/tx/" + channel + "/trigger/ufl_pol",
                "/mboards/0/tx/" + channel + "/trigger/gating",
            };
            for(const auto& path : paths)
            {
                std::string old;
                usrp->get_tree_value(path, old);
                usrp->set_tree_value(path, old);
                std::cout << old << std::endl;
            }
        }
        // RX Paths.
        for(const auto& channel : channels)
        {
            const std::vector<std::string> paths = {
                "/mboards/0/rx/" + channel + "/trigger/sma_mode",
                "/mboards/0/rx/" + channel + "/trigger/trig_sel",
                "/mboards/0/rx/" + channel + "/trigger/edge_backoff",
                "/mboards/0/rx/" + channel + "/trigger/edge_sample_num",
                "/mboards/0/rx/" + channel + "/trigger/ufl_mode",
                "/mboards/0/rx/" + channel + "/trigger/ufl_dir",
                "/mboards/0/rx/" + channel + "/trigger/ufl_pol",
            };
            for(const auto& path : paths)
            {
                std::string old;
                usrp->get_tree_value(path, old);
                usrp->set_tree_value(path, old);
                std::cout << old << std::endl;
            }
        }
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
    test_trigger_settings(usrp);

    return 0;
}
