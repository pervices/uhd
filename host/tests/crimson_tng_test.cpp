#include <boost/test/unit_test.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>

//#include <iomanip>
#include <unordered_map>

#include <uhd/device.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/static.hpp>

#include "../lib/usrp/crimson_tng/crimson_tng_fw_common.h"
#include "../lib/usrp/crimson_tng/crimson_tng_impl.hpp"

constexpr size_t NUM_RX_CHANNELS = 4;
constexpr size_t NUM_TX_CHANNELS = 4;
constexpr size_t CLOCK_RATE = 325000000;
constexpr size_t CLOCK_RATE_STEPS = (1ULL << 16);
constexpr double FREQ_MAX = 6000000000;

extern void tng_csv_parse(std::vector<std::string> &tokens, char* data, const char delim);

class fake_server {
public:

    enum {
        MGMT,
        FC0,
        FC1,
        CANCEL_READ,
        CANCEL_WRITE,
        NUM_FDS,
    };
    std::array<int,NUM_FDS> fd;


    fake_server()
    : fd({-1, -1, -1})
    {
        int r;

        std::array<int,NUM_FDS> port;
        port[MGMT] = CRIMSON_TNG_FW_COMMS_UDP_PORT;

        for(size_t i = 0; i < fd.size(); ++i) {
            if (CANCEL_READ == i) {
                r = ::socketpair(AF_UNIX, SOCK_STREAM, 0, &fd[CANCEL_READ]);
                if (-1 == r) {
                    throw std::system_error(errno, std::system_category(), "socketpair(2)");
                }
                continue;
            } else if (CANCEL_WRITE == i) {
                continue;
            } else {
                r = ::socket(AF_INET, SOCK_DGRAM, 0);
                if (-1 == r) {
                    throw std::system_error(errno, std::system_category(), "socket(2)");
                }
            }

            fd[i] = r;

            ::sockaddr_in addr = {};
            ::socklen_t addrlen = sizeof(addr);
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            if (MGMT == i) {
                addr.sin_port = htons(port[MGMT]);
            } else {
                // using port 0 signals to Linux that it should bind to any available port
                // we then use getsockname() to determine the actual port we then
                // are able to put that value into the tree
                addr.sin_port = 0;
            }

            int enable = true;
            r = ::setsockopt(fd[i], SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
            if (-1 == r) {
                throw std::system_error(errno, std::system_category(), "setsockopt(2)");
            }

            r = ::bind(fd[i], (::sockaddr *)&addr, addrlen);
            if (-1 == r) {
                throw std::system_error(errno, std::system_category(), "bind(2)");
            }

            if (MGMT != i) {
                // use getsockname() to determine the actual port and put that in the tree
                r = ::getsockname(fd[i], (::sockaddr *)&addr, &addrlen);
                if (-1 == r) {
                    throw std::system_error(errno, std::system_category(), "getsockname(2)");
                }

                port[i] = ntohs(addr.sin_port);
            }
        }


        properties["fpga/about/name"] = "crimson_tng";
        properties["fpga/about/id"] = "42";
        properties["fpga/about/serial"] = "4242";
        properties["fpga/about/fw_ver"] = "424242";
        properties["fpga/about/hw_ver"] = "42424242";
        properties["fpga/about/sw_ver"] = "4242424242";

        properties["fpga/board/led"] = "1";
        properties["fpga/board/temp"] = "42";
        properties["fpga/board/flow_control/sfpa_port"] = std::to_string(port[FC0]);
        properties["fpga/board/flow_control/sfpb_port"] = std::to_string(port[FC1]);

        properties["fpga/link/sfpa/ip_addr"] = "127.0.0.1";
        properties["fpga/link/sfpa/mac_addr"] = "00:50:C2:85:3f:ff";
        properties["fpga/link/sfpa/pay_len"] = "9000";

        properties["fpga/link/sfpb/ip_addr"] = "127.0.0.1";
        properties["fpga/link/sfpb/mac_addr"] = "00:50:C2:85:3f:33";
        properties["fpga/link/sfpb/pay_len"] = "9000";

        th = std::thread(&fake_server::thread_fn, this);
    }

    ~fake_server() {
        int r = ::write(fd[CANCEL_WRITE], "x", 1);
        if (-1 == r) {
            std::cerr << "write failed: " << errno << std::endl;
        }
        for(auto &x: fd) {
            ::close(x);
            x = -1;
        }
        if (th.joinable()) {
            //std::cerr << "joining fake server thread" << std::endl;
            th.join();
        }
    }

    std::unordered_map<std::string, std::string> properties;
    std::thread th;

    void handle_mgmt() {
        constexpr size_t len = 1024;
        std::string buf(len, '\0');
        ::sockaddr_in addr;
        ::socklen_t addrlen = sizeof(addr);

        int r = ::recvfrom(fd[0], &buf.front(), buf.size(), 0, (::sockaddr *)&addr, &addrlen);
        if (-1 == r) {
            throw std::system_error(errno, std::system_category(), "recvfrom(2)");
        }

        //std::cerr << "received '" << buf << std::endl;
        std::vector<std::string> tokens;
        tng_csv_parse(tokens, &buf.front(), ',');
        if (tokens.size() < 3) {
            throw std::runtime_error("malformed query '" + std::string(buf) + "'");
        }

        std::string mid = tokens[0];
        std::string action = tokens[1];
        std::string key = tokens[2];
        std::string val;
        std::string rsp;
        const std::string success = std::string(1,CMD_SUCCESS);
        if ("get" == action) {
            val = get(key);
            rsp = mid + "," + success + "," + val;
        } else if ("set" == action) {
            if (tokens.size() < 4) {
                throw std::runtime_error("malformed query '" + std::string(buf) + "'");
            }
            set(key, val);
            val = get(key);
            rsp = mid + "," + success + "," + val;
        } else {
            throw std::runtime_error("malformed query '" + std::string(buf) + "'");
        }

        //std::cerr << "sent '" << rsp << "'" << std::endl;
        r = sendto(fd[0], rsp.c_str(), rsp.size(), 0, (::sockaddr *)&addr, ::socklen_t(sizeof(addr)));
        if (-1 == r) {
            throw std::system_error(errno, std::system_category(), "sendto(2)");
        }
    }

    void handle_fc(int fd) {

        constexpr size_t len = 3;
        int64_t buf[len];
        ::sockaddr_in addr;
        ::socklen_t addrlen = sizeof(addr);

        //std::cerr << "traffic on flow control file descriptor " << fd << std::endl;

        int r = ::recvfrom(fd, buf, sizeof(buf), 0, (::sockaddr *)&addr, &addrlen);
        if (-1 == r ) {
            throw std::system_error(errno, std::system_category(), "recvfrom(2)");
        }

        // see make_time_diff_packet()
        // 3 fields, all big-endian
        // signed 64-bit header
        // signed 64-bit seconds
        // signed 64-bit ticks

        // time diff response is simply
        // signed 64-bit seconds (difference)
        // signed 64-bit ticks (difference)
        buf[0] = 0;
        buf[1] = 0;
        r = ::sendto(fd, buf, 2 * sizeof(int64_t), 0, (::sockaddr *)&addr, addrlen);
        if (-1 == r) {
            throw std::system_error(errno, std::system_category(), "sendto(2)");
        }
    }

    void thread_fn() {
        try {
            for(;;) {

                // prepare pollfds
                std::array<::pollfd,NUM_FDS> pfd;
                for(size_t i = 0; i < fd.size(); ++i) {
                    pfd[i].fd = fd[i];
                    pfd[i].events = POLLIN;
                    pfd[i].revents = 0;
                }

                int r = ::poll(&pfd.front(), pfd.size(), 0);
                if (-1 == r) {
                    throw std::system_error(errno, std::system_category(), "poll(2)");
                }

                if (pfd[CANCEL_READ].revents & POLLIN) {
                    //std::cerr << "cancellation requested" << std::endl;
                    break;
                }

                if (pfd[MGMT].revents & POLLIN) {
                    handle_mgmt();
                }

                if (pfd[FC0].revents & POLLIN) {
                    handle_fc(fd[FC0]);
                }

                if (pfd[FC1].revents & POLLIN) {
                    handle_fc(fd[FC1]);
                }
            }
        } catch(std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    std::string get(const std::string &key) {
        /*
         * FIXME: @CF: 20200825: dirty hack
         * I didn't want to fill out all of the properties manually
         * This creates an empty string if a property doesn't exist
         */
        return properties[key];
        //return properties.at(key);
    }

    void set(const std::string &key, const std::string &val) {
        properties[key] = val;
    }
};
// it is *really* slow creating and destroying this in every test case
static fake_server fake;

class crimson_tng_mock : public uhd::usrp::crimson_tng_impl {
public:
    crimson_tng_mock(const uhd::device_addr_t &device_addr)
    : uhd::usrp::crimson_tng_impl(device_addr) {
    }
    ~crimson_tng_mock() {}
};

static uhd::device_addrs_t crimson_tng_mock_find(const uhd::device_addr_t &hint_)
{
    uhd::device_addrs_t addrs;
    if (hint_.has_key("addr") && hint_["addr"] == "127.0.0.1") {
        addrs.push_back(hint_);
    }
    return addrs;
}

static uhd::device::sptr crimson_tng_mock_make(const uhd::device_addr_t &device_addr)
{
    return uhd::device::sptr(new crimson_tng_mock(device_addr));
}

UHD_STATIC_BLOCK(register_crimson_tng_mock)
{
    uhd::device::register_device(&crimson_tng_mock_find, &crimson_tng_mock_make, uhd::device::USRP);
}

static boost::shared_ptr<uhd::usrp::multi_usrp>
get_usrp(){
    uhd::device_addr_t dev_addr;
    dev_addr["addr"] = "127.0.0.1";
    return uhd::usrp::multi_usrp::make(dev_addr);
}

//
// Test the multi_usrp interface
//

BOOST_AUTO_TEST_CASE(test_get_device){
    auto usrp = get_usrp();
    BOOST_CHECK(nullptr != usrp->get_device());
}

BOOST_AUTO_TEST_CASE(test_is_device3){
    auto usrp = get_usrp();
    BOOST_CHECK_EQUAL(false, usrp->is_device3());
}

BOOST_AUTO_TEST_CASE(test_get_device3){
    auto usrp = get_usrp();
    std::exception_ptr e = nullptr;
    boost::shared_ptr<uhd::device3> dev3 = nullptr;
    try {
        dev3 = usrp->get_device3();
    } catch(...) {
        e = std::current_exception();
    }

    BOOST_CHECK(dev3 == nullptr);
    BOOST_CHECK(e != nullptr);
}

// get_rx_stream() is expected to throw with mocked-out hw
// get_tx_stream() is expected to throw with mocked-out hw

BOOST_AUTO_TEST_CASE(test_get_rx_info){
    auto usrp = get_usrp();
    auto rx_info = usrp->get_usrp_rx_info(0);

    BOOST_CHECK(rx_info.has_key("mboard_name"));
    BOOST_CHECK_EQUAL("FPGA Board", rx_info.get("mboard_name"));

    BOOST_CHECK(rx_info.has_key("rx_subdev_name"));
    BOOST_CHECK_EQUAL("RX Board", rx_info.get("rx_subdev_name"));

    BOOST_CHECK(rx_info.has_key("rx_subdev_spec"));
    std::string sd_spec;
    for(size_t i = 0; i < NUM_RX_CHANNELS; ++i) {
        std::string letter(1, 'A' + i);
        if (i > 0) {
            sd_spec += " ";
        }
        sd_spec += letter + ":" + "Channel_" + letter;
    }
    BOOST_CHECK_EQUAL(sd_spec, rx_info.get("rx_subdev_spec"));
}

BOOST_AUTO_TEST_CASE(test_get_tx_info){
    auto usrp = get_usrp();
    auto tx_info = usrp->get_usrp_tx_info(0);

    BOOST_CHECK(tx_info.has_key("mboard_name"));
    BOOST_CHECK_EQUAL("FPGA Board", tx_info.get("mboard_name"));

    BOOST_CHECK(tx_info.has_key("tx_subdev_name"));
    BOOST_CHECK_EQUAL("TX Board", tx_info.get("tx_subdev_name"));

    BOOST_CHECK(tx_info.has_key("tx_subdev_spec"));
    std::string sd_spec;
    for(size_t i = 0; i < NUM_TX_CHANNELS; ++i) {
        std::string letter(1, 'A' + i);
        if (i > 0) {
            sd_spec += " ";
        }
        sd_spec += letter + ":" + "Channel_" + letter;
    }
    BOOST_CHECK_EQUAL(sd_spec, tx_info.get("tx_subdev_spec"));
}

/*******************************************************************
 * Mboard methods
 ******************************************************************/

BOOST_AUTO_TEST_CASE(test_set_master_clock_rate){
    auto usrp = get_usrp();
    double good_rate = CLOCK_RATE/2;
    //double bad_rate = -42;
    size_t good_mboard = 0;
    //size_t bad_mboard = -1;

    // happy path
    usrp->set_master_clock_rate(good_rate, good_mboard);
    BOOST_CHECK_EQUAL(usrp->get_master_clock_rate(good_mboard), good_rate);

    usrp->set_master_clock_rate(good_rate, uhd::usrp::multi_usrp::ALL_MBOARDS);
    BOOST_CHECK_EQUAL(usrp->get_master_clock_rate(good_mboard), good_rate);

    // should setting an unsupported rate throw?
    // usrp->set_master_clock_rate(bad_rate, good_mboard);
    // BOOST_CHECK_EQUAL(usrp->get_master_clock_rate(good_mboard), good_rate);

    // usrp->set_master_clock_rate(bad_rate, uhd::usrp::multi_usrp::ALL_MBOARDS);
    // BOOST_CHECK_EQUAL(usrp->get_master_clock_rate(good_mboard), good_rate);

    // I believe we only support 1 mboard. Should unsupported mboard throw?
    //usrp->set_master_clock_rate(good_rate, bad_mboard);
    //BOOST_CHECK_EQUAL(usrp->get_master_clock_rate(good_mboard), good_rate);
}

BOOST_AUTO_TEST_CASE(test_get_master_clock_rate){
    auto usrp = get_usrp();
    double good_rate = CLOCK_RATE/2; // add something sensible
    size_t good_mboard = 0;
    //size_t bad_mboard = -1;

    // happy path
    BOOST_CHECK_EQUAL(usrp->get_master_clock_rate(good_mboard), good_rate);

    // I believe we only support 1 mboard. Should unsupported mboard throw?
    //usrp->get_master_clock_rate(bad_mboard);
}

BOOST_AUTO_TEST_CASE(test_get_master_clock_rate_range){
    auto usrp = get_usrp();
    auto ranges = usrp->get_master_clock_rate_range(0);
    BOOST_CHECK_EQUAL(1, ranges.size());
    auto range = ranges[0];
    BOOST_CHECK_EQUAL(CLOCK_RATE/2, range.start());
    BOOST_CHECK_EQUAL(CLOCK_RATE/2, range.stop());
    BOOST_CHECK_EQUAL(0, range.step());
}

// get_pp_string()

BOOST_AUTO_TEST_CASE(test_get_mboard_name){
    auto usrp = get_usrp();
    BOOST_CHECK_EQUAL("FPGA Board", usrp->get_mboard_name(0));
}

// get_time_now()
// get_time_last_pps()  
// set_time_now()
// set_time_next_pps()
// set_time_unknown_pps()
// get_time_synchronized()
// set_command_time()
// clear_command_time()

BOOST_AUTO_TEST_CASE(test_issue_stream_cmd){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

// set_clock_config() // deprecated

BOOST_AUTO_TEST_CASE(test_set_time_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_time_source){
    auto usrp = get_usrp();
    //BOOST_CHECK_EQUAL("internal", usrp->get_clock_source(0));
    // not happy-path testing?
}

BOOST_AUTO_TEST_CASE(test_get_time_sources){
    auto usrp = get_usrp();

    // test the happy path
    auto sources = usrp->get_time_sources(0);
    BOOST_CHECK_EQUAL(2, sources.size());
    BOOST_CHECK_EQUAL("internal", sources[0]);
    BOOST_CHECK_EQUAL("external", sources[1]);

    // todo: test the non-happy path
    //sources = usrp->get_time_sources(1);
}

BOOST_AUTO_TEST_CASE(test_set_clock_source){
    auto usrp = get_usrp();
    // todo: test the non-happy path
    //sources = usrp->get_clock_sources(1);
}

BOOST_AUTO_TEST_CASE(test_get_clock_source){
    auto usrp = get_usrp();
    //BOOST_CHECK_EQUAL("internal", usrp->get_clock_source(0));
    // not happy-path testing?
}

BOOST_AUTO_TEST_CASE(test_get_clock_sources){
    auto usrp = get_usrp();

    // test the happy path
    auto sources = usrp->get_clock_sources(0);
    BOOST_CHECK_EQUAL(2, sources.size());
    BOOST_CHECK_EQUAL("internal", sources[0]);
    BOOST_CHECK_EQUAL("external", sources[1]);

    // not happy-path testing?
}

BOOST_AUTO_TEST_CASE(test_set_sync_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_sync_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_sync_sources){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_clock_source_out){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_time_source_out){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_num_mboards){
    auto usrp = get_usrp();
    BOOST_CHECK_EQUAL(1, usrp->get_num_mboards());
}

BOOST_AUTO_TEST_CASE(test_get_mboard_sensor){
    auto usrp = get_usrp();
    auto val = usrp->get_mboard_sensor("ref_locked", 0);
    BOOST_CHECK_EQUAL(uhd::sensor_value_t::BOOLEAN, val.type);
    // non-happy path tests?
}

BOOST_AUTO_TEST_CASE(test_get_mboard_sensor_names){
    auto usrp = get_usrp();
    auto names = usrp->get_mboard_sensor_names(0);
    BOOST_CHECK_EQUAL(1, names.size());
    BOOST_CHECK_EQUAL("ref_locked", names[0]);
    // non-happy path tests?
}

BOOST_AUTO_TEST_CASE(test_set_user_register){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

// get_user_settings_iface()

/*******************************************************************
 * RX methods
 ******************************************************************/

BOOST_AUTO_TEST_CASE(test_set_rx_subdev_spec){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_subdev_spec){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_num_channels){
    auto usrp = get_usrp();
    BOOST_CHECK_EQUAL(4, usrp->get_rx_num_channels());
}

BOOST_AUTO_TEST_CASE(test_get_rx_subdev_name){
    auto usrp = get_usrp();
    BOOST_CHECK_EQUAL("RX Board", usrp->get_rx_subdev_name(0));
    // non-happy path tests?
}

BOOST_AUTO_TEST_CASE(test_set_rx_rate){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_rate){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_rates){
    auto usrp = get_usrp();
    for(size_t chan = 0; chan < NUM_RX_CHANNELS; ++chan) {
        auto ranges = usrp->get_rx_rates(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(CLOCK_RATE/double(CLOCK_RATE_STEPS), range.start());
        BOOST_CHECK_EQUAL(CLOCK_RATE, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    // test non-happy paths?
}

BOOST_AUTO_TEST_CASE(test_set_rx_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_freq_range){
    auto usrp = get_usrp();
    for(size_t chan = 0; chan < NUM_RX_CHANNELS; ++chan) {
        auto ranges = usrp->get_rx_freq_range(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(-double(CLOCK_RATE)/4, range.start());
        BOOST_CHECK_EQUAL(FREQ_MAX + double(CLOCK_RATE)/4, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    // test non-happy paths?
}

BOOST_AUTO_TEST_CASE(test_get_fe_rx_freq_range){
    auto usrp = get_usrp();
    for(size_t chan = 0; chan < NUM_RX_CHANNELS; ++chan) {
        auto ranges = usrp->get_rx_freq_range(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(-double(CLOCK_RATE)/4, range.start());
        BOOST_CHECK_EQUAL(FREQ_MAX + double(CLOCK_RATE)/4, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    // test non-happy paths?
}

/**************************************************************************
 * LO controls
 *************************************************************************/

BOOST_AUTO_TEST_CASE(test_get_rx_lo_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_lo_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_lo_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_lo_sources){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_lo_export_enabled){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_lo_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_lo_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_lo_freq_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_lo_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_lo_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_get_tx_lo_source){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_lo_sources){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_lo_export_enabled){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_lo_export_enabled){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_lo_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_lo_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_lo_freq_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

/**************************************************************************
 * Gain controls
 *************************************************************************/

BOOST_AUTO_TEST_CASE(test_set_rx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_gain_profile_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_gain_profile){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_gain_profile){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_normalized_rx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_rx_agc){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_normalized_rx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_gain_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_gain_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_antenna){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_antenna){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_antennas){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_bandwidth){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_bandwidth){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_bandwidth_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_dboard_iface){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_sensor){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_sensor_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_dc_offset){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_rx_dc_offset_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_rx_iq_balance){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

/*******************************************************************
 * TX methods
 ******************************************************************/

BOOST_AUTO_TEST_CASE(test_set_tx_subdev_spec){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_subdev_spec){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_num_channels){
    auto usrp = get_usrp();
    BOOST_CHECK_EQUAL(4, usrp->get_tx_num_channels());
}

BOOST_AUTO_TEST_CASE(test_get_tx_subdev_name){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_rate){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_rate){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_rates){
    auto usrp = get_usrp();
    for(size_t chan = 0; chan < 2; ++chan) {
        auto ranges = usrp->get_tx_rates(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(CLOCK_RATE/double(CLOCK_RATE_STEPS), range.start());
        BOOST_CHECK_EQUAL(CLOCK_RATE, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    for(size_t chan = 2; chan < NUM_TX_CHANNELS; ++chan) {
        auto ranges = usrp->get_tx_rates(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(CLOCK_RATE/double(CLOCK_RATE_STEPS), range.start());
        BOOST_CHECK_EQUAL(double(CLOCK_RATE)/4, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    // test non-happy paths?
}

BOOST_AUTO_TEST_CASE(test_set_tx_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_freq){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_freq_range){
    auto usrp = get_usrp();
    for(size_t chan = 0; chan < 2; ++chan) {
        auto ranges = usrp->get_tx_freq_range(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(-double(CLOCK_RATE)/4, range.start());
        BOOST_CHECK_EQUAL(FREQ_MAX + double(CLOCK_RATE)/4, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    for(size_t chan = 2; chan < NUM_TX_CHANNELS; ++chan) {
        auto ranges = usrp->get_tx_freq_range(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(-double(CLOCK_RATE)/8, range.start());
        BOOST_CHECK_EQUAL(FREQ_MAX + double(CLOCK_RATE)/8, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    // test non-happy paths?
}

BOOST_AUTO_TEST_CASE(test_get_fe_tx_freq_range){
    auto usrp = get_usrp();
    for(size_t chan = 0; chan < 2; ++chan) {
        auto ranges = usrp->get_tx_rates(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(CLOCK_RATE/double(CLOCK_RATE_STEPS), range.start());
        BOOST_CHECK_EQUAL(CLOCK_RATE, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    for(size_t chan = 2; chan < NUM_TX_CHANNELS; ++chan) {
        auto ranges = usrp->get_tx_rates(chan);
        BOOST_CHECK_EQUAL(1, ranges.size());
        auto range = ranges[0];
        BOOST_CHECK_EQUAL(CLOCK_RATE/double(CLOCK_RATE_STEPS), range.start());
        BOOST_CHECK_EQUAL(CLOCK_RATE/4, range.stop());
        BOOST_CHECK_EQUAL(1, range.step());
    }
    // test non-happy paths?
}

BOOST_AUTO_TEST_CASE(test_set_tx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_gain_profile_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_gain_profile){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_gain_profile){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_normalized_tx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_normalized_tx_gain){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_gain_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_gain_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_antenna){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_antenna){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_antennas){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_bandwidth){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_bandwidth_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_dboard_iface){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_sensor){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_sensor_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_tx_dc_offset_range){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_tx_iq_balance){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

/*******************************************************************
 * GPIO methods
 ******************************************************************/

BOOST_AUTO_TEST_CASE(test_get_gpio_banks){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_set_gpio_attr){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_gpio_attr){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_gpio_string_attr){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_enumerate_registers){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_register_info){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_write_register){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_read_register){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

/*******************************************************************
 * Filter API methods
 ******************************************************************/

BOOST_AUTO_TEST_CASE(test_get_filter_names){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

BOOST_AUTO_TEST_CASE(test_get_filter){
    auto usrp = get_usrp();
    // it's quite possible that we will need to test a few things here
    //BOOST_FAIL("");
}

// note: none of the set_tree_value() and get_tree_value() methods
// are upstream, and I would recommend not modifying that API.
