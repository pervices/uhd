#include <boost/asio.hpp>
#include <boost/endian/conversion.hpp>

#include "pv_device_impl.hpp"

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

static std::string mb_root(const size_t mboard = 0) {
    return "/mboards/" + std::to_string(mboard);
}

static size_t pre_to_ch(const std::string& pre) {
    char x = -1;
    if (1 != sscanf(pre.c_str(), "rx_%c/stream", &x)) {
        throw value_error("Invalid 'pre' argument '" + pre + "'");
    }
    return x - 'a';
}

pv_device_impl::pv_device_impl()
    : _pps_thread_needed(false)
    , _pps_thread_running(false)
    , _pps_thread_should_exit(false)
    , _command_time(0.0)
{
}

pv_device_impl::~pv_device_impl() {}

std::string pv_device_impl::rx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/rx_link/" + std::to_string(channel);
}

std::string pv_device_impl::tx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/tx_link/" + std::to_string(channel);
}

std::string pv_device_impl::tx_dsp_root(const size_t channel, const size_t mboard) {
    return mb_root(mboard) + "/tx_dsps/" + std::to_string(channel);
}

void pv_device_impl::lock_xx_channel_streaming(const size_t channel_num, const uhd::direction_t xx_sign) {
    int lock_fd;
    switch(xx_sign) {
        case RX_DIRECTION:
            lock_fd = rx_streaming_lock_fd[channel_num];
            break;
        case TX_DIRECTION:
            lock_fd = tx_streaming_lock_fd[channel_num];
            break;
        default:
            throw uhd::value_error("Invalid 'xx_sign' argument. Expected either RX_DIRECTION or TX_DIRECTION of type uhd::direction_t");
    }
    int r = flock(lock_fd, LOCK_EX | LOCK_NB);
    if (r == -1) {
        int err = errno;
        if (err == EWOULDBLOCK) {
            throw uhd::runtime_error("Another instance of UHD is currently using channel " + std::to_string(channel_num) + ".");
        } else {
            throw uhd::runtime_error("flock failed to lock streaming for channel " + std::to_string(channel_num) + " with error: " + std::string(strerror(err)));
        }
    }
}

void pv_device_impl::set_stream_cmd( const std::string pre, stream_cmd_t stream_cmd ) {
    const size_t ch = pre_to_ch( pre );
    rx_stream_cmd_issuer[ch].issue_stream_command(stream_cmd);
}

void pv_device_impl::set_command_time( const std::string key, time_spec_t value ) {
    (void) key;
    _command_time = value;
}

void pv_device_impl::send_gpio_burst_req(const gpio_burst_req& req) {
    _basic_sfp_iface[0]->send(boost::asio::const_buffer(&req, sizeof(req)));
}

void pv_device_impl::set_user_reg(const std::string key, user_reg_t value) {
    (void) key;

    const uint8_t  address = value.first;
    const uint64_t setting = value.second;

    static uint64_t pins = 0x0;
    static uint64_t mask = 0x0;

    const uint64_t all = 0xFFFFFFFF;
    if(address == 0) pins &= ~(all << 0x00);
    if(address == 1) pins &= ~(all << 0x20);
    if(address == 2) mask &= ~(all << 0x00);
    if(address == 3) mask &= ~(all << 0x20);

    if(address == 0) pins |= (setting << 0x00);
    if(address == 1) pins |= (setting << 0x20);
    if(address == 2) mask |= (setting << 0x00);
    if(address == 3) mask |= (setting << 0x20);

    if(address > 3)
        std::cout << "UHD: WARNING: User defined registers [4:256] not defined" << std::endl;

    if(address == 3)
    {
        gpio_burst_req pkt;
        pkt.header = ((uint64_t) 0x3) << 32;
        pkt.tv_sec = _command_time.get_full_secs();
        pkt.tv_psec = _command_time.get_frac_secs() * 1e12;
        pkt.pins = pins;
        pkt.mask = mask;

        std::printf(
            "SHIPPING(set_user_reg):\n"
            "0x%016lX\n"
            "0x%016lX\n"
            "0x%016lX\n"
            "0x%016lX\n"
            "0x%016lX\n", pkt.header, pkt.tv_sec, pkt.tv_psec, pkt.pins, pkt.mask);

        boost::endian::native_to_big_inplace(pkt.header);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.tv_sec);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.tv_psec);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.pins);
        boost::endian::native_to_big_inplace((uint64_t&) pkt.mask);

        send_gpio_burst_req(pkt);
    }
}

uhd::time_spec_t pv_device_impl::get_time_now() {
    return device_clock_sync_info->get_device_time();
}

void pv_device_impl::set_properties_from_addr() {
    const std::string prop_prefix(get_prop_prefix());
    static const std::vector<std::string> blacklist { "crimson:sob" };

    for( auto & prop: device_addr.keys() ) {
        if ( 0 == prop.compare( 0, prop_prefix.length(), prop_prefix ) ) {
            bool is_blacklisted = false;
            for( auto & e: blacklist ) {
                if ( e == prop ) {
                    is_blacklisted = true;
                }
            }
            if ( is_blacklisted ) {
                continue;
            }

            std::string key = prop.substr( prop_prefix.length() );
            std::string expected_string = device_addr[ prop ];

            _mbc.iface->set_string( key, expected_string );
            std::string actual_string = _mbc.iface->get_string( key );
            if ( actual_string != expected_string ) {
                UHD_LOGGER_ERROR(get_log_id() + "_IMPL")
                    << __func__ << "(): "
                    << "Setting device property failed: "
                    << "key: '"<< key << "', "
                    << "expected val: '" << expected_string << "', "
                    << "actual val: '" << actual_string  << "'"
                    << std::endl;
            }
        }
    }
}

void pv_device_impl::start_pps_dtc() {
    if ( ! _pps_thread_needed ) {
        return;
    }

    if ( ! _pps_thread_running.load(std::memory_order_relaxed) ) {
        _pps_thread_should_exit.store(false, std::memory_order_relaxed);
        _pps_thread = std::thread([this]() { detect_pps_loop(); });
    }
}

void pv_device_impl::stop_pps_dtc() {
    if ( ! _pps_thread_needed ) {
        return;
    }

    if(_pps_thread.joinable()) {
        _pps_thread_should_exit.store(true, std::memory_order_relaxed);
        _pps_thread.join();
    }
}
