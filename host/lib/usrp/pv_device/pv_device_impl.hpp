#ifndef INCLUDED_PV_DEVICE_IMPL_HPP
#define INCLUDED_PV_DEVICE_IMPL_HPP

#include <set>
#include <vector>
#include <thread>
#include <atomic>
#include <sys/file.h>

#include "uhd/device.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/multi_usrp.hpp"
#include "uhd/transport/udp_zero_copy.hpp"

#include <uhdlib/usrp/common/pv_iface.hpp>
#include <uhdlib/usrp/common/clock_sync.hpp>
#include <uhdlib/usrp/common/stream_cmd_issuer.hpp>
#include <uhdlib/utils/system_time.hpp>
#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>
#include <immintrin.h>

#include "pv_device_io_impl.hpp"

typedef std::pair<uint8_t, uint32_t> user_reg_t;

namespace uhd {
namespace usrp {

#pragma pack(push,1)
struct gpio_burst_req {
    uint64_t header;
    int64_t tv_sec;
    int64_t tv_psec;
    uint64_t pins;
    uint64_t mask;
};
#pragma pack(pop)

class pv_device_impl : public uhd::device
{
public:
    static constexpr uint_fast8_t CACHE_LINE_SIZE = 64;

    pv_device_impl();
    virtual ~pv_device_impl();

    bool recv_async_msg_deprecated_warning = false;
    std::shared_ptr<uhd::pv_tx_async_msg_queue> _async_msg_fifo;
    uhd::device_addr_t device_addr;

    uhd::time_spec_t get_time_now();
    void start_pps_dtc();
    void stop_pps_dtc();

protected:
    virtual std::string get_log_id() const = 0;
    virtual std::string get_prop_prefix() const = 0;
    virtual void detect_pps_loop() = 0;
    virtual std::vector<stream_cmd_issuer>& get_rx_stream_cmd_issuer() = 0;
    virtual pv_iface::sptr get_mb_iface() = 0;

    int device_lock_fd;
    std::vector<int> tx_channel_lock_fd;
    std::vector<int> rx_channel_lock_fd;
    std::vector<int> tx_streaming_lock_fd;
    std::vector<int> rx_streaming_lock_fd;

    void lock_xx_channel_streaming(const size_t channel_num, const uhd::direction_t xx_sign);

    std::string rx_link_root(const size_t channel, const size_t mboard = 0);
    std::string tx_link_root(const size_t channel, const size_t mboard = 0);
    std::string tx_dsp_root(const size_t channel, const size_t mboard = 0);

    // Common device runtime/config state shared by Crimson/Cyan implementations
    int64_t max_buffer_level;
    int64_t buffer_level_multiple;
    int nsamps_multiple_rx;
    double max_sample_rate;
    int otw_rx;
    std::string otw_rx_s;
    int otw_tx;
    std::string otw_tx_s;

    bool gain_reset_warning_printed = false;
    std::vector<int> rx_gain_is_set;
    std::vector<int> last_set_rx_band;
    std::vector<int> tx_gain_is_set;
    std::vector<int> last_set_tx_band;

    // Common per-channel streaming/rate tracking state
    std::vector<double> tx_sfp_throughput_used;
    std::shared_ptr<std::vector<bool>> tx_channel_in_use;
    double link_rate_cache = 0;
    bool tx_rate_warning_printed = false;

    std::vector<double> rx_sfp_throughput_used;
    std::shared_ptr<std::vector<bool>> rx_channel_in_use;
    bool rx_rate_warning_printed = false;

    std::vector<stream_cmd_issuer> rx_stream_cmd_issuer;

    void set_stream_cmd(const std::string pre, uhd::stream_cmd_t data);
    void set_command_time(const std::string key, uhd::time_spec_t value);
    void send_gpio_burst_req(const gpio_burst_req& req);
    void set_user_reg(const std::string key, user_reg_t value);
    void set_properties_from_addr();

    std::vector<uhd::transport::udp_simple::sptr> _basic_sfp_iface;
    std::shared_ptr<clock_sync> device_clock_sync_info;

    std::thread _pps_thread;
    bool _pps_thread_needed;
    std::atomic<bool> _pps_thread_running;
    std::atomic<bool> _pps_thread_should_exit;

    time_spec_t _command_time;
};

}
}

#endif
