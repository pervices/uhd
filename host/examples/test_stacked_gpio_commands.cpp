#include <array>
#include <unordered_map>

#include <uhd/device.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

namespace gpio
{
    void write(uhd::usrp::multi_usrp::sptr& usrp, const std::vector<uint64_t> &pins, const std::vector<uint64_t> &mask, const double time)
    {
    	if (pins.size() != mask.size()) {
    		throw uhd::runtime_error("pins vector and mask vector must be same size");
    	}

        usrp->set_command_time(uhd::time_spec_t(time));
        for(size_t i = 0, N = pins.size(); i < N; ++i) {
            usrp->set_user_register(4*i + 0, uint32_t(pins[i] >>  0));
            usrp->set_user_register(4*i + 1, uint32_t(mask[i] >>  0));
            usrp->set_user_register(4*i + 2, uint32_t(pins[i] >> 32));
            usrp->set_user_register(4*i + 3, uint32_t(mask[i] >> 32));
        }
    }
}


int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    using type = uhd::device::device_filter_t;

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    usrp->set_time_now(uhd::time_spec_t(0.0));


    std::unordered_map<type,std::vector<uint64_t>> device_pin_map = {

        // Note that Vaunt has 48 GPIO pins
        {type::CRIMSON_TNG, {0x0}},

        // Note that Tate has 80 GPIO pins
        // The following is the mapping of the GPIO pins to the registers
        //
        //    pwr_en        : Power on the HDR board
        //    hi_pwr_en     : Enable the high power branch
        //    atten64..1    : Amount of attenuation (all will be summed together).
        //                      9          8          7          6          5          4          3          2          1          0
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL A:   9 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |   0
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL B:  19 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  10
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL C:  29 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  20
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL D:  39 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  30
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL E:  49 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  40
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL F:  59 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  50
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL G:  69 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  60
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // CHANNEL H:  79 | Reserved |   pwr_en | hi_pwr_en| atten64  | atten32  | atten16  | atten8   | atten4   | atten2   | atten1   |  70
        //                +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
        // default is to set pwr_en and enable hi_pwr branch and set attenuation to minimum (0).
        {type::CYAN_16T, {0x0601806018060180, 0x6018}},
    };

    std::unordered_map<type,std::vector<uint64_t>> device_mask_map = {
        {type::CRIMSON_TNG, {0xFFFFFFFFFF}},
        {type::CYAN_16T, {0xFFFFFFFFFFFFFFFF, 0xFFFF}},
    };

    auto dev = usrp->get_device()->get_device_type();
    auto pins = device_pin_map[dev];
    auto mask = device_mask_map[dev];

    // Toggle the pins for the next 10 seconds
    for(double time = 0.0; time < 10.0; time++) {
        for(size_t i = 0; i < pins.size(); ++i) {
            pins[i] ^= mask[i];
        }
        gpio::write(usrp, pins, mask, time);
    }

    return 0;
}
