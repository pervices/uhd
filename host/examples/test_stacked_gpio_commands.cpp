#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
namespace gpio
{
    void write(uhd::usrp::multi_usrp::sptr& usrp, const uint64_t pins [], const uint64_t mask [], const double time)
    {
        usrp->set_command_time(uhd::time_spec_t(time));
        // NOTE: We expect set_user_register to be called sequentially for these registers
        //       and the last register (3 for vaunt and 7 for tate) will trigger the
        //       gpio packet to be sent
        usrp->set_user_register(0, (uint32_t) (pins[0] >> 0x00)); // GPIO 31:0
        usrp->set_user_register(1, (uint32_t) (mask[0] >> 0x00)); // MASK for 31:0
        usrp->set_user_register(2, (uint32_t) (pins[0] >> 0x20)); // GPIO 63:32
        usrp->set_user_register(3, (uint32_t) (mask[0] >> 0x20)); // MASK for 63:32
#ifdef PV_TATE
        usrp->set_user_register(4, (uint32_t) (pins[1] >> 0x00)); // GPIO 95:64
        usrp->set_user_register(5, (uint32_t) (mask[1] >> 0x00)); // MASK for 95:64
        usrp->set_user_register(6, (uint32_t) (pins[1] >> 0x20)); // GPIO 128:96
        usrp->set_user_register(7, (uint32_t) (mask[1] >> 0x20)); // MASK for 128:96 (Also writes packet).
#endif
    }
}


int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    usrp->set_time_now(uhd::time_spec_t(0.0));


#ifdef PV_TATE
    std::cout << "GPIO example for Tate" << std::endl;
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
    uint64_t pins [2] = {0x0601806018060180, 0x6018};
    uint64_t mask [2] = {0xFFFFFFFFFFFFFFFF, 0xFFFF};
    // Toggle the pins for the next 10 seconds
    for(double time = 0.0; time < 10.0; time++) {
        pins[0] ^= mask[0];
        pins[1] ^= mask[1];
        gpio::write(usrp, pins, mask, time);
    }
#else
    std::cout << "GPIO example for Vaunt" << std::endl;
    // Note that Vaunt has 48 GPIO pins
    uint64_t pins = 0x0;
    const uint64_t mask = 0xFFFFFFFFFF;
    for(double time = 0.0; time < 64.0; time++)
        pins ^= mask;
        gpio::write(usrp, pins, mask, time);
#endif

    return 0;
}
