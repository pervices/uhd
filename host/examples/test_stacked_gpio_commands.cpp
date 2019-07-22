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
        usrp->set_user_register(0, (uint32_t) (pins[0] >> 0x00)); // Lower first.
        usrp->set_user_register(1, (uint32_t) (mask[0] >> 0x00)); // Same goes for the mask.
        usrp->set_user_register(2, (uint32_t) (pins[0] >> 0x20)); // Then upper.
        usrp->set_user_register(3, (uint32_t) (mask[0] >> 0x20));
#ifdef PV_TATE
        usrp->set_user_register(4, (uint32_t) (pins[1] >> 0x00)); // Lower first.
        usrp->set_user_register(5, (uint32_t) (mask[1] >> 0x00)); // Same goes for the mask.
        usrp->set_user_register(6, (uint32_t) (pins[1] >> 0x20)); // Then upper.
        usrp->set_user_register(7, (uint32_t) (mask[1] >> 0x20));
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
    uint64_t pins [2] = {0x0, 0x0};
    uint64_t mask [2] = {0xFFFFFFFFFFFFFFFF, 0xFFFF};
    for(double time = 0.0; time < 64.0; time++) {
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
