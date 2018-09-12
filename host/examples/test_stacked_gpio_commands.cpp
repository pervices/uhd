#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

namespace gpio
{
    void write(uhd::usrp::multi_usrp::sptr& usrp, const uint64_t pins, const uint64_t mask, const double time)
    {
        usrp->set_command_time(uhd::time_spec_t(time));
        usrp->set_user_register(0, (uint32_t) (pins >> 0x00)); // Lower first.
        usrp->set_user_register(1, (uint32_t) (pins >> 0x20)); // Then upper.
        usrp->set_user_register(2, (uint32_t) (mask >> 0x00)); // Same goes for the mask.
        usrp->set_user_register(3, (uint32_t) (mask >> 0x20));
    }
}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    usrp->set_time_now(uhd::time_spec_t(0.0));

    uint64_t pins = 0x0;
    const uint64_t all = 0xFFFFFFFFFF;
    for(double time = 0.0; time < 64.0; time++)
        gpio::write(usrp, pins ^= all, all, time);

    return 0;
}
