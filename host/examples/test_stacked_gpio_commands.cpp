#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
namespace gpio
{
    void write(uhd::usrp::multi_usrp::sptr& usrp, const uint64_t pins, const uint64_t mask, const double time)
    {
        usrp->set_command_time(uhd::time_spec_t(time));
        // NOTE: We expect set_user_register to be called sequentially for these registers
        //       and the last register (3 for vaunt and 7 for tate) will trigger the
        //       gpio packet to be sent
        usrp->set_user_register(0, (uint32_t) (pins >> 0x00)); // GPIO 31:0
        usrp->set_user_register(1, (uint32_t) (mask >> 0x00)); // MASK for 31:0
        usrp->set_user_register(2, (uint32_t) (pins >> 0x20)); // GPIO 63:32
        usrp->set_user_register(3, (uint32_t) (mask >> 0x20)); // MASK for 63:32
    }
}


int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    usrp->set_time_now(uhd::time_spec_t(0.0));

    std::cout << "GPIO example for Vaunt" << std::endl;
    // Note that Vaunt has 48 GPIO pins
    uint64_t pins = 0x0;
    const uint64_t mask = 0xFFFFFFFFFF;
    for(double time = 0.0; time < 64.0; time++) {
        pins ^= mask;
        gpio::write(usrp, pins, mask, time);
	}

    return 0;
}
