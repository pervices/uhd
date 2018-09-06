#include <uhd/usrp/multi_usrp.hpp>

int main()
{
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));
    usrp->set_command_time(uhd::time_spec_t(42.9999999));
    //usrp->set_user_register(0x10, 10);
}
