#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>

#define DEBUG_MANUAL

namespace
{

    void set_at_path(uhd::usrp::multi_usrp::sptr& usrp, std::string path, std::string value)
    {
        std::cout << __FUNCTION__ << std::endl;

        std::cout << "Setting value at: " << path << std::endl;

        //gets the old value from the state tree
        std::string old_value;
        usrp->get_tree_value(path, old_value);
        std::cout << "Old value: " << old_value << std::endl;

        usrp->set_tree_value(path, value);
        std::cout << "Set to: " << value << std::endl;

        std::string new_value;
        usrp->get_tree_value(path, value);
        std<<cout << "The value is now: " << new_value << std::endl;

    }

}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    set_at_path(usrp, );

    return 0;
}
