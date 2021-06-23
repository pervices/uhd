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
        std::cout << "The value is now: " << new_value << std::endl;

    }

}

int UHD_SAFE_MAIN(int argc, char *argv[])
{

    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string path, value;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("path", po::value<std::string>(&path)->default_value(""), "The path for the value in the UHD state tree to set")
        ("value", po::value<std::string>(&value)->default_value(""), "The value the variable it to be set to")

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD set value at path %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This set the value at the path specified.\n"
            << std::endl;
        return ~0;
    }

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    set_at_path(usrp, path, value);

    std::cout << "Done" << std::endl;

    return 0;
}
