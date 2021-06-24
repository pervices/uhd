#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace
{

    void set_at_path(uhd::usrp::multi_usrp::sptr& usrp, std::string path)
    {
        std::cout << __FUNCTION__ << std::endl;

        std::cout << "Getting value at: " << path << std::endl;
        //gets the value from the state tree
        std::string value;
        usrp->get_tree_value(path, value);
        std::cout << "Value: " << old_value << std::endl;
    }

}

int UHD_SAFE_MAIN(int argc, char *argv[])
{

    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string path;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("path", po::value<std::string>(&path)->default_value(""), "The path for the value in the UHD state tree to get")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD get value at path %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This gets the value at the path specified.\n"
            << std::endl;
        return ~0;
    }

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(std::string(""));

    set_at_path(usrp, path);

    std::cout << "Done" << std::endl;

    return 0;
}
