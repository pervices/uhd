#include <uhd/utils/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace
{

    void get_string_at_path(uhd::usrp::multi_usrp::sptr& usrp, std::string path)
    {
        std::cout << __FUNCTION__ << std::endl;

        std::cout << "Getting value at: " << path << std::endl;
        //gets the value from the state tree
        std::string value;
        usrp->get_tree_value(path, value);
        std::cout << "Value: " << value << std::endl;

    }
    void get_int_at_path(uhd::usrp::multi_usrp::sptr& usrp, std::string path)
    {
        std::cout << __FUNCTION__ << std::endl;

        std::cout << "Getting value at: " << path << std::endl;
        //gets the value from the state tree
        int value;
        usrp->get_tree_value(path, value);
        std::cout << "Value: " << value << std::endl;
    }
    void get_double_at_path(uhd::usrp::multi_usrp::sptr& usrp, std::string path)
    {
        std::cout << __FUNCTION__ << std::endl;

        std::cout << "Getting value at: " << path << std::endl;
        //gets the value from the state tree
        double value;
        usrp->get_tree_value(path, value);
        std::cout << "Value: " << value << std::endl;
    }

}

int UHD_SAFE_MAIN(int argc, char *argv[])
{

    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, path, type;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("path", po::value<std::string>(&path)->default_value(""), "The path for the value in the UHD state tree to get")
        ("type", po::value<std::string>(&type)->default_value("string"), "The data type of the variable to get")
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

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    if(type.compare("double")==0) get_double_at_path(usrp, path);
    else if (type.compare("int")==0) get_int_at_path(usrp, path);
    else get_string_at_path(usrp, path);

    std::cout << "Done" << std::endl;

    return 0;
}
