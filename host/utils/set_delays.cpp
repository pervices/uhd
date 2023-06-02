#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

namespace po = boost::program_options;

std::vector<size_t> parse_argument(std::string argument) {
    if(argument.size() == 0) return std::vector<size_t>(0);
    std::vector<std::string> seperated_arguments;
    boost::split(seperated_arguments, argument, boost::is_any_of("\"',"));
    size_t num_args = seperated_arguments.size();
    std::vector<size_t> parsed_args(num_args);
    for(size_t n = 0; n < num_args; n++) {
        parsed_args[n] = std::stoul(seperated_arguments[n]);
    }
    return parsed_args;
}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    //variables to be set by po
    std::string args, tx_channels_s, tx_i_delays_s, tx_q_delays_s, rx_channels_s, rx_i_delays_s, rx_q_delays_s;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("tx_channels", po::value<std::string>(&tx_channels_s)->default_value(""), "The tx channels to set delays for")
        ("tx_i_delay", po::value<std::string>(&tx_i_delays_s)->default_value(""), "The amount to delay tx i by in samples")
        ("tx_q_delay", po::value<std::string>(&tx_q_delays_s)->default_value(""), "The amount to delay tx q by in samples")
        ("rx_channels", po::value<std::string>(&rx_channels_s)->default_value(""), "The rx channels to set delays for")
        ("rx_i_delay", po::value<std::string>(&rx_i_delays_s)->default_value(""), "The amount to delay rx i by in samples")
        ("rx_q_delay", po::value<std::string>(&rx_q_delays_s)->default_value(""), "The amount to delay rx q by in samples")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD set value at path %s") % desc << std::endl;
        std::cout
            << std::endl
            << "Sets delays on i and q of channels. Used for calibration\n"
            << std::endl;
        return ~0;
    }

    std::vector<size_t> tx_channels = parse_argument(tx_channels_s);
    std::vector<size_t> tx_i_delays = parse_argument(tx_i_delays_s);
    std::vector<size_t> tx_q_delays = parse_argument(tx_q_delays_s);
    std::vector<size_t> rx_channels = parse_argument(rx_channels_s);
    std::vector<size_t> rx_i_delays = parse_argument(rx_i_delays_s);
    std::vector<size_t> rx_q_delays = parse_argument(rx_q_delays_s);

    size_t num_tx_channels = tx_channels.size();
    if(num_tx_channels != tx_i_delays.size()) {
        std::cerr << "Mismatch between number of tx channels and number of tx i delays specified" << std::endl;
        return ~0;
    }
    if(num_tx_channels != tx_q_delays.size()) {
        std::cerr << "Mismatch between number of tx channels and number of tx q delays specified" << std::endl;
        return ~0;
    }
    size_t num_rx_channels = rx_channels.size();
    if(num_rx_channels != rx_i_delays.size()) {
        std::cerr << "Mismatch between number of rx channels and number of rx i delays specified" << std::endl;
        return ~0;
    }
    if(num_rx_channels != rx_q_delays.size()) {
        std::cerr << "Mismatch between number of rx channels and number of rx q delays specified" << std::endl;
        return ~0;
    }

    if(num_tx_channels == 0 && num_rx_channels == 0) {
        std::cout << "No channels specified" << std::endl;
        return 0;
    }

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    for(size_t n = 0; n < num_tx_channels; n++) {
        usrp->set_tx_delay(tx_channels[n], tx_i_delays[n], tx_q_delays[n]);
    }
    for(size_t n = 0; n < num_rx_channels; n++) {
        usrp->set_rx_delay(rx_channels[n], rx_i_delays[n], rx_q_delays[n]);
    }

    std::cout << "Done" << std::endl;

    return 0;
}
