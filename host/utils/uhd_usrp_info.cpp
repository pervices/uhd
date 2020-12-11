#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/version.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <iostream>

namespace po = boost::program_options;

namespace {
//! Conditionally append find_all=1 if the key isn't there yet
uhd::device_addr_t append_findall(const uhd::device_addr_t& device_args)
{
    uhd::device_addr_t new_device_args(device_args);
    if (!new_device_args.has_key("find_all")) {
        new_device_args["find_all"] = "1";
    }

    return new_device_args;
}
} // namespace

typedef std::map<std::string, std::set<std::string>> device_multi_addrs_t;
typedef std::map<std::string, device_multi_addrs_t> device_addrs_filtered_t;

void print_usage(po::options_description desc)
{
    std::cout << "Usage: uhd_usrp_init [OPTIONS]..." << std::endl
              << "Diagnostic script that completely captures the state, code, and "
                 "revision of a Per Vices SDR"
              << std::endl
              << std::endl
              << desc << std::endl
              << std::endl
              << "Examples:" << std::endl
              << "    uhd_usrp_init -v" << std::endl
              << std::endl;
}

std::string get_from_tree(
    uhd::property_tree::sptr tree, const int device_id, const char* relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return tree->access<std::string>(path).get();
}

std::string get_from_tree_int(
    uhd::property_tree::sptr tree, const int device_id, const char* relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return std::to_string(tree->access<int>(path).get());
}

std::string get_from_tree_double(
    uhd::property_tree::sptr tree, const int device_id, const char* relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return std::to_string(tree->access<double>(path).get());
}

device_addrs_filtered_t find_devices(uhd::device_addrs_t device_addrs)
{
    device_addrs_filtered_t found_devices;
    for (auto it = device_addrs.begin(); it != device_addrs.end(); ++it) {
        std::string serial    = (*it)["serial"];
        found_devices[serial] = device_multi_addrs_t();
        for (std::string key : it->keys()) {
            if (key != "serial") {
                found_devices[serial][key].insert(it->get(key));
            }
        }
        for (auto sit = it + 1; sit != device_addrs.end();) {
            if ((*sit)["serial"] == serial) {
                for (std::string key : sit->keys()) {
                    if (key != "serial") {
                        found_devices[serial][key].insert(sit->get(key));
                    }
                }
                sit = device_addrs.erase(sit);
            } else {
                sit++;
            }
        }
    }
    return found_devices;
}


int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // uhd::set_thread_priority_safe();
    po::options_description desc("Allowed options");

    // clang-format off
    desc.add_options()
        ("networking,i", "provide the Management/SFP+ port IP addresses")
        ("time,t", "provide the SOC: 'date' 'hwclock' and the time on SDR")
        ("lock,l", "provide the PLL lock status for Rx/Tx/time and if we are using internal/external reference")
        ("boards,r", "provide the RFE front end status for each board")
        ("all,v", "prints all of the information described above.")
	("args", po::value<std::string>()->default_value(""), "device address args")
    ;
    // clang-format on


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    const uhd::device_addr_t args(vm["args"].as<std::string>());

    if (argc <= 1) {
        print_usage(desc);
        return EXIT_SUCCESS;
    }

    std::cout << "UHD Software Library : " << uhd::get_version_string() << std::endl;

    // Find the devices
    uhd::device_addrs_t device_addrs = uhd::device::find(append_findall(args));
    if (device_addrs.empty()) {
        std::cerr << "No UHD Devices Found" << std::endl;
        return EXIT_FAILURE;
    }
    device_addrs_filtered_t found_devices = find_devices(device_addrs);

    int i        = 0;
    int ndevices = found_devices.size();

    std::cout << std::endl;
    std::cout << ndevices << " SDRs found!" << std::endl << std::endl;

    // create property tree for later
    uhd::device::sptr dev         = uhd::device::make(vm["args"].as<std::string>());
    uhd::property_tree::sptr tree = dev->get_tree();

    std::string device_type, device_address;
    for (auto dit = found_devices.begin(); dit != found_devices.end(); ++dit) {
        std::cout << "Device " << (i + 1) << std::endl;

        for (auto mit = dit->second.begin(); mit != dit->second.end(); ++mit) {
            for (auto vit = mit->second.begin(); vit != mit->second.end(); ++vit) {
                if (mit->first == "addr") {
                    device_address = *vit;
                }
                if (mit->first == "type") {
                    device_type = *vit;
                }
            }
        }

        if(device_type.compare("crimson_tng") != 0){
            std::cout << "ERROR: not implemented for current device type\n";
            continue;
        }

        std::cout << "Device Type    : " << device_type << std::endl;
        std::cout << "Server Version : " << get_from_tree(tree, i, "server_version")
                  << std::endl;
        std::cout << "FPGA Version   : " << get_from_tree(tree, i, "fw_version")
                  << std::endl;

        std::cout << "Board MCU revision: " << std::endl;
        std::cout << "\tTime : " << get_from_tree(tree, i, "time/fw_ver") << std::endl;
        std::cout << "\trx(a): " << get_from_tree(tree, i, "rx/0/fw_ver") << std::endl;
        std::cout << "\trx(a): " << get_from_tree(tree, i, "rx/1/fw_ver") << std::endl;
        std::cout << "\trx(a): " << get_from_tree(tree, i, "rx/2/fw_ver") << std::endl;
        std::cout << "\trx(a): " << get_from_tree(tree, i, "rx/3/fw_ver") << std::endl;
        std::cout << "\ttx(a): " << get_from_tree(tree, i, "tx/0/fw_ver") << std::endl;
        std::cout << "\ttx(a): " << get_from_tree(tree, i, "tx/1/fw_ver") << std::endl;
        std::cout << "\ttx(a): " << get_from_tree(tree, i, "tx/2/fw_ver") << std::endl;
        std::cout << "\ttx(a): " << get_from_tree(tree, i, "tx/3/fw_ver") << std::endl;

        if (vm.count("networking") || vm.count("all")) {
            std::cout << "Device Address : " << std::endl;
            std::cout << "\tManagement IP: " << device_address << std::endl;
            std::cout << "\tSFP A IP     : " << get_from_tree(tree, i, "sfpa/ip_addr")
                      << std::endl;
            std::cout << "\tSFP B IP     : " << get_from_tree(tree, i, "sfpb/ip_addr")
                      << std::endl;
        }

        if (vm.count("time") || vm.count("all")) {
            std::cout << "Time (fpga/gps_time) : " << get_from_tree_int(tree, i,"gps_time") << std::endl;
            std::cout << "Time (time/curr_time): " << get_from_tree_int(tree, i,"time/now") << std::endl;
        }

        if (vm.count("lock") || vm.count("all")) {
            std::cout << "TODO: provide the PLL lock status for Rx/Tx/time and if we are "
                         "using internal/external reference\n";
        }

        if (vm.count("boards") || vm.count("all")) {
            std::cout << "TODO: RFE frontend status of each board\n";
            std::cout << "Rx Board A" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/A/rx_frontends/Channel_A/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/A/rx_frontends/Channel_A/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/A/rx_frontends/Channel_A/gain/value") << std::endl;
            std::cout << "Rx Board B" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/B/rx_frontends/Channel_B/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/B/rx_frontends/Channel_B/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/B/rx_frontends/Channel_B/gain/value") << std::endl;
            std::cout << "Rx Board C" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/C/rx_frontends/Channel_C/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/C/rx_frontends/Channel_C/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/C/rx_frontends/Channel_C/gain/value") << std::endl;
            std::cout << "Rx Board D" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/D/rx_frontends/Channel_D/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/D/rx_frontends/Channel_D/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/D/rx_frontends/Channel_D/gain/value") << std::endl;
            std::cout << "Tx Board A" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/A/tx_frontends/Channel_A/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/A/tx_frontends/Channel_A/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/A/tx_frontends/Channel_A/gain/value") << std::endl;
            std::cout << "Tx Board B" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/B/tx_frontends/Channel_B/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/B/tx_frontends/Channel_B/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/B/tx_frontends/Channel_B/gain/value") << std::endl;
            std::cout << "Tx Board C" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/C/tx_frontends/Channel_C/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/C/tx_frontends/Channel_C/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/C/tx_frontends/Channel_C/gain/value") << std::endl;
            std::cout << "Tx Board D" << std::endl;
            std::cout << "    RF Band: " << get_from_tree_double(tree, i,"dboards/D/tx_frontends/Channel_D/freq/band") << std::endl;
            std::cout << "    RF Freq: " << get_from_tree_double(tree, i,"dboards/D/tx_frontends/Channel_D/freq/value") << std::endl;
            std::cout << "    RF Gain: " << get_from_tree_double(tree, i,"dboards/D/tx_frontends/Channel_D/gain/value") << std::endl;
        }

        i++;
    }

    std::cout << std::endl;

    return EXIT_SUCCESS;
}
