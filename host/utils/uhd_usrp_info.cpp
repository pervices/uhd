#include <uhd/device.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/version.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <cstdlib>
#include <iostream>
#include <chrono>
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
typedef std::function<uhd::sensor_value_t(const std::string&)> get_sensor_fn_t;

std::string get_from_tree(
    uhd::property_tree::sptr tree, const int device_id, std::string relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    std::cout << "get_from_tree: " << path << std::endl;
    return tree->access<std::string>(path).get();
}

std::string silent_get_from_tree(
    uhd::property_tree::sptr tree, const int device_id, std::string relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return tree->access<std::string>(path).get();
}

std::string get_from_tree_int(
    uhd::property_tree::sptr tree, const int device_id, std::string relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return std::to_string(tree->access<int>(path).get());
}

std::string get_from_tree_double(
    uhd::property_tree::sptr tree, const int device_id, std::string relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return std::to_string(tree->access<double>(path).get());
}

std::string get_from_tree_time_spec(
    uhd::property_tree::sptr tree, const int device_id, std::string relative_path)
{
    std::string path = "/mboards/" + std::to_string(device_id) + "/" + relative_path;
    return std::to_string(tree->access<uhd::time_spec_t>(path).get().get_real_secs());
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

std::string extract_git_hash(std::string verbose_version) {
    size_t version_line_start = verbose_version.find("Revision");
    if(version_line_start == std::string::npos) {
        std::cout << "Error when attempting to extract git hash from " << verbose_version << std::endl;
        return verbose_version;
    }
    size_t version_line_end = verbose_version.find('\n', version_line_start);

    size_t git_hash_start = version_line_end;
    while(git_hash_start >= version_line_start) {
        if(verbose_version[git_hash_start - 1] == 'g' || verbose_version[git_hash_start - 1] == ' ') {
            return verbose_version.substr(git_hash_start, version_line_end - git_hash_start);
        }
        git_hash_start--;
    }
    std::cout << "Unable to locate git version line on: " << verbose_version << std::endl;
    // If unable to locate the git hash, return the full verbose string
    return verbose_version;
}

void parse_server_version(std::string server_version) {
    size_t revision_start = server_version.find("Revision");
    size_t rtm_start = server_version.find("RTM");
    size_t fpga_rev_start = server_version.find(':', server_version.find("FPGA:"));

    std::cout << "\nServer " << server_version.substr(revision_start, server_version.find('\n', revision_start) - revision_start) << std::endl;
    std::cout << "Server " << server_version.substr(rtm_start, server_version.find('\n', rtm_start) - rtm_start) << std::endl;
    std::cout << "FPGA Revision: " << server_version.substr(fpga_rev_start, server_version.find('\n', fpga_rev_start) - fpga_rev_start) << std::endl;
}

void parse_time_version(std::string time_version) {
    size_t rev_start = time_version.find("Revision:");
    size_t branch_start = time_version.find("Branch:");
    
    std::cout << "Time MCU " << time_version.substr(rev_start, time_version.find('\n', rev_start) - rev_start) << std::endl;
    std::cout << "Time MCU " << time_version.substr(branch_start, time_version.find('\n', branch_start) - branch_start) << std::endl;
}


int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // uhd::set_thread_priority_safe();
    po::options_description desc("Allowed options");

    std::string args;

    // clang-format off
    desc.add_options()
        ("help,h", "help message")
        ("all,v", "prints information for all subsystems")
	("hwinfo,p", "prints formatted versioning info")
        ("git,g", "prints only the git hash instead of full version info for subsystems")
        ("server,s", "prints all information related to the server")
        ("fpga,f", "prints all information related to the fpga")
        ("tx,t", "prints all information related to tx")
        ("rx,r", "prints all information related to rx")
        ("time,c", "prints all information related to the time board")
        ("network,i", "provide the Management/SFP+ port IP addresses")
        ("args", po::value<std::string>(&args)->default_value(""), "device address args")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

        //print the help message
    if (vm.count("help") || argc <= 1) {
        std::cout << boost::format("UHD_USRP_INFO %s") % desc << std::endl;
                std::cout << std::endl
                  << "This application gets various parameters related to the system. It is primarily meant for disagnostic/checking versions\n"
                  << std::endl;
        return ~0;
    }

    bool all_info = vm.count("all");
    bool hw_info = vm.count("hwinfo");
    bool git_hash_only = vm.count("git");
    bool server_info = all_info || vm.count("server");
    bool fpga_info = all_info || vm.count("fpga");
    bool tx_info = all_info || vm.count("tx");
    bool rx_info = all_info || vm.count("rx");
    bool time_info = all_info || vm.count("time");
    bool network_info = all_info || vm.count("network");

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
    uhd::usrp::multi_usrp::sptr dev         = uhd::usrp::multi_usrp::make(args);
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

	if (hw_info) {
	    std::time_t timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	    std::cout << std::put_time(std::gmtime(&timestamp), "\n%Y%m%dT%H%M%S") << std::endl;
	    //USER@HOST HERE
	    std::cout << "\nUHD Library Version: " << uhd::get_version_string() << std::endl; 
	    std::cout << "Device Type: " << device_type << std::endl;
	    std::cout << "Serial: " << dit->first << std::endl;

	    std::string server_version = silent_get_from_tree(tree, i, "server_version");
	    std::string fpga_version = silent_get_from_tree(tree, i, "fw_version");
	    std::string time_version = silent_get_from_tree(tree, i, "time/fw_version");
	    
	    parse_server_version(server_version);
	    std::cout << "FPGA Sample Rate: " << get_from_tree_double(tree, i, "system/max_rate") << std::endl;

	    if (device_type == "crimson_tng") {
		std::cout << "FPGA Flags: " << get_from_tree_int(tree, i, "system/is_full_tx") << '\n' << std::endl;
	    } else {
		std::cout << "FPGA Ethernet Flags: " << get_from_tree_double(tree, i, "link_max_rate") << std::endl;
		std::cout << "FPGA backplane pinout: " << get_from_tree_int(tree, i, "imgparam/backplane_pinout") << '\n' << std::endl;
	    }

	    parse_time_version(time_version);
	    if (tree->exists("mboards/0/time/eeprom")) {
		std::string time_eeprom = silent_get_from_tree(tree, i, "time/eeprom");
		std::cout << "Time EEPROM: " << time_eeprom.substr(0, time_eeprom.find('\n')) << std::endl;
	    }

            size_t num_tx_channels = dev->get_tx_num_channels();
            size_t num_rx_channels = dev->get_rx_num_channels();

	    if (num_tx_channels > 1) {
		std::cout << "Num Tx RFE: " << num_tx_channels << std::endl;
	    }
	    if (num_rx_channels > 1) {
		std::cout << "Num Rx RFE: " << num_rx_channels << std::endl;
	    }

	    for(size_t rx_chan = 0; rx_chan < num_rx_channels; rx_chan++) {
		char path[50];
		sprintf(path, "rx/%lu/fw_version", rx_chan);
		std::string rx_version = silent_get_from_tree(tree, i, path);
		size_t rfe_rev_start = rx_version.find("Revision:");
		
		std::cout << "Rx" << rx_chan << " MCU " << rx_version.substr(rfe_rev_start, rx_version.find('\n', rfe_rev_start)-rfe_rev_start) << std::endl;

		sprintf(path, "rx/%lu/eeprom", rx_chan);
		if (tree->exists(std::string("mboards/0/") + path)) {
		    std::string rx_eeprom = silent_get_from_tree(tree, i, path);
		    std::cout << "Rx" << rx_chan << " MCU EEPROM: " << rx_eeprom.substr(0, rx_eeprom.find('\n')) << std::endl;
		} 
	    }

	    for (size_t tx_chan = 0; tx_chan < num_tx_channels; tx_chan++) {
		char path[50];
		sprintf(path, "tx/%lu/fw_version", tx_chan);
		std::string tx_version = silent_get_from_tree(tree, i, path);
		size_t rfe_rev_start = tx_version.find("Revision:");
		
		std::cout << "Tx" << tx_chan << " MCU " << tx_version.substr(rfe_rev_start, tx_version.find('\n', rfe_rev_start)-rfe_rev_start) << std::endl;

		sprintf(path, "tx/%lu/eeprom", tx_chan);
		if (tree->exists(std::string("mboards/0/") + path)) {
		    std::string tx_eeprom = silent_get_from_tree(tree, i, path);
		    std::cout << "Tx" << tx_chan << " MCU EEPROM: " << tx_eeprom.substr(0, tx_eeprom.find('\n')) << std::endl;
		}
	    }
	}

        std::cout << "Device Type    : " << device_type << std::endl;

        if(server_info) {
            try {
                std::string version = get_from_tree(tree, i, "server_version");
                if(git_hash_only) {
                    version = extract_git_hash(version);
                }
                std::cout << "Server Version : " <<  version << std::endl;
            } catch (const uhd::lookup_error&) {
                std::cout << "Server version lookup not implemented" << std::endl;
            }
        }

        if(fpga_info) {
            try {
                std::cout << "FPGA Version   : " << get_from_tree(tree, i, "fw_version") << std::endl;
            } catch (const uhd::lookup_error&) {
                std::cout << "FPGA version lookup not implemented" << std::endl;
            }

            try {
            } catch (const uhd::lookup_error&) {
                std::cout << "FPGA version lookup not implemented" << std::endl;
            }

            try {
                std::cout << "System sample rate: " << get_from_tree_double(tree, i, "system/max_rate") << std::endl;
            } catch (const uhd::lookup_error&) {
                // Should be implemented except on very old servers
                std::cout << "System sample rate lookup not implemented" << std::endl;
            }

            if(!git_hash_only) {
                try {
                    std::cout << "FPGA backplane pinout: " << get_from_tree_int(tree, i, "imgparam/backplane_pinout") << std::endl;

                    std::cout << "FPGA DDR in use: " << get_from_tree_int(tree, i, "imgparam/ddr_used") << std::endl;

                    std::cout << "FPGA is hps only image: " << get_from_tree_int(tree, i, "imgparam/hps_only") << std::endl;

                    std::cout << "FPGA build number of rx channel: " << get_from_tree_int(tree, i, "imgparam/num_rx") << std::endl;

                    std::cout << "FPGA build number of tx channel: " << get_from_tree_int(tree, i, "imgparam/num_tx") << std::endl;

                    std::cout << "FPGA compiled for rtm: " << get_from_tree_int(tree, i, "imgparam/rtm") << std::endl;
                } catch (const uhd::lookup_error&) {
                    std::cout << "FPGA build parameter lookup not implemented (Only relevant on Cyan)" << std::endl;
                }
            }
        }

        if(rx_info) {
            size_t num_rx_channels = dev->get_rx_num_channels();
            std::cout << num_rx_channels << " rx channels should present on the unit" << std::endl;
            for(size_t rx_chan = 0; rx_chan < num_rx_channels; rx_chan++) {
                try {
                    char path[50];
                    sprintf(path, "rx/%lu/eeprom", rx_chan);
                    std::string eeprom = get_from_tree(tree, i, path);
                    std::cout << std::string("\trx(" + std::to_string(rx_chan) + ") EEPROM: ").c_str() << eeprom << std::endl;

                    sprintf(path, "rx/%lu/fw_version", rx_chan);
                    std::string version = get_from_tree(tree, i, path);
                    if(git_hash_only) {
                        version = extract_git_hash(version);
                    }

                    std::cout << std::string("\trx(" + std::to_string(rx_chan) + "): ").c_str() << version << std::endl << std::endl;
                    if(!git_hash_only) {
                        try {
                            std::vector<std::string> sensor_list = dev->get_rx_sensor_names(rx_chan);
                            for(size_t n = 0; n < sensor_list.size(); n++) {
                                uhd::sensor_value_t sensor_value = dev->get_rx_sensor(sensor_list[n], rx_chan);
                                std::cout << "\trx(" << std::to_string(rx_chan) << "): " << sensor_value.to_pp_string() << std::endl;
                            }
                        } catch (...) {}

                        try {
                            sprintf(path, "rx/%lu/jesd/status", rx_chan);
                            std::cout << std::string("\trx(" + std::to_string(rx_chan) + ") JESD status: ").c_str() << get_from_tree(tree, i, path) << std::endl;
                        } catch (...) {}
                        try {
                            sprintf(path, "rx/%lu/status/lna", rx_chan);
                            std::cout << std::string("\trx(" + std::to_string(rx_chan) + ") lna status: ").c_str() << get_from_tree(tree, i, path) << std::endl;
                        } catch (...) {}
                    }
                } catch (...) {
                }
            }
        }

        if(tx_info) {
            size_t num_tx_channels = dev->get_tx_num_channels();
            std::cout << num_tx_channels << " tx channels should be present on the unit" << std::endl;
            for(size_t tx_chan = 0; tx_chan < num_tx_channels; tx_chan++) {

                try {
                    char path[50];
                    sprintf(path, "tx/%lu/eeprom", tx_chan);
                    std::string eeprom = get_from_tree(tree, i, path);
                    std::cout << std::string("\ttx(" + std::to_string(tx_chan) + ") EEPROM: ").c_str() << eeprom << std::endl;

                    sprintf(path, "tx/%lu/fw_version", tx_chan);
                    std::string version = get_from_tree(tree, i, path);
                    if(git_hash_only) {
                        version = extract_git_hash(version);
                    }
                    std::cout << std::string("\ttx(" + std::to_string(tx_chan) + "): ").c_str() << version << std::endl << std::endl;

                    if(!git_hash_only) {
                        try {
                        std::vector<std::string> sensor_list = dev->get_tx_sensor_names(tx_chan);
                            for(size_t n = 0; n < sensor_list.size(); n++) {
                                uhd::sensor_value_t sensor_value = dev->get_tx_sensor(sensor_list[n], tx_chan);
                                std::cout << "\ttx(" << std::to_string(tx_chan) << "): " << sensor_value.to_pp_string() << std::endl;
                            }
                        } catch (...) {}

                        try {
                            sprintf(path, "tx/%lu/jesd/status", tx_chan);
                            std::cout << std::string("\ttx(" + std::to_string(tx_chan) + ") JESD status: ").c_str() << get_from_tree(tree, i, path) << std::endl;
                        } catch (...) {}
                    }
                } catch (...) {
                }
            }
        }

        try {
            if (network_info) {
                std::cout << "Device Address : " << std::endl;
                std::cout << "\tManagement IP: " << device_address << std::endl;
                std::cout << "\tSFP A IP     : " << get_from_tree(tree, i, "sfpa/ip_addr")
                      << std::endl;
                std::cout << "\tSFP B IP     : " << get_from_tree(tree, i, "sfpb/ip_addr")
                        << std::endl;
                std::cout << "Maximum SFP link rate: " << get_from_tree_double(tree, i, "link_max_rate") << std::endl;
            }
        } catch (...) {
            std::cout << "Unable to get all network info" << std::endl;
        }

        try {
            if (time_info) {
                std::string eeprom = get_from_tree(tree, i, "time/eeprom");
                std::cout << "Time EEPROM: " << eeprom << std::endl;

                std::cout << "Board MCU revision: " << std::endl;
                std::string version = get_from_tree(tree, i, "time/fw_version");
                if(git_hash_only) {
                    version = extract_git_hash(version);
                }
                std::cout << "\tTime : " << version << std::endl;

                try {
                    std::vector<std::string> sensor_list = dev->get_mboard_sensor_names(0);
                    for(size_t n = 0; n < sensor_list.size(); n++) {
                        uhd::sensor_value_t sensor_value = dev->get_mboard_sensor(sensor_list[n]);
                        std::cout << "\tTime: " << sensor_value.to_pp_string() << std::endl;
                    }
                } catch (...) {}

                try {
                    std::cout << "\tTime: reference: " << dev->get_clock_source(0) << std::endl;
                } catch (...) {}

                // Sets the property in order to update it
                tree->access<std::string>("/mboards/0/gps_time").set("1");
                std::cout << "Current time (fpga/gps_time) : " << get_from_tree(tree, i,"gps_time") << std::endl;
                std::cout << "Last time set (time/curr_time): " << get_from_tree_time_spec(tree, i,"time/now") << std::endl;
            }
        } catch (...) {
            std::cout << "Unable to get all time info" << std::endl;
        }

        i++;
    }

    std::cout << std::endl;

    return EXIT_SUCCESS;
}
