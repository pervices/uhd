//
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2019 Ettus Research, A National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/config.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/paths.hpp>
#include <uhdlib/utils/paths.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <vector>

BOOST_AUTO_TEST_CASE(test_paths_expandvars)
{
#ifdef UHD_PLATFORM_WIN32
    const std::string path_to_expand("%programdata%/uhd/uhd.conf");
#else
    const std::string path_to_expand("$HOME/.uhd/uhd.conf");
#endif
    const std::string expanded_path = uhd::path_expandvars(path_to_expand);

    std::cout << "Expanded path: " << path_to_expand << " -> " << expanded_path
              << std::endl;
    BOOST_CHECK(path_to_expand != expanded_path);

#ifdef UHD_PLATFORM_WIN32
    const std::string var_identifier = "%";
#else
    const std::string var_identifier = "$";
#endif

    BOOST_CHECK(expanded_path.find(var_identifier) == std::string::npos);
}


BOOST_AUTO_TEST_CASE(test_get_paths)
{
    using namespace uhd;

    std::cout << "tmp_path: " << get_tmp_path() << std::endl;
    BOOST_CHECK(true);
    std::cout << "pkg_path: " << get_pkg_path() << std::endl;
    BOOST_CHECK(true);
    std::cout << "pkg_data_path: " << get_pkg_data_path() << std::endl;
    BOOST_CHECK(true);
    std::cout << "cal_path: " << get_cal_data_path() << std::endl;
    BOOST_CHECK(true);

    const auto module_paths = get_module_paths();
    for (const auto& module_path : module_paths) {
        std::cout << "module path: " << module_path << std::endl;
    }
    BOOST_CHECK(true);

    const std::string images_dir_search_path = "";
    std::cout << "images_dir: " << get_images_dir(images_dir_search_path) << std::endl;
    BOOST_REQUIRE_THROW(
        find_image_path("this_device_does_not_exist.bit", ""), uhd::io_error);

    const std::string utility_path = find_utility("uhd_images_downloader");
    std::cout << "utility_path: " << utility_path << std::endl;

    const std::string utility_error =
        print_utility_error("uhd_images_downloader", "--help");
    std::cout << "utility_error: " << utility_error << std::endl;
}
