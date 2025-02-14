//
// Copyright 2024 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <vector>
#include <string>

namespace uhd {

/*!
 * Get the performance governors in use by the system
 * \return a vector containing the governors in use by each core
 */
std::vector<std::string> get_performance_governors(void);

/*!
 * Checks if all cores are using the desired governor and prints a warning message if they are not
 * \param desired_governor The expected governor
 * \return a true if all cores are using the desired governor
 */
bool check_if_only_using_governor(std::string desired_governor = "performance");

}; /* namespace uhd */
