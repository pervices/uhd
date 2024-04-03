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
 * \return a vecotr containing the governors in use by each core
 */
std::vector<std::string> get_performance_governors(void);

}; /* namespace uhd */
