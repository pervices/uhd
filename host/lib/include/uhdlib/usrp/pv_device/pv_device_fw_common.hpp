//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/property_tree.hpp>

// fs_path is not a literal type, so it cannot be constexpr.
inline const uhd::fs_path PV_DEVICE_MB_PATH{"/mboards/0"};
inline const uhd::fs_path PV_DEVICE_TIME_PATH{PV_DEVICE_MB_PATH / "time"};
