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

// The minimum server versions to support various features
// See common.h in firmware to see what they mean
constexpr int TRIGGER_OVERHAUL_SERVER_R3_S = 20261119;
constexpr int TRIGGER_OVERHAUL_SERVER_R2_S = 20261112;
constexpr int TRIGGER_OVERHAUL_SERVER_R1_S = 20261111;