//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/device.hpp>
#include <cstdint>
#include <stdexcept>

namespace uhd {

/*!
 * Base class for Per Vices devices (Crimson, Cyan and the devices based on
 * them). Sits between uhd::device and the device implementations so that
 * multi_usrp_impl can reach device-specific behaviour that uhd::device does not
 * provide in a way that is easy to maintain ABI compatibility.
 *
 * multi_usrp_impl holds a uhd::device::sptr; use
 * dynamic_cast<pv_device*>(dev.get()) to get this interface. The cast will
 * always work for Per Vices's products and fail for any other.
 *
 * This exists to make achieving ABI compatibility easier, since device.hpp is
 * exposed to the public.
 *
 * TODO: when updating to the next UHD version (where breaking ABI changes are
 * expected) reset device.hpp to upstream and move the variables/functions here.
 */
class pv_device : public device
{
public:
    ~pv_device() override = default;
};

} // namespace uhd
