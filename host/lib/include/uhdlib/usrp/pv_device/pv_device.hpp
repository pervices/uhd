//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/device.hpp>
#include <uhd/types/trigger_options.hpp>
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

    /*!
     * The server's API version. Format: yyyymmdd
     * 0 indicates the server predates the property that provides this value.
     * Set during the child class's constructor.
     */
    int server_api_ver = 0;

    /*!
     * How many FPGA commits have been made in the FPGA.
     *
     * Used to track the FPGA version to check for features and bugs
     */
    int fpga_ver = 0;

    /***********************************************************************
     * Trigger hooks for multi_usrp::trigger_configure()
     *
     * The rules for pin directions, relevance and validation are implemented
     * once in multi_usrp_impl. These hooks only describe how this device's
     * hardware differs. None of them may access the device.
     **********************************************************************/

    //! The kinds of trigger pin described by trigger_pv::pin_direction_t.
    enum class trigger_pin_type_t {
        toggleable, //!< One pin that can be switched between input and output
        separate, //!< Separate input and output pins
        input_only, //!< A single pin that can only be an input
        output_only, //!< A single pin that can only be an output
    };

    //! The type of the global SMA trigger pin.
    virtual trigger_pin_type_t trigger_sma_pin_type() const
    {
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Translate a validated trig_sel to the raw bits the device expects.
     *
     * \param api_trig_sel trig_sel with the meanings given by
     *        trigger_pv::active_trigger_t. Never active_trigger_t::get or
     *        active_trigger_t::bypass, and never contains unknown bits.
     * \return the device's raw trig_sel bits
     */
    virtual uint32_t trig_sel_to_device(trigger_pv::active_trigger_t api_trig_sel) const
    {
        (void) api_trig_sel;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Translate raw trig_sel bits read from the device back to their
     * trigger_pv::active_trigger_t meaning.
     *
     * \param device_trig_sel the device's raw trig_sel bits
     * \param sma_direction the SMA direction in effect on the device: in or out on
     *        a toggleable SMA pin (it can decide what the bits mean), ignored for
     *        other pin types
     * \return trig_sel with the meanings given by trigger_pv::active_trigger_t.
     *         May contain unknown bits if the device reported them; the caller
     *         rejects those.
     * \throws uhd::runtime_error if sma_direction is needed but is not in or out
     */
    virtual trigger_pv::active_trigger_t trig_sel_from_device(
        uint32_t device_trig_sel, trigger_pv::pin_direction_t sma_direction) const
    {
        (void) device_trig_sel;
        (void) sma_direction;
        throw std::runtime_error("concrete classes are expected to override this method");
    }
};

} // namespace uhd
