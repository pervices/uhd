//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/device.hpp>
#include <uhd/types/trigger_options.hpp>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

namespace uhd {

/*!
 * Base class for Per Vices devices (Crimson, Cyan and the devices based on
 * them). Sits between uhd::device and the device implementations so that
 * multi_usrp_impl can reach device-specific behaviour that uhd::device does not
 * provide.
 *
 * This header is internal to the library (not installed), so this class can
 * change freely without affecting the ABI.
 *
 * multi_usrp_impl holds a uhd::device::sptr; use
 * dynamic_cast<pv_device*>(dev.get()) to get this interface. The cast returns
 * nullptr for devices that are not Per Vices devices.
 */
class pv_device : public device
{
public:
    ~pv_device() override = default;

    /*!
     * Device-specific part of multi_usrp::trigger_configure().
     *
     * Validates the request against this device's trigger hardware and fills in
     * the derived_* structs with the values to apply to the device. Called once,
     * before anything is changed on the device, so an exception leaves the
     * device untouched.
     *
     * The derived_* structs start as deep copies of the lib_* structs. This
     * function changes them where the device's representation differs from the
     * API's. In particular, after this call derived_rx[i].trig_sel and
     * derived_tx[i].trig_sel hold the device's raw trig_sel bits, which do not
     * necessarily have the meanings given by trigger_pv::active_trigger_t (on
     * Crimson, bit 0 means SMA input or output depending on the SMA direction).
     * Use trig_sel_from_device() to translate them back.
     *
     * If lib_global is empty (the caller passed no global settings), the SMA
     * direction is never changed. Instead, the current direction is read into
     * derived_global, and the request is validated against it. Error messages
     * for this case must say that the existing direction was used because no
     * global settings were given.
     *
     * \param lib_op how to apply the change (library format)
     * \param lib_global global settings, or empty if the caller gave none
     * \param lib_rx per-channel RX settings (library format)
     * \param lib_tx per-channel TX settings (library format)
     * \param unspecified_rx physical RX channels not in lib_rx
     * \param unspecified_tx physical TX channels not in lib_tx
     * \param derived_global out: global settings to apply. Always filled in,
     *        even if lib_global is empty.
     * \param derived_rx out: one element per element of lib_rx, in the same order
     * \param derived_tx out: one element per element of lib_tx, in the same order
     * \throws uhd::value_error if the request is invalid on this device
     */
    virtual void trigger_derive(const trigger_pv::trigger_op_t& lib_op,
        const std::optional<trigger_pv::trigger_global_t>& lib_global,
        const std::vector<trigger_pv::trigger_rx_t>& lib_rx,
        const std::vector<trigger_pv::trigger_tx_t>& lib_tx,
        const std::vector<size_t>& unspecified_rx,
        const std::vector<size_t>& unspecified_tx,
        trigger_pv::trigger_global_t& derived_global,
        std::vector<trigger_pv::trigger_rx_t>& derived_rx,
        std::vector<trigger_pv::trigger_tx_t>& derived_tx)
    {
        (void) lib_op;
        (void) lib_global;
        (void) lib_rx;
        (void) lib_tx;
        (void) unspecified_rx;
        (void) unspecified_tx;
        (void) derived_global;
        (void) derived_rx;
        (void) derived_tx;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Translate a raw trig_sel value read from the device (as stored in the
     * derived_* structs) back to its trigger_pv::active_trigger_t meaning.
     *
     * \param device_trig_sel the device's raw trig_sel bits
     * \param derived_global the global settings in effect on the device
     *        (on Crimson, the SMA direction decides what bit 0 means)
     * \return trig_sel with the meanings given by trigger_pv::active_trigger_t
     */
    virtual trigger_pv::active_trigger_t trig_sel_from_device(
        trigger_pv::active_trigger_t device_trig_sel,
        const trigger_pv::trigger_global_t& derived_global)
    {
        (void) device_trig_sel;
        (void) derived_global;
        throw std::runtime_error("concrete classes are expected to override this method");
    }
};

} // namespace uhd
