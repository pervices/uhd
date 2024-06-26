//
// Copyright 2011,2014,2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once


#include <uhd/config.hpp>
#include <uhd/types/wb_iface.hpp>
#include <uhd/usrp/dboard_iface.hpp>
#include <uhd/usrp/gpio_defs.hpp>
#include <uhd/utils/noncopyable.hpp>
#include <uhdlib/usrp/gpio_defs.hpp>
#include <memory>


namespace uhd { namespace usrp { namespace gpio_atr {

struct gpio_atr_offsets
{
    uhd::wb_iface::wb_addr_type idle;
    uhd::wb_iface::wb_addr_type rx;
    uhd::wb_iface::wb_addr_type tx;
    uhd::wb_iface::wb_addr_type duplex;
    uhd::wb_iface::wb_addr_type ddr;
    uhd::wb_iface::wb_addr_type disable;
    uhd::wb_iface::wb_addr_type readback;

    /*!
     * Returns whether this GPIO regmap is write-only.
     *
     * \return whether the readback register is valid
     */
    bool is_writeonly() const;

    /*!
     * Create a GPIO regmap according to the typical "defaults": Four
     * sequential ATR registers followed immediately by, in order, duplex,
     * direction, disable, and an explicitly specified readback address.
     *
     * \param base base settings offset for GPIO ATR registers
     * \param rb_addr readback offset for GPIO ATR registers
     * \param stride Delta between the register addresses
     */
    static gpio_atr_offsets make_default(
        const uhd::wb_iface::wb_addr_type base,
        const uhd::wb_iface::wb_addr_type rb_addr,
        const size_t stride = 4);

    /*!
     * Create a GPIO regmap according to the typical "defaults" (see
     * make_default), with the readback register disabled.
     *

     * \param base base settings offset for GPIO ATR registers
     * \param rb_addr readback offset for GPIO ATR registers
     * \param stride Delta between the register addresses
     */
    static gpio_atr_offsets make_write_only(

        const uhd::wb_iface::wb_addr_type base,
        const size_t stride = 4);
};

class gpio_atr_3000 : uhd::noncopyable
{
public:
    typedef std::shared_ptr<gpio_atr_3000> sptr;

    static const uint32_t MASK_SET_ALL = 0xFFFFFFFF;

    virtual ~gpio_atr_3000(void) {}

    /*!
     * Create a GPIO ATR interface object using the given registers
     *
     * \param iface register iface to GPIO ATR registers
     * \param registers Register offsets
     */
    static sptr make(uhd::wb_iface::sptr iface, gpio_atr_offsets registers);


    /*!
     * Select the ATR mode for all bits in the mask
     *
     * \param mode the mode to apply {ATR = outputs driven by ATR state machine, GPIO =
     * outputs static} \param mask apply the mode to all non-zero bits in the mask
     */
    virtual void set_atr_mode(const gpio_atr_mode_t mode, const uint32_t mask) = 0;

    /*!
     * Select the data direction for all bits in the mask
     *
     * \param dir the direction {OUTPUT, INPUT}
     * \param mask apply the mode to all non-zero bits in the mask
     */
    virtual void set_gpio_ddr(const gpio_ddr_t dir, const uint32_t mask) = 0;

    /*!
     * Write the specified (masked) value to the ATR register
     *
     * \param atr the type of ATR register to write to {IDLE, RX, TX, FDX}
     * \param value the value to write
     * \param mask only writes to the bits where mask is non-zero
     */
    virtual void set_atr_reg(const gpio_atr_reg_t atr,
        const uint32_t value,
        const uint32_t mask = MASK_SET_ALL) = 0;

    /*!
     * Write to a static GPIO output
     *
     * \param value the value to write
     * \param mask only writes to the bits where mask is non-zero
     */
    virtual void set_gpio_out(
        const uint32_t value, const uint32_t mask = MASK_SET_ALL) = 0;

    /*!
     * Read the state of the GPIO pins
     * If a pin is configured as an input, reads the actual value of the pin
     * If a pin is configured as an output, reads the last value written to the pin
     *
     * \return the value read back
     */
    virtual uint32_t read_gpio() = 0;

    /*!
     * Get a GPIO attribute
     * This will likely returned a cached value, and not read the state from the physical
     * GPIO controller.
     *
     * \param attr the attribute to read
     * \return the current value of that attribute
     */
    virtual uint32_t get_attr_reg(const gpio_attr_t attr) = 0;

    /*!
     * Set a GPIO attribute
     *
     * \param attr the attribute to set
     * \param value the value to write to the attribute
     */
    virtual void set_gpio_attr(const gpio_attr_t attr, const uint32_t value) = 0;
};

class db_gpio_atr_3000
{
public:
    typedef std::shared_ptr<db_gpio_atr_3000> sptr;

    typedef uhd::usrp::dboard_iface::unit_t db_unit_t;

    virtual ~db_gpio_atr_3000(void) {}

    /*!
     * Create a GPIO ATR interface object for a daughterboard connector
     *
     * \param iface register iface to GPIO ATR registers
     * \param registers Register offsets

     */
    static sptr make(uhd::wb_iface::sptr iface, gpio_atr_offsets registers);


    /*!
     * Configure the GPIO mode for all pins in the daughterboard connector
     *
     * \param unit the side of the daughterboard interface to configure (TX or RX)
     * \param value if value[i] is 1, the i'th bit is in ATR mode otherwise it is in GPIO
     * mode \param mask mask
     */
    virtual void set_pin_ctrl(
        const db_unit_t unit, const uint32_t value, const uint32_t mask) = 0;

    virtual uint32_t get_pin_ctrl(const db_unit_t unit) = 0;

    /*!
     * Configure the direction for all pins in the daughterboard connector
     *
     * \param unit the side of the daughterboard interface to configure (TX or RX)
     * \param value if value[i] is 1, the i'th bit is an output otherwise it is an input
     * \param mask mask
     */
    virtual void set_gpio_ddr(
        const db_unit_t unit, const uint32_t value, const uint32_t mask) = 0;

    virtual uint32_t get_gpio_ddr(const db_unit_t unit) = 0;

    /*!
     * Write the specified value to the ATR register (all bits)
     *
     * \param unit the side of the daughterboard interface to configure (TX or RX)
     * \param atr the type of ATR register to write to {IDLE, RX, TX, FDX}
     * \param value the value to write
     * \param mask mask
     */
    virtual void set_atr_reg(const db_unit_t unit,
        const gpio_atr_reg_t atr,
        const uint32_t value,
        const uint32_t mask) = 0;

    virtual uint32_t get_atr_reg(const db_unit_t unit, const gpio_atr_reg_t atr) = 0;

    /*!
     * Write the specified value to the GPIO register (all bits)
     *
     * \param unit the side of the daughterboard interface to configure (TX or RX)
     * \param value the value to write
     * \param mask mask
     */
    virtual void set_gpio_out(
        const db_unit_t unit, const uint32_t value, const uint32_t mask) = 0;

    virtual uint32_t get_gpio_out(const db_unit_t unit) = 0;

    /*!
     * Read the state of the GPIO pins
     * If a pin is configured as an input, reads the actual value of the pin
     * If a pin is configured as an output, reads the last value written to the pin
     *
     * \param unit the side of the daughterboard interface to configure (TX or RX)
     * \return the value read back
     */
    virtual uint32_t read_gpio(const db_unit_t unit) = 0;
};

}}} // namespace uhd::usrp::gpio_atr
