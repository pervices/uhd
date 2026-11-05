//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

/*
 * Versioned argument structs for the trigger API.
 *
 * ABI rules for every struct in this file:
 *  - `size` must remain the first member. It is set by the default member
 *    initializer, which the compiler runs in the *caller's* code, so it records
 *    sizeof() as the caller was compiled. The library uses it to tell which
 *    version of the struct it received and, for arrays, as the element stride.
 *  - Only ever append new fields. Never reorder, resize, or remove existing ones.
 *  - Every new field's default must preserve the behaviour of older versions.
 *  - Do not add a user-declared constructor, virtual functions, or members that
 *    are not trivially copyable. A user-declared constructor must never be
 *    defined out of line in the library: it would write the library's sizeof()
 *    into a caller's possibly smaller object.
 *  - `min_size` is sizeof() of the first released version. It is frozen and must
 *    never change, even when fields are appended.
 *  - Padding is written out explicitly as `reserved*` fields so the layout is
 *    visible in the source. Reserved fields must be zero; the library rejects
 *    non-zero values so they can be given a meaning later.
 *
 * ABI rules for every enum in this file:
 *  - Each enum has a fixed underlying type, so its size never changes.
 *  - Values are explicit. Never renumber or reuse a value; only add new ones.
 */

namespace uhd { namespace trigger_pv {

/***********************************************************************
 * Flag operators
 **********************************************************************/
// Defines |, &, ~, |=, &= and has_any() for a flag enum so values combine
// without casts. All inline and compiled into the caller: no library ABI impact.
#define UHD_TRIGGER_PV_FLAG_OPERATORS(E)                                          \
    constexpr E operator|(E a, E b)                                              \
    {                                                                            \
        using U = std::underlying_type_t<E>;                                     \
        return static_cast<E>(static_cast<U>(a) | static_cast<U>(b));            \
    }                                                                            \
    constexpr E operator&(E a, E b)                                              \
    {                                                                            \
        using U = std::underlying_type_t<E>;                                     \
        return static_cast<E>(static_cast<U>(a) & static_cast<U>(b));            \
    }                                                                            \
    constexpr E operator~(E a)                                                   \
    {                                                                            \
        using U = std::underlying_type_t<E>;                                     \
        return static_cast<E>(~static_cast<U>(a));                               \
    }                                                                            \
    constexpr E& operator|=(E& a, E b)                                           \
    {                                                                            \
        return a = a | b;                                                        \
    }                                                                            \
    constexpr E& operator&=(E& a, E b)                                           \
    {                                                                            \
        return a = a & b;                                                        \
    }                                                                            \
    /*! True if any bit in f is set, e.g. has_any(sel & E::some_flag) */         \
    constexpr bool has_any(E f)                                                  \
    {                                                                            \
        return static_cast<std::underlying_type_t<E>>(f) != 0;                   \
    }

/***********************************************************************
 * Enums
 **********************************************************************/

//! Direction of a trigger pin.
enum class direction_t : uint32_t {
    get = 0, //!< Get the direction, do not set
    in             = 1, //!< Pin is an input. Does nothing on devices without a toggleable direction
    out            = 2, //!< Pin is an output. Does nothing on devices without a toggleable direction
};

//! Which signal level counts as an active trigger.
enum class polarity_t : uint32_t {
    get = 0, //!< Get the existing value
    positive = 1, //!< Active high.
    negative = 2, //!< Active low.
};

//! How a trigger input is interpreted.
enum class trigger_mode_t : uint32_t {
    get = 0,
    edge  = 1, //!< Trigger on an edge. TODO: document which edge(s).
    level = 2, //!< Stream while the trigger is active.
};

//! Which triggers a channel listens to. Values may be combined with |.
// Unlike other enums, the raw number is passed to the SDR
enum class active_trigger_t : uint32_t {
    none        = 0,       //!< Listen to no trigger.
    global_sma  = 1u << 0, //!< Listen to the global SMA trigger.
    channel_ufl = 1u << 1, //!< Listen to the per-channel U.FL trigger.
    get = 1u << 31; //! Get the trigger, do not set
    // Add new triggers as new bits only.
};
UHD_TRIGGER_PV_FLAG_OPERATORS(active_trigger_t)

//! Trigger gating.
//! TODO: document what each option gates.
enum class gating_t : uint32_t {
    get = 0,
    dsp  = 1, //!< TODO: document.
    output = 2, //!< TODO: document.
};

//! How to behave while the trigger is being configured. Values may be combined with |.
enum class setup_mode_t : uint64_t {
    none = 0, //!< Default behaviour. TODO: document.
    /*!
     * Whe hot configure is not enabled for unspecified channels: they will be disabled by having their trig_sel set to none all other setting are applied.
     */
    hot_configure_unspecified_rx = 1u << 0,
    hot_configure_unspecified_tx = 1u << 1,
};
UHD_TRIGGER_PV_FLAG_OPERATORS(setup_mode_t)

//! What to do if configuring the trigger fails part way through.
enum class failure_handling_t : uint64_t {
    disable_all = 0, //!< Disable triggering on all channels. TODO: confirm.
    // TODO: add other failure handling modes.
};

/*!
 * Categories of strict error checking. Values may be combined with |.
 *
 * Unknown bits are ignored rather than rejected, so `all` from an older
 * header also enables checks added in later library versions.
 */
enum class strict_error_t : uint64_t {
    none                      = 0,
    coerced_value = 1u << 0, //!< Error when a value is coerced
    //! Every check that existed in v1. This is the default, so checks added
    //! later are off unless the caller asks for them (or for `all`).
    all_v1 = coerced_value,
    all    = ~uint64_t{0},
};
UHD_TRIGGER_PV_FLAG_OPERATORS(strict_error_t)

/*!
 * Categories of warnings that can be disabled. Values may be combined with |.
 *
 * Unknown bits are ignored rather than rejected, so `all` from an older
 * header also disables warnings added in later library versions.
 */
enum class warning_t : uint64_t {
    none                      = 0,
    coerced_value = 1u << 1, //!< Warn when a value is coerced.
    all                       = ~uint64_t{0},
};
UHD_TRIGGER_PV_FLAG_OPERATORS(warning_t)

#undef UHD_TRIGGER_PV_FLAG_OPERATORS

/***********************************************************************
 * Structs
 **********************************************************************/

/*!
 * How to apply a trigger configuration change
 * (e.g. whether to disable the trigger while changes are made).
 */
struct trigger_op_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 32;

    //! Set automatically to the caller's sizeof(trigger_op_t). Do not modify.
    uint32_t size = sizeof(trigger_op_t);

    //! How to behave while setting up the trigger.
    setup_mode_t setup_mode = setup_mode_t::none;

    //! How to handle failures while attempting to configure the trigger.
    failure_handling_t failure_handling = failure_handling_t::disable_all;

    /*!
     * Ignore all other arguments and attempt to return the device to its
     * default non-trigger streaming state.
     */
    bool restore_default = false;

    //! Explicit padding. Must be zero.
    uint8_t reserved0[3] = {};

    //! OR together flags to enable stricter error checking.
    //! Defaults to every v1 check; checks added after v1 default to off.
    strict_error_t enable_strict_errors = strict_error_t::all_v1;

    //! OR together flags to disable warnings.
    warning_t disable_warnings = warning_t::none;

    // --- end of v1: append new fields below this line only ---
};

/*!
 * Trigger arguments that apply to every channel.
 */
struct trigger_global_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 16;

    //! Set automatically to the caller's sizeof(trigger_global_t). Do not modify.
    uint32_t size = sizeof(trigger_global_t);

    /*!
     * Whether the global trigger pin is an input or output.
     * Use direction_t::not_applicable on devices with separate pins.
     * When set to output it outputs its internal PPS. TODO: confirm this.
     */
    direction_t sma_direction = direction_t::in;

    //! Whether the global trigger is active high or active low.
    polarity_t sma_polarity = polarity_t::negative;

    /*!
     * Override the global trigger so that it acts as if the input is active.
     * TODO: verify it does not affect the output.
     */
    int8_t sma_override = -1;

    //! Explicit padding. Must be zero.
    uint8_t reserved0[3] = {};

    // --- end of v1: append new fields below this line only ---
};

/*!
 * Per-channel RX trigger arguments.
 */
struct trigger_rx_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 48;

    //! Value of `channel` when the caller has not set it. Rejected by the library.
    static constexpr size_t channel_unset = SIZE_MAX;

    //! Set automatically to the caller's sizeof(trigger_rx_t). Do not modify.
    uint32_t size = sizeof(trigger_rx_t);

    /*!
     * How many ticks to wait after an edge before triggering again.
     * Mainly used for debouncing.
     * A negative value passed to UHD indicates get only, do not change.
     * A negative value returned by UHD indicates an error
     * The value passed to the SDR is a uint32_t
     * TODO: check behaviour if a second edge is received during the backoff.
     */
    int64_t edge_backoff = -1;

    //! The physical channel this struct applies to. Must be set by the caller.
    size_t channel = channel_unset;

    /*!
     * The number of samples to stream per edge in edge triggered mode.
     * 0 means stream indefinitely after the trigger is received.
     * The device takes uint64_t
     * In UHD negative values are used to indicate errors when returns from UHD and get only when passed to UHD.
     * TODO: verify this does nothing in level mode.
     */
    int64_t edge_sample_num = -1;

    //! Which triggers to listen to. OR together to listen to several.
    active_trigger_t trig_sel = active_trigger_t::none;

    /*!
     * Whether to trigger on level or edge on the global SMA.
     * Not applicable unless trig_sel includes active_trigger_t::global_sma.
     */
    trigger_mode_t sma_mode = trigger_mode_t::level;

    /*!
     * Whether to trigger on level or edge on the per-channel U.FL.
     * Not applicable unless trig_sel includes active_trigger_t::channel_ufl.
     */
    trigger_mode_t ufl_mode = trigger_mode_t::level;

    /*!
     * Direction of the per-channel U.FL.
     * When set to input, the U.FL is an input used in acc.
     * TODO: verify out is a good default.
     */
    direction_t ufl_dir = direction_t::out;

    bool hot_configure = false;

    /*!
     *
     * Valid values:
     * -1: get
     * 0: disable
     * 1: enable
     * This is only implemented on rx
     * Must be set to enabled to restore normal streaming without creating a new streamer
     */
    int8_t enable = -1;

    //! Explicit tail padding (struct is 8-byte aligned). Must be zero.
    uint32_t reserved0 = 0;

    // --- end of v1: append new fields below this line only ---
};

/*!
 * Per-channel TX trigger arguments.
 */
struct trigger_tx_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 40;

    //! Value of `channel` when the caller has not set it. Rejected by the library.
    static constexpr size_t channel_unset = SIZE_MAX;

    //! Set automatically to the caller's sizeof(trigger_tx_t). Do not modify.
    uint32_t size = sizeof(trigger_tx_t);

    /*!
     * How many ticks to wait after an edge before triggering again.
     * Mainly used for debouncing.
     * A negative value passed to UHD indicates get only, do not change.
     * A negative value returned by UHD indicates an error
     * The value passed to the SDR is a uint32_t
     * TODO: check behaviour if a second edge is received during the backoff.
     */
    int64_t edge_backoff = -1;

    //! The physical channel this struct applies to. Must be set by the caller.
    size_t channel = channel_unset;

    /*!
     * The number of samples to stream per edge in edge triggered mode.
     * 0 means stream indefinitely after the trigger is received.
     * The device takes uint64_t
     * In UHD negative values are used to indicate errors when returns from UHD and get only when passed to UHD.
     * TODO: verify this does nothing in level mode.
     */
    int64_t edge_sample_num = -1;

    //! Which triggers to listen to. OR together to listen to several.
    active_trigger_t trig_sel = active_trigger_t::none;

    /*!
     * Whether to trigger on level or edge on the global SMA.
     * Not applicable unless trig_sel includes active_trigger_t::global_sma.
     */
    trigger_mode_t sma_mode = trigger_mode_t::level;

    /*!
     * Whether to trigger on level or edge on the per-channel U.FL.
     * Not applicable unless trig_sel includes active_trigger_t::channel_ufl.
     */
    trigger_mode_t ufl_mode = trigger_mode_t::level;

    /*!
     * Direction of the per-channel U.FL.
     * When set to input, the U.FL is an input used in acc.
     * TODO: verify out is a good default.
     */
    direction_t ufl_dir = direction_t::out;

    //! Keep the channel enabled while configuring it
    bool hot_configure = false;

    //! Trigger gating. TODO: document.
    gating_t gating = gating_t::dsp;

    // --- end of v1: append new fields below this line only ---
};

/***********************************************************************
 * Compile-time checks
 **********************************************************************/
 // TODO: re-enable once the struct layout is finalized
// // bool members assume a 1-byte bool (true on all supported ABIs).
// static_assert(sizeof(bool) == 1, "trigger structs assume a 1-byte bool");
// // The size_t `channel` members assume a 64-bit target; on a 32-bit target the
// // layout (and therefore min_size) would differ.
// static_assert(sizeof(size_t) == 8, "trigger structs assume a 64-bit size_t");

// // Required so the library can copy a caller's struct with memcpy and so that
// // offsetof() is well-defined. LAST is the last field of the v1 layout.
// #define UHD_TRIGGER_PV_CHECK_LAYOUT(T, LAST)
//     static_assert(std::is_standard_layout_v<T>, #T " must be standard layout");
//     static_assert(std::is_trivially_copyable_v<T>, #T " must be trivially copyable");
//     static_assert(offsetof(T, size) == 0, #T "::size must be the first member");
//     /* Any user-declared constructor (C++20) makes T a non-aggregate. This
//      * forbids one entirely, which rules out a constructor defined out of line
//      * in the library writing the library's sizeof() into a caller's object. */
//     static_assert(std::is_aggregate_v<T>, #T " must not declare a constructor");
//     /* Catches fields inserted or resized inside the v1 region. */
//     static_assert(offsetof(T, LAST) + sizeof(T::LAST) <= T::min_size,
//         #T " v1 fields no longer fit in min_size")

// UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_op_t, disable_warnings);
// UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_global_t, reserved0);
// UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_rx_t, reserved0);
// UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_tx_t, gating);

// #undef UHD_TRIGGER_PV_CHECK_LAYOUT

// // Verifies min_size matches the v1 layout.
// // DELETE these once the structs are released: they will (correctly) fail as
// // soon as a field is appended.
// static_assert(sizeof(trigger_op_t) == trigger_op_t::min_size);
// static_assert(sizeof(trigger_global_t) == trigger_global_t::min_size);
// static_assert(sizeof(trigger_rx_t) == trigger_rx_t::min_size);
// static_assert(sizeof(trigger_tx_t) == trigger_tx_t::min_size);

}} // namespace uhd::trigger_pv
