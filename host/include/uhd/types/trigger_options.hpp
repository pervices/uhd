//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/exception.hpp>
#include <cstddef>
#include <cstdint>
#include <format>
#include <type_traits>
#include <vector>

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
enum class pin_direction_t : uint32_t {
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
    global_sma_in = 1u << 0, //!< Listen to the global SMA trigger.
    channel_ufl = 1u << 1, //!< Listen to the per-channel U.FL trigger.
    global_sma_out = 1u << 1, //!< Output to the the global SMA trigger. Every channel set to output is ORed together.
    get         = 1u << 31, //!< Get the current value, do not set.
    // Add new triggers as new bits only.
};
UHD_TRIGGER_PV_FLAG_OPERATORS(active_trigger_t)

//! Trigger gating.
// TODO: document what gating does when sma/ufl set to output, it probably only applies to input triggers
enum class gating_t : uint32_t {
    get = 0,
    dsp  = 1, //!< Consume samples only when the trigger active. Start/stop/time to send are ANDed with the trigger. i.e. if you send a burst with a start time of 5s then the first trigger at or after 5s with consume the first edge_sample_num samples from the burst.
    output = 2, //!< Consume samples in the same way as normal streaming, except the output is blanked when the trigger is not active.
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
    // TODO: before release, remove coerced_value from all_v1 so coercion
    // errors are off by default. Kept on during development.
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
     * When set to output it outputs its internal PPS. TODO: confirm this.
     */
    pin_direction_t sma_direction = pin_direction_t::get;

    //! Whether the global trigger is active high or active low.
    polarity_t sma_polarity = polarity_t::get;

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
    active_trigger_t trig_sel = active_trigger_t::get;

    /*!
     * Whether to trigger on level or edge on the global SMA.
     * Not applicable unless trig_sel includes active_trigger_t::global_sma.
     */
    trigger_mode_t sma_mode = trigger_mode_t::get;

    /*!
     * Whether to trigger on level or edge on the per-channel U.FL.
     * Not applicable unless trig_sel includes active_trigger_t::channel_ufl.
     */
    trigger_mode_t ufl_mode = trigger_mode_t::get;

    /*!
     * Direction of the per-channel U.FL.
     * When set to input, the U.FL is an input used in acc.
     * TODO: verify out is a good default.
     */
    pin_direction_t ufl_dir = pin_direction_t::get;

    //! Whether the per-channel U.FL trigger is active high or active low.
    polarity_t ufl_polarity = polarity_t::get;

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
    active_trigger_t trig_sel = active_trigger_t::get;

    /*!
     * Whether to trigger on level or edge on the global SMA.
     * Not applicable unless trig_sel includes active_trigger_t::global_sma.
     */
    trigger_mode_t sma_mode = trigger_mode_t::get;

    /*!
     * Whether to trigger on level or edge on the per-channel U.FL.
     * Not applicable unless trig_sel includes active_trigger_t::channel_ufl.
     */
    trigger_mode_t ufl_mode = trigger_mode_t::get;

    /*!
     * Direction of the per-channel U.FL.
     * When set to input, the U.FL is an input used in acc.
     * TODO: verify out is a good default.
     */
    pin_direction_t ufl_dir = pin_direction_t::get;

    //! Whether the per-channel U.FL trigger is active high or active low.
    polarity_t ufl_polarity = polarity_t::get;

    //! Keep the channel enabled while configuring it
    bool hot_configure = false;

    //! Trigger gating. TODO: document.
    gating_t gating = gating_t::get;

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

/***********************************************************************
 * Preset configurations
 *
 * These fill out the arguments for multi_usrp::trigger_configure() in the
 * caller's struct format.
 *
 * They are defined here in the header, not in a .cpp, so that they are
 * compiled into the caller and fill the structs using the caller's layout. A
 * version compiled into the library would write the library's layout into the
 * caller's (possibly smaller) structs.
 *
 * `static` gives each file that includes this header its own copy, so two
 * libraries built against different versions of this header can never end up
 * sharing one copy (and one struct layout) at runtime.
 **********************************************************************/

/*!
 * Fill out arguments that read every trigger setting without changing anything.
 *
 * Every setting is set to get, and every channel (specified or not) is hot
 * configured, so nothing is disabled during the reads. After trigger_configure()
 * returns, the structs hold the values currently on the device.
 *
 * Element i of rx and tx reads physical channel i, so pass arrays with
 * get_rx_num_channels() and get_tx_num_channels() elements to read every channel.
 *
 * When adding a field to a struct, set it to get here.
 *
 * \param op filled in; must not be nullptr
 * \param global filled in, or nullptr to skip reading the global settings
 * \param rx filled in; may be nullptr if rx_len is 0
 * \param rx_len number of elements in rx
 * \param tx filled in; may be nullptr if tx_len is 0
 * \param tx_len number of elements in tx
 * 	hrows uhd::value_error if a required pointer is null
 */
static inline void get_read_config(trigger_op_t* op,
    trigger_global_t* global,
    trigger_rx_t* rx,
    size_t rx_len,
    trigger_tx_t* tx,
    size_t tx_len)
{
    if (op == nullptr) {
        throw uhd::value_error("get_read_config: op must not be null");
    }
    if (rx == nullptr && rx_len != 0) {
        throw uhd::value_error(std::format("get_read_config: rx is null but rx_len is {}", rx_len));
    }
    if (tx == nullptr && tx_len != 0) {
        throw uhd::value_error(std::format("get_read_config: tx is null but tx_len is {}", tx_len));
    }

    *op            = trigger_op_t{};
    op->setup_mode = setup_mode_t::hot_configure_unspecified_rx
                     | setup_mode_t::hot_configure_unspecified_tx;

    if (global != nullptr) {
        *global               = trigger_global_t{};
        global->sma_direction = pin_direction_t::get;
        global->sma_polarity  = polarity_t::get;
        global->sma_override  = -1;
    }

    for (size_t channel_index = 0; channel_index < rx_len; channel_index++) {
        trigger_rx_t& chan   = rx[channel_index];
        chan                 = trigger_rx_t{};
        chan.channel         = channel_index;
        chan.hot_configure   = true;
        chan.edge_backoff    = -1;
        chan.edge_sample_num = -1;
        chan.trig_sel        = active_trigger_t::get;
        chan.sma_mode        = trigger_mode_t::get;
        chan.ufl_mode        = trigger_mode_t::get;
        chan.ufl_dir         = pin_direction_t::get;
        chan.ufl_polarity    = polarity_t::get;
        chan.enable          = -1;
    }

    for (size_t channel_index = 0; channel_index < tx_len; channel_index++) {
        trigger_tx_t& chan   = tx[channel_index];
        chan                 = trigger_tx_t{};
        chan.channel         = channel_index;
        chan.hot_configure   = true;
        chan.edge_backoff    = -1;
        chan.edge_sample_num = -1;
        chan.trig_sel        = active_trigger_t::get;
        chan.sma_mode        = trigger_mode_t::get;
        chan.ufl_mode        = trigger_mode_t::get;
        chan.ufl_dir         = pin_direction_t::get;
        chan.ufl_polarity    = polarity_t::get;
        chan.gating          = gating_t::get;
    }
}

/*!
 * Fill out arguments that do the same as the older rx_trigger_setup() and
 * tx_trigger_setup() functions:
 *  - edge triggering on the global SMA, positive polarity, no edge backoff,
 *    and \p edge_sample_num samples per edge
 *  - RX channels stream once configured (enable = 1)
 *  - TX channels use DSP gating
 *  - the global SMA direction is set to \p sma_direction
 *
 * As with the older functions, other settings and unspecified channels are left
 * unchanged, and coerced values (e.g. edge_sample_num rounded by the device) are
 * adjusted rather than treated as errors. The actual values are written back
 * into the structs by trigger_configure().
 *
 * \param op filled in; must not be nullptr
 * \param global filled in; must not be nullptr
 * \param rx filled in; may be nullptr if rx_len is 0
 * \param rx_len number of elements in rx
 * \param tx filled in; may be nullptr if tx_len is 0
 * \param tx_len number of elements in tx
 * \param rx_channels physical channel for each element of rx; must have rx_len elements
 * \param tx_channels physical channel for each element of tx; must have tx_len elements
 * \param sma_direction pin_direction_t::in or pin_direction_t::out
 * \param edge_sample_num samples to stream per edge; 0 streams indefinitely; must be >= 0
 * \throws uhd::value_error on an invalid request
 */
static inline void get_simple_sma_config(trigger_op_t* op,
    trigger_global_t* global,
    trigger_rx_t* rx,
    size_t rx_len,
    trigger_tx_t* tx,
    size_t tx_len,
    const std::vector<size_t>& rx_channels,
    const std::vector<size_t>& tx_channels,
    const pin_direction_t sma_direction,
    const int64_t edge_sample_num)
{
    if (op == nullptr || global == nullptr) {
        throw uhd::value_error("get_simple_sma_config: op and global must not be null");
    }
    if (rx == nullptr && rx_len != 0) {
        throw uhd::value_error(
            std::format("get_simple_sma_config: rx is null but rx_len is {}", rx_len));
    }
    if (tx == nullptr && tx_len != 0) {
        throw uhd::value_error(
            std::format("get_simple_sma_config: tx is null but tx_len is {}", tx_len));
    }
    if (rx_channels.size() != rx_len || tx_channels.size() != tx_len) {
        throw uhd::value_error(std::format(
            "get_simple_sma_config: {} rx_channels for {} rx elements, {} tx_channels for {} tx elements",
            rx_channels.size(), rx_len, tx_channels.size(), tx_len));
    }
    if (sma_direction != pin_direction_t::in && sma_direction != pin_direction_t::out) {
        throw uhd::value_error("get_simple_sma_config: sma_direction must be in or out");
    }
    if (edge_sample_num < 0) {
        throw uhd::value_error(std::format(
            "get_simple_sma_config: edge_sample_num must be >= 0, got {}", edge_sample_num));
    }

    *op            = trigger_op_t{};
    op->setup_mode = setup_mode_t::hot_configure_unspecified_rx
                     | setup_mode_t::hot_configure_unspecified_tx;
    // The older functions accepted whatever the device applied.
    op->enable_strict_errors &= ~strict_error_t::coerced_value;

    *global               = trigger_global_t{};
    global->sma_direction = sma_direction;
    global->sma_polarity  = polarity_t::positive;
    global->sma_override  = -1;

    for (size_t channel_index = 0; channel_index < rx_len; channel_index++) {
        trigger_rx_t& chan   = rx[channel_index];
        chan                 = trigger_rx_t{};
        chan.channel         = rx_channels[channel_index];
        chan.hot_configure   = false;
        chan.edge_backoff    = 0;
        chan.edge_sample_num = edge_sample_num;
        chan.trig_sel        = active_trigger_t::global_sma;
        chan.sma_mode        = trigger_mode_t::edge;
        chan.enable          = 1;
        // ufl_mode, ufl_dir and ufl_polarity keep their defaults (get).
    }

    for (size_t channel_index = 0; channel_index < tx_len; channel_index++) {
        trigger_tx_t& chan   = tx[channel_index];
        chan                 = trigger_tx_t{};
        chan.channel         = tx_channels[channel_index];
        chan.hot_configure   = false;
        chan.edge_backoff    = 0;
        chan.edge_sample_num = edge_sample_num;
        chan.trig_sel        = active_trigger_t::global_sma;
        chan.sma_mode        = trigger_mode_t::edge;
        chan.gating          = gating_t::dsp;
        // ufl_mode, ufl_dir and ufl_polarity keep their defaults (get).
    }
}

}} // namespace uhd::trigger_pv
