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
 *    sizeof() as the caller was compiled.
 *  - ONLY APPEND NEW FIELDS. Never reorder, resize, or remove existing ones.
 *  - Every new field's default behaviour must preserve the behaviour of older versions.
 *  - Do not add a user-declared constructor, virtual functions, or members that
 *    are not trivially copyable. A user-declared constructor must never be
 *    defined out of line in the library: it would write the library's sizeof()
 *    into a caller's possibly smaller object.
 *  - Padding is written out explicitly as `reserved*` fields so the layout is
 *    visible in the source. Reserved fields must be zero; the library rejects
 *    non-zero values so they can be given a meaning later.
 *
 * ABI rules for every enum in this file:
 *  - Each enum has a fixed underlying type, so its size never changes.
 *  - Values are explicit. Never renumber or reuse a value; only add new ones.
 */

namespace uhd { namespace trigger_pv {

/*
 * Fields that use these constants are int64_t, even ones that are essentially a
 * bool. This allows negative values below the 32 bit signed int limit to
 * unambiguously make requests without risk of them being mistaken for error codes.
 */

/*!
 * Request a get When passed supported fields
 */
constexpr int64_t GET_INT = -2147483649;
/*!
 * When passed supported fields;
 * SKIP_IF_NOT_RELEVANT indicates do nothing if the field is not relevant, value error if it is.
 * When deciding whether a field is relevant, dependent fields set to get or bypass are ignored,
 * even if the value from get indicates it would be relevant
 */
constexpr int64_t SKIP_IF_NOT_RELEVANT = -2147483650;

/***********************************************************************
 * Flag operators
 **********************************************************************/
// Defines operators for a flag enum so values combine without casts.
// Must be inline and compiled into the caller for ABI compatibility
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

/*!
 * Direction of a trigger pin.
 *
 * A pin is one of:
 *  - toggleable: one pin that can be switched between input and output
 *  - separate: separate input and output pins, so there is no direction to set
 *  - input-only: a single pin that can only be an input
 *  - output-only: a single pin that can only be an output
 */
enum class pin_direction_t : uint32_t {
    /*!
     * No direction specified. Always throws an error if other fields in the
     * request make the pin relevant. See the documentation for each pin for when
     * it counts as relevant.
     * If the pin is not relevant, nothing is changed and this value is returned
     * unchanged.
     * \internal
     * Use as the default for pins whose direction cannot always be derived when
     * they are relevant. Pins that can always be derived when relevant should
     * default to derive instead.
     * \endinternal
     */
    must_specify_if_applicable = 0,

    //! Get the current direction without changing it.
    get = 1,

    /*!
     * The pin is an input.
     * Always throws an error on an output-only pin.
     * On an input-only pin, succeeds without changing anything.
     * On separate pins:
     *     Throws an error if pedantic_dir is set in enable_strict_errors.
     *     Otherwise it is treated as get.
     *     Warns if pedantic_dir is not set in disable_warnings.
     * On a toggleable pin, always throws an error if this conflicts with the
     * direction required by other fields in the same trigger_configure call.
     */
    in = 2,

    /*!
     * The pin is an output.
     * Always throws an error on an input-only pin.
     * On an output-only pin, succeeds without changing anything.
     * On separate pins:
     *     Throws an error if pedantic_dir is set in enable_strict_errors.
     *     Otherwise it is treated as get.
     *     Warns if pedantic_dir is not set in disable_warnings.
     * On a toggleable pin, always throws an error if this conflicts with the
     * direction required by other fields in the same trigger_configure call.
     */
    out = 3,

    /*!
     * Derive the direction from the per-channel settings of the channels in the
     * request. Only usable for pins
     * that can be determined from other fields
     *
     * Channels not in the request, and channels whose trig_sel is
     * active_trigger_t::get or active_trigger_t::bypass, are ignored.
     *
     * See the documentation for each pin to check whether derive is recommended
     * or unsupported. Always throws an error if used for a pin that does not
     * support derive.
     * Always throws an error if the fields used to derive the direction
     * conflict. Separate pins can never conflict.
     * This value is always returned unchanged, so a struct returned by
     * trigger_configure can be edited and passed in again without fixing the
     * direction to whatever was derived this time.
     *
     * If no direction can be determined from the other fields, or the pin is not
     * toggleable, nothing is changed.
     * \internal
     * Use as the default for pins that can always be derived when they are
     * relevant.
     * \endinternal
     */
    derive = 4,

    /*!
     * The device has separate input and output pins.
     * On separate pins, succeeds without changing anything.
     * On any other pin, throws a uhd::value_error.
     */
    separate = 5,
};

//! Which signal level counts as an active trigger. Applies to both inputs and outputs.
enum class polarity_t : uint32_t {
    get      = 0, //!< Get the existing value
    positive = 1, //!< Active high.
    negative = 2, //!< Active low.
    /*!
     * Do not read or write this field; it is returned as is.
     * Fields that depend on it do not count as relevant.
     */
    bypass = 1u << 31,
};

// TODO: add descriptions of what counts as an edge to instances
//! How a trigger input/output is interpreted.
enum class trigger_mode_t : uint32_t {
    get = 0, //!< Get the existing value
    /*!
     * Activate the trigger on edge for a duration specified in other fields.
     * In input mode activate on a positive or negative edge depending on polarity.
     * In output mode activate when streaming starts.
     */
    edge = 1,
    /*!
     * In input mode activate continuously when the signal matches the polarity.
     * In output mode activate while streaming is ongoing.
     */
    level = 2,
    /*!
     * Do not read or write this field; it is returned as is.
     * Fields that depend on it do not count as relevant.
     */
    bypass = 1u << 31,
};

/*!
 * Which triggers a channel listens/outputs on. Values may be combined with |.
 * Triggers are processed independently. Multiple input triggers are ANDed together at the end.
 * Multiple output triggers are ORed together at the end.
 * e.g. if:
 *  - rx stream commands say to stream 850 samples starting at the time of sample 400,
 *  - edge_sample_num is 1000 and both triggers are edge triggered,
 *  - the sma input receives an edge at the time of sample 500,
 *  - and the ufl receives an edge at the time of sample 1000,
 * then streaming only occurs from the time of sample 1000 to 1249.
 * \internal
 * On Cyan the raw bits can be passed through.
 * On the device in Crimson sma_output/sma_input are controlled by bit 0
 * and input vs output is controlled by sma_dir
 * \endinternal
 */
enum class active_trigger_t : uint32_t {
    none        = 0,       //!< Listen to no trigger/send to no output.
    sma_input   = 1u << 0, //!< Listen to the global SMA trigger.
    channel_ufl = 1u << 1, //!< Listen/output to the per-channel U.FL trigger depending on ufl_dir.
    /*!
     * Output to the global SMA trigger.
     * The final output is every channel's individual output ORed together.
     * Cannot be combined with sma_input on the same channel.
     */
    sma_output = 1u << 2,
    /*!
     * Skip error checking and always return as is.
     * Invalid if combined with anything else.
     * Not counted as relevant for fields that depend on it.
     * Use for compatibility purposes only.
     */
    bypass = 1u << 30,
    //! Get the current value, do not set. Invalid if combined with anything else.
    get = 1u << 31,
    // Add new triggers as new bits only.
};
UHD_TRIGGER_PV_FLAG_OPERATORS(active_trigger_t)

//! Trigger gating.
enum class gating_t : uint32_t {
    get = 0, //!< Get the existing value
    dsp = 1, //!< Consume samples only while the trigger is active.
    /*!
     * Samples are consumed the same way as when not using triggers (based on send times).
     * The final output is blanked while triggers are not active.
     */
    output = 2,
};

//! How to behave while the trigger is being configured.
enum class setup_mode_t : uint64_t {
    /*!
     * Default behaviour.
     * Disable all unspecified channels.
     * Disable all channels without hot_configure while applying the configuration
     * (even if every field is get).
     */
    standard = 0,
};
UHD_TRIGGER_PV_FLAG_OPERATORS(setup_mode_t)

/*!
 * What to attempt on the device if trigger_configure fails.
 * Not used for uhd::index_error or uhd::value_error, which are thrown during
 * validation, before anything on the device is changed, so nothing is attempted.
 * See multi_usrp::trigger_configure for details about what errors mean.
 * In every case the structs passed to trigger_configure are not modified. The
 * attempt is not guaranteed to succeed, so the device is in an undefined state
 * regardless of this setting.
 * \internal
 * uhd::io_error from the SMA direction read during validation triggers this
 * despite being in the validation stage, to avoid needing to throw an index/value
 * error for a possible networking error.
 * \endinternal
 */
enum class failure_handling_t : uint64_t {
    /*!
     * Attempt to disable all channels (including channels not specified) and set
     * all toggleable pins to input. trig_sel is not changed.
     */
    disable_all_after_validation = 0,
    /*!
     * Attempt nothing further. The requested changes may be partially applied,
     * and any channel (including channels not specified) may be left disabled
     * if hot_configure was not set for it.
     */
    flexible = 1,
};

/*!
 * Categories of strict error checking. Values may be combined with |.
 *
 * Unknown bits are ignored rather than rejected, so `all` from an older
 * header also enables checks added in later library versions.
 */
enum class strict_error_t : uint64_t {
    none = 0,
    /*!
     * Throw an error instead of coercing a value: uhd::value_error if the need
     * to coerce is detected during validation, uhd::runtime_error if it is only
     * detected after the value has been applied.
     * \internal
     * Currently every coercion is done by the server, so this always results in
     * uhd::runtime_error. value_error is allowed in case detection is added
     * within UHD in the future.
     * \endinternal
     */
    coerced_value = 1u << 0,
    /*!
     * Throw uhd::value_error, before anything is changed on the device, when the
     * requested pin direction does not match the pin's type, and is not a control value.
     */
    pedantic_dir = 2u << 1,
    /*!
     * Throw uhd::runtime_error if the server version is not supported by this
     * version of UHD. This flag can be removed for compatibility, but features may
     * be missing and you lose the guarantees that prevent accidental trigger activations.
     */
    unsupported_server = 2u << 2,
    /*!
     * Default error flags for the initial release version.
     * Flags added later are not enabled by default for compatibility.
     * \internal
     * Every initial error except coerced_value is enabled by default for high default strictness.
     * New errors are left off by default to avoid breaking existing programs.
     * \endinternal
     */
    default_v1 = pedantic_dir | unsupported_server,
    all = ~uint64_t{0},
};
UHD_TRIGGER_PV_FLAG_OPERATORS(strict_error_t)

/*!
 * Categories of warnings that can be disabled. Values may be combined with |.
 *
 * Unknown bits are ignored rather than rejected, so `all` from an older
 * header also disables warnings added in later library versions.
 */
enum class warning_t : uint64_t {
    none = 0,
    /*!
     * Warns when a value is coerced.
     */
    coerced_value = 1u << 0,
    /*!
     * Warn when a requested pin direction does not match the pin's type.
     */
    pedantic_dir = 2u << 1,
    /*!
     * Warn if the server version is not supported by this
     * version of UHD. Features may
     * be missing and you lose the guarantees that prevent accidental trigger activations.
     */
    unsupported_server = 2u << 2,
};
UHD_TRIGGER_PV_FLAG_OPERATORS(warning_t)

#undef UHD_TRIGGER_PV_FLAG_OPERATORS

/***********************************************************************
 * Structs
 **********************************************************************/

/*!
 * How to apply a trigger configuration change
 * (e.g. how to behave when an error occurs).
 */
struct trigger_op_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 40;

    //! Set automatically to the caller's sizeof(trigger_op_t). Do not modify.
    uint32_t size = sizeof(trigger_op_t);

    //! Explicit padding. Must be zero.
    uint32_t reserved0 = 0;

    //! How to behave while setting up the trigger.
    setup_mode_t setup_mode = setup_mode_t::standard;

    //! How to handle failures while attempting to configure the trigger.
    failure_handling_t failure_handling = failure_handling_t::disable_all_after_validation;

    //! OR together flags to ENABLE stricter error checking.
    strict_error_t enable_strict_errors = strict_error_t::default_v1;

    //! OR together flags to DISABLE warnings.
    warning_t disable_warnings = warning_t::none;

    // --- end of v1: append new fields below this line only ---
};

/*!
 * Trigger arguments that apply to every channel.
 */
struct trigger_global_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 32;

    //! Set automatically to the caller's sizeof(trigger_global_t). Do not modify.
    uint32_t size = sizeof(trigger_global_t);

    /*!
     * Direction of the global SMA trigger pin.
     * Crimson and Calamine have a single toggleable SMA pin. Cyan and Chestnut
     * have separate SMA input and output pins.
     * Recommended to use pin_direction_t::derive to keep code portable between these devices.
     * On a toggleable pin, if this is pin_direction_t::get (or no global settings
     * are passed), the current direction is read and the channels in the request
     * are validated against it.
     * Counts as relevant if any channel in the same trigger_configure call has
     * active_trigger_t::sma_input or active_trigger_t::sma_output set in trig_sel.
     * Channels whose trig_sel is active_trigger_t::get or active_trigger_t::bypass
     * are ignored.
     */
    pin_direction_t sma_direction = pin_direction_t::derive;

    //! Whether the global trigger is active high or active low, applies to both input and output.
    polarity_t sma_polarity = polarity_t::get;

    //! Explicit padding. Must be zero.
    uint32_t reserved0 = 0;

    /*!
     * Override the global trigger so that it acts as if the input is active.
     * Supported values: GET_INT, 0 (disable), 1 (enable)
     * A returned value between -1 and -2,147,483,648 indicates an error.
     */
    int64_t sma_override = GET_INT;

    /*!
     * How long to keep the sma trigger high in units of samples when a channel starts streaming and is edge triggered.
     * This argument applies to all channels
     * Supported values:
     * Any positive value is a regular value.
     * 0 indicates keep the output active forever.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if any channel in the same
     * trigger_configure call has active_trigger_t::sma_output in trig_sel and
     * sma_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * Servers with an API version older than 20261112 do not support this field.
     * With them a SKIP_IF_NOT_RELEVANT value that is not relevant is still skipped,
     * GET_INT returns -ENOSYS if strict_error_t::unsupported_server is enabled,
     * and anything else throws uhd::runtime_error.
     * \internal
     * The device takes uint64_t, negative values are purely within UHD
     * The version is TRIGGER_OVERHAUL_SERVER_R2_S.
     * \endinternal
     */
    int64_t sma_output_pulse_length = SKIP_IF_NOT_RELEVANT;

    // --- end of v1: append new fields below this line only ---
};

/*!
 * Per-channel RX trigger arguments.
 */
struct trigger_rx_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 72;

    //! Value of `channel` when the caller has not set it. Rejected by the library.
    static constexpr size_t channel_unset = SIZE_MAX;

    //! Set automatically to the caller's sizeof(trigger_rx_t). Do not modify.
    uint32_t size = sizeof(trigger_rx_t);

    //! Explicit padding. Must be zero.
    uint32_t reserved0 = 0;

    /*!
     * After an edge, ignore every edge for this many ticks of the device clock.
     * It is used for debouncing. Rarely needs to be changed. No effect in level mode.
     * Supported values:
     * Any non-negative value is a regular value.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if trig_sel includes
     * active_trigger_t::sma_input with sma_mode set to edge, or
     * active_trigger_t::channel_ufl with ufl_dir set to in and ufl_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * \internal
     * The device takes uint32_t, negative values are purely within UHD.
     * \endinternal
     */
    int64_t edge_backoff = GET_INT;

    //! The physical channel this struct applies to.
    size_t channel = channel_unset;

    /*!
     * How long to stream in units of samples when an edge triggered input trigger is activated.
     * Applies to both ufl and sma triggers independently.
     * Supported values:
     * Any positive value is a regular value.
     * 0 means stream indefinitely after the trigger is received.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if trig_sel includes
     * active_trigger_t::sma_input with sma_mode set to edge, or
     * active_trigger_t::channel_ufl with ufl_dir set to in and ufl_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * \internal
     * The device takes uint64_t, negative values are purely within UHD.
     * \endinternal
     */
    int64_t input_edge_sample_num = SKIP_IF_NOT_RELEVANT;

    /*!
     * How long to keep the ufl trigger high in units of samples when the channel starts streaming and is edge triggered.
     * Supported values:
     * Any positive value is a regular value.
     * 0 indicates keep the output active forever.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if trig_sel includes
     * active_trigger_t::channel_ufl with ufl_dir set to out and ufl_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * Servers with an API version older than 20261112 do not support this field.
     * With them a SKIP_IF_NOT_RELEVANT value that is not relevant is still skipped,
     * GET_INT returns -ENOSYS if strict_error_t::unsupported_server is enabled,
     * and anything else throws uhd::runtime_error.
     * \internal
     * The device takes uint64_t, negative values are purely within UHD
     * The version is TRIGGER_OVERHAUL_SERVER_R2_S.
     * \endinternal
     */
    int64_t ufl_output_pulse_length = SKIP_IF_NOT_RELEVANT;

    //! Which triggers to listen/output to/from. OR together to listen/output to/from several.
    active_trigger_t trig_sel = active_trigger_t::get;

    /*!
     * Edge or level behaviour on the global SMA, applies to both input and output.
     * Not relevant unless trig_sel includes active_trigger_t::sma_input or active_trigger_t::sma_output.
     */
    trigger_mode_t sma_mode = trigger_mode_t::get;

    /*!
     * Edge or level behaviour on the per-channel U.FL, applies to both input and output.
     * Not relevant unless trig_sel includes active_trigger_t::channel_ufl.
     */
    trigger_mode_t ufl_mode = trigger_mode_t::get;

    /*!
     * Direction of the per-channel ufl.
     * Relevant for the purposes of pin_direction_t if trig_sel includes active_trigger_t::channel_ufl.
     * ufl_dir cannot be derived from other fields. Must be specified, get, or not relevant.
     */
    pin_direction_t ufl_dir = pin_direction_t::must_specify_if_applicable;

    //! Whether the per-channel U.FL trigger is active high or active low, as an input or an output.
    polarity_t ufl_polarity = polarity_t::get;

    /*!
     * Keep the channel enabled while the configuration is being applied.
     * If false, the channel is disabled and reset during configuration.
     * Any commands/samples will be lost.
     */
    bool hot_configure = false;

    //! Explicit padding. Must be zero.
    uint8_t reserved1[3] = {};

    /*!
     * Whether the channel is enabled.
     * Must be set to 1 to enable streaming.
     * Note that creating/destroying a streamer has the same effect as enable/disable.
     * Supported values: GET_INT, 0 (disable), 1 (enable)
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * \internal
     * Corresponds to rx/<ch>/stream on the server
     * \endinternal
     */
    int64_t enable = GET_INT;

    // --- end of v1: append new fields below this line only ---
};

/*!
 * Per-channel TX trigger arguments.
 */
struct trigger_tx_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 80;

    //! Value of `channel` when the caller has not set it. Rejected by the library.
    static constexpr size_t channel_unset = SIZE_MAX;

    //! Set automatically to the caller's sizeof(trigger_tx_t). Do not modify.
    uint32_t size = sizeof(trigger_tx_t);

    //! Explicit padding. Must be zero.
    uint32_t reserved0 = 0;

    /*!
     * After an edge, ignore every edge for this many ticks of the device clock.
     * Exists for trigger sources that bounce. Rarely needs to be changed.
     * Supported values:
     * Any non-negative value is a regular value.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if trig_sel includes
     * active_trigger_t::sma_input with sma_mode set to edge, or
     * active_trigger_t::channel_ufl with ufl_dir set to in and ufl_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * TODO: figure out if this does anything in level mode
     * \internal
     * The device takes uint32_t, negative values are purely within UHD.
     * \endinternal
     */
    int64_t edge_backoff = GET_INT;

    //! The physical channel this struct applies to. Must be set by the caller.
    size_t channel = channel_unset;

    /*!
     * How long to stream in units of samples when an edge triggered input trigger is activated.
     * Applies to both ufl and sma triggers.
     * Supported values:
     * Any positive value is a regular value.
     * 0 means stream indefinitely after the trigger is received.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if trig_sel includes
     * active_trigger_t::sma_input with sma_mode set to edge, or
     * active_trigger_t::channel_ufl with ufl_dir set to in and ufl_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * \internal
     * The device takes uint64_t, negative values are purely within UHD.
     * \endinternal
     */
    int64_t input_edge_sample_num = SKIP_IF_NOT_RELEVANT;

    /*!
     * How long to keep the ufl trigger high in units of samples when the channel starts streaming and is edge triggered.
     * Supported values:
     * Any positive value is a regular value.
     * 0 indicates keep the output active forever.
     * GET_INT.
     * SKIP_IF_NOT_RELEVANT.
     * Counts as relevant for SKIP_IF_NOT_RELEVANT if trig_sel includes
     * active_trigger_t::channel_ufl with ufl_dir set to out and ufl_mode set to edge.
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * Servers with an API version older than 20261112 do not support this field.
     * With them a SKIP_IF_NOT_RELEVANT value that is not relevant is still skipped,
     * GET_INT returns -ENOSYS if strict_error_t::unsupported_server is enabled,
     * and anything else throws uhd::runtime_error.
     * \internal
     * The device takes uint64_t, negative values are purely within UHD
     * This field requires TRIGGER_OVERHAUL_SERVER_R2_S or later.
     * \endinternal
     */
    int64_t ufl_output_pulse_length = SKIP_IF_NOT_RELEVANT;

    //! Which triggers to listen/output to/from. OR together to listen/output to/from several.
    active_trigger_t trig_sel = active_trigger_t::get;

    /*!
     * Edge or level behaviour on the global SMA, applies to both input and output.
     * Not relevant unless trig_sel includes active_trigger_t::sma_input or active_trigger_t::sma_output.
     */
    trigger_mode_t sma_mode = trigger_mode_t::get;

    /*!
     * Edge or level behaviour on the per-channel U.FL, applies to both input and output.
     * Not relevant unless trig_sel includes active_trigger_t::channel_ufl.
     */
    trigger_mode_t ufl_mode = trigger_mode_t::get;

    /*!
     * Direction of the per-channel ufl.
     * Relevant for the purposes of pin_direction_t if trig_sel includes active_trigger_t::channel_ufl.
     * ufl_dir cannot be derived from other fields. Must be specified, get, or not relevant.
     */
    pin_direction_t ufl_dir = pin_direction_t::must_specify_if_applicable;

    //! Whether the per-channel U.FL trigger is active high or active low, applies to both input and output.
    polarity_t ufl_polarity = polarity_t::get;

    /*!
     * Keep the channel enabled while the configuration is being applied.
     * If false, the channel is disabled during configuration, which resets it:
     * any samples, SOB, or EOB from before are lost.
     * Servers with an API version older than 20261119 cannot disable TX channels
     * without losing their RF settings, so TX channels are never disabled with
     * them (false behaves like true) and a warning_t::unsupported_server
     * warning is logged. Spurious activations may occur while configuring.
     * \internal
     * This field requires TRIGGER_OVERHAUL_SERVER_R3_S or later.
     * \endinternal
     */
    bool hot_configure = false;

    //! Explicit padding. Must be zero.
    uint8_t reserved1[3] = {};

    /*!
     * Whether the channel is enabled.
     * Must be set to 1 to enable streaming.
     * Note that creating/destroying a streamer has the same effect as enable/disable.
     * Supported values: GET_INT, 0 (disable), 1 (enable)
     * A returned value between -1 and -2,147,483,648 indicates an error.
     * Servers with an API version older than 20261119 cannot disable TX channels
     * so spurious activations may occurs while applying triggers.
     * \internal
     * Corresponds to tx/<ch>/stream on servers TRIGGER_OVERHAUL_SERVER_R3_S or newer.
     * \endinternal
     */
    int64_t enable = GET_INT;

    /*!
     * Set gating. See gating_t for options. Only relevant with input triggers.
     *
     * IMPORTANT: call disable_blocking_fc on the tx streamer for any channel using
     * dsp gating.
     */
    gating_t gating = gating_t::get;

    //! Explicit tail padding (struct is 8-byte aligned). Must be zero.
    uint32_t reserved2 = 0;

    // --- end of v1: append new fields below this line only ---
};

/***********************************************************************
 * Compile-time checks
 **********************************************************************/
// bool members assume a 1-byte bool (true on all supported ABIs).
static_assert(sizeof(bool) == 1, "trigger structs assume a 1-byte bool");
// The size_t `channel` members assume a 64-bit target; on a 32-bit target the
// layout (and therefore min_size) would differ.
static_assert(sizeof(size_t) == 8, "trigger structs assume a 64-bit size_t");

// Required so the library can copy a caller's struct with memcpy and so that
// offsetof() is well-defined. LAST is the last field of the v1 layout.
#define UHD_TRIGGER_PV_CHECK_LAYOUT(T, LAST)                                           \
    static_assert(std::is_standard_layout_v<T>, #T " must be standard layout");       \
    static_assert(std::is_trivially_copyable_v<T>, #T " must be trivially copyable"); \
    static_assert(offsetof(T, size) == 0, #T "::size must be the first member");      \
    /* Any user-declared constructor (C++20) makes T a non-aggregate. This            \
     * forbids one entirely, which rules out a constructor defined out of line        \
     * in the library writing the library's sizeof() into a caller's object. */       \
    static_assert(std::is_aggregate_v<T>, #T " must not declare a constructor");      \
    /* Catches fields inserted or resized inside the v1 region. */                    \
    static_assert(offsetof(T, LAST) + sizeof(T::LAST) <= T::min_size,                 \
        #T " v1 fields no longer fit in min_size")

UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_op_t, disable_warnings);
UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_global_t, sma_output_pulse_length);
UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_rx_t, enable);
UHD_TRIGGER_PV_CHECK_LAYOUT(trigger_tx_t, reserved2);

#undef UHD_TRIGGER_PV_CHECK_LAYOUT

/***********************************************************************
 * Preset configurations
 *
 * These fill out the arguments for multi_usrp::trigger_configure() in the
 * caller's struct format.
 *
 * They must be inline so they follow the caller's format, not the library's
 *
 * `static` gives each file that includes this header its own copy, so two
 * libraries built against different versions of this header can never end up
 * sharing one copy (and one struct layout) at runtime.
 **********************************************************************/

/*!
 * Helper functions for presets, do not call directly
 */
namespace detail {

/*!
 * Throw uhd::value_error if a preset's array arguments are inconsistent.
 * `preset` is only used in error messages.
 */
template <typename channel_t>
static inline void check_preset_array(const char* preset,
    const char* name,
    const channel_t* array,
    size_t len,
    size_t expected_len)
{
    if (array == nullptr && len != 0) {
        throw uhd::value_error(
            std::format("{}: {} is null but {}_len is {}", preset, name, name, len));
    }
    if (len != expected_len) {
        throw uhd::value_error(std::format(
            "{}: {}_len is {} but the channel lists have {} channels in total",
            preset, name, len, expected_len));
    }
}

//! Number of channels in an optional list of preserved channels.
static inline size_t preserved_count(const std::vector<size_t>* preserved)
{
    return (preserved == nullptr) ? 0 : preserved->size();
}

/*!
 * Fill one RX entry so trigger_configure() leaves the channel exactly as it is:
 * the channel is hot configured and every field is get, except
 * ufl_output_pulse_length, which is SKIP_IF_NOT_RELEVANT.
 */
static inline void fill_untouched(trigger_rx_t& chan, size_t channel)
{
    chan                         = trigger_rx_t{};
    chan.channel                 = channel;
    chan.hot_configure           = true;
    chan.edge_backoff            = GET_INT;
    chan.input_edge_sample_num         = GET_INT;
    // Never relevant while trig_sel is get, so the device is not accessed for it.
    chan.ufl_output_pulse_length = SKIP_IF_NOT_RELEVANT;
    chan.trig_sel                = active_trigger_t::get;
    chan.sma_mode                = trigger_mode_t::get;
    chan.ufl_mode                = trigger_mode_t::get;
    chan.ufl_dir                 = pin_direction_t::get;
    chan.ufl_polarity            = polarity_t::get;
    chan.enable                  = GET_INT;
}

//! TX version of fill_untouched().
static inline void fill_untouched(trigger_tx_t& chan, size_t channel)
{
    chan                         = trigger_tx_t{};
    chan.channel                 = channel;
    chan.hot_configure           = true;
    chan.edge_backoff            = GET_INT;
    chan.input_edge_sample_num         = GET_INT;
    // Never relevant while trig_sel is get, so the device is not accessed for it.
    chan.ufl_output_pulse_length = SKIP_IF_NOT_RELEVANT;
    chan.trig_sel                = active_trigger_t::get;
    chan.sma_mode                = trigger_mode_t::get;
    chan.ufl_mode                = trigger_mode_t::get;
    chan.ufl_dir                 = pin_direction_t::get;
    chan.ufl_polarity            = polarity_t::get;
    chan.enable                  = GET_INT;
    chan.gating                  = gating_t::get;
}

/*!
 * Fill array[first], array[first + 1], ... with untouched entries for the
 * channels in `preserved` (which may be nullptr).
 */
template <typename channel_t>
static inline void fill_preserved(
    channel_t* array, size_t first, const std::vector<size_t>* preserved)
{
    if (preserved == nullptr) {
        return;
    }
    for (size_t i = 0; i < preserved->size(); i++) {
        fill_untouched(array[first + i], (*preserved)[i]);
    }
}

} // namespace detail

/*!
 * Fill out arguments that read every trigger setting without changing anything on listed channels.
 * Channels not in the list will be disabled.
 *
 * Everything is set to get, and every channel specified is hot
 * configured, so no settings are changed.
 * After trigger_configure() returns, the structs hold the values currently on the device.
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
 * \throws uhd::value_error if a required pointer is null
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
    detail::check_preset_array("get_read_config", "rx", rx, rx_len, rx_len);
    detail::check_preset_array("get_read_config", "tx", tx, tx_len, tx_len);

    *op = trigger_op_t{};

    if (global != nullptr) {
        *global                         = trigger_global_t{};
        global->sma_direction           = pin_direction_t::get;
        global->sma_polarity            = polarity_t::get;
        global->sma_override            = GET_INT;
        global->sma_output_pulse_length = GET_INT;
    }

    // fill_untouched() hot configures the channel and sets every field to get
    // except ufl_output_pulse_length, which is read here too.
    for (size_t channel_index = 0; channel_index < rx_len; channel_index++) {
        detail::fill_untouched(rx[channel_index], channel_index);
        rx[channel_index].ufl_output_pulse_length = GET_INT;
    }
    for (size_t channel_index = 0; channel_index < tx_len; channel_index++) {
        detail::fill_untouched(tx[channel_index], channel_index);
        tx[channel_index].ufl_output_pulse_length = GET_INT;
    }
}

/*!
 * Fill out arguments that do the same as the older rx_trigger_setup() and
 * tx_trigger_setup() functions:
 *  - edge triggering on the global SMA, positive polarity, no edge backoff,
 *    and \p edge_sample_num samples per edge
 *  - the channels use the SMA as an input or output, as set by \p sma_direction
 *  - RX channels stream once configured (enable = 1)
 *  - TX channels use DSP gating
 *  - global->sma_direction is set to pin_direction_t::derive, so the same
 *    arguments work on devices with a single toggleable SMA pin and on devices
 *    with separate SMA input and output pins
 *  - sma_output_pulse_length is set to \p edge_sample_num in output mode, and
 *    skipped in input mode
 *  - input_edge_sample_num is set to \p edge_sample_num in input mode, and
 *    skipped in output mode
 * Elements 0 to rx_channels.size() - 1 of \p rx configure \p rx_channels, and
 * the remaining elements leave \p preserved_rx_channels unchanged. The same
 * applies to \p tx.
 *
 * Note that channels not included in either \p rx_channels , \p tx_channels ,
 * \p preserved_rx_channels , \p preserved_tx_channels will be disabled.
 *
 * \param op filled in; must not be nullptr
 * \param global filled in; must not be nullptr
 * \param rx filled in; may be nullptr if rx_len is 0
 * \param rx_len number of elements in rx. Must be the length of rx_channels +
 *        preserved_rx_channels.
 * \param tx filled in; may be nullptr if tx_len is 0
 * \param tx_len number of elements in tx. Must be the length of tx_channels +
 *        preserved_tx_channels.
 * \param rx_channels The rx channels we wish to apply the configuration to.
 * \param tx_channels The tx channels we wish to apply the configuration to.
 * \param sma_direction pin_direction_t::in for the channels to listen to the SMA
 *        (active_trigger_t::sma_input), pin_direction_t::out for them to drive it
 *        (active_trigger_t::sma_output). Any other value is invalid.
 * \param edge_sample_num samples to stream per edge (input), or how long to keep
 *        the output active (output); 0 means indefinitely; must be >= 0
 * \param preserved_rx_channels By default any channel not specified during
 *        trigger_configure is disabled. Channels listed here will not be
 *        impacted. May be nullptr.
 * \param preserved_tx_channels By default any channel not specified during
 *        trigger_configure is disabled. Channels listed here will not be
 *        impacted. May be nullptr.
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
    const int64_t edge_sample_num,
    const std::vector<size_t>* preserved_rx_channels,
    const std::vector<size_t>* preserved_tx_channels)
{
    if (op == nullptr || global == nullptr) {
        throw uhd::value_error("get_simple_sma_config: op and global must not be null");
    }
    detail::check_preset_array("get_simple_sma_config", "rx", rx, rx_len,
        rx_channels.size() + detail::preserved_count(preserved_rx_channels));
    detail::check_preset_array("get_simple_sma_config", "tx", tx, tx_len,
        tx_channels.size() + detail::preserved_count(preserved_tx_channels));
    if (sma_direction != pin_direction_t::in && sma_direction != pin_direction_t::out) {
        throw uhd::value_error("get_simple_sma_config: sma_direction must be in or out");
    }
    if (edge_sample_num < 0) {
        throw uhd::value_error(std::format(
            "get_simple_sma_config: edge_sample_num must be >= 0, got {}", edge_sample_num));
    }

    const active_trigger_t sma_trigger = (sma_direction == pin_direction_t::in)
                                             ? active_trigger_t::sma_input
                                             : active_trigger_t::sma_output;

    *op = trigger_op_t{};
    // The older functions accepted whatever the device applied.
    op->enable_strict_errors &= ~strict_error_t::coerced_value;

    *global                         = trigger_global_t{};
    global->sma_direction           = pin_direction_t::derive;
    global->sma_polarity            = polarity_t::positive;
    global->sma_override            = GET_INT;
    // Only relevant when the channels drive the SMA (edge mode). There is no
    // parameter for it, so in that case the device's existing value is kept.
    global->sma_output_pulse_length = (sma_direction == pin_direction_t::out)
                                          ? edge_sample_num
                                          : SKIP_IF_NOT_RELEVANT;

    for (size_t i = 0; i < rx_channels.size(); i++) {
        trigger_rx_t& chan   = rx[i];
        chan                 = trigger_rx_t{};
        chan.channel         = rx_channels[i];
        chan.hot_configure   = false;
        chan.edge_backoff    = 0;
        chan.input_edge_sample_num = (sma_direction == pin_direction_t::in)
                                        ? edge_sample_num
                                        : SKIP_IF_NOT_RELEVANT;
        chan.trig_sel        = sma_trigger;
        chan.sma_mode        = trigger_mode_t::edge;
        chan.enable          = 1;
        // The U.FL fields keep their defaults; they are not relevant without
        // active_trigger_t::channel_ufl.
    }
    detail::fill_preserved(rx, rx_channels.size(), preserved_rx_channels);

    for (size_t i = 0; i < tx_channels.size(); i++) {
        trigger_tx_t& chan   = tx[i];
        chan                 = trigger_tx_t{};
        chan.channel         = tx_channels[i];
        chan.hot_configure   = false;
        chan.edge_backoff    = 0;
        chan.input_edge_sample_num = (sma_direction == pin_direction_t::in)
                                        ? edge_sample_num
                                        : SKIP_IF_NOT_RELEVANT;
        chan.trig_sel        = sma_trigger;
        chan.sma_mode        = trigger_mode_t::edge;
        chan.gating          = gating_t::dsp;
        // enable stays get: the older functions did not change it.
    }
    detail::fill_preserved(tx, tx_channels.size(), preserved_tx_channels);
}

/*!
 * Fill out arguments to disable streaming and switch back to normal operations.
 * The direction of bidirectional pins is not touched.
 *
 * For every channel in \p rx_channels and \p tx_channels:
 *  - trig_sel is set to active_trigger_t::none
 *  - hot_configure is set to false
 *
 * Elements 0 to rx_channels.size() - 1 of \p rx return \p rx_channels to normal
 * operation, and the remaining elements leave \p preserved_rx_channels
 * unchanged. The same applies to \p tx.
 *
 * Note that channels not included in either \p rx_channels , \p tx_channels ,
 * \p preserved_rx_channels , \p preserved_tx_channels will be disabled.
 *
 * All other trigger fields are undefined and must be set as desired if you want to re-enable streaming.
 * Channel enable is not modified. If it is not 1, you must either set enable to 1 or create a new
 * streamer to stream again.
 * \internal
 * Leave all fields as is. The documentation says to leave all fields as undefined
 * to give us flexibility
 * \endinternal
 *
 * \param op filled in; must not be nullptr
 * \param global filled in; may be nullptr
 * \param rx filled in; may be nullptr if rx_len is 0
 * \param rx_len number of elements in rx. Must be the length of rx_channels + preserved_rx_channels.
 * \param tx filled in; may be nullptr if tx_len is 0
 * \param tx_len number of elements in tx. Must be the length of tx_channels + preserved_tx_channels.
 * \param rx_channels The rx channels to return to normal operation.
 * \param tx_channels The tx channels to return to normal operation.
 * \param preserved_rx_channels By default any channel not specified during trigger_configure is disabled. Channels listed here will not be impacted. May be nullptr.
 * \param preserved_tx_channels By default any channel not specified during trigger_configure is disabled. Channels listed here will not be impacted. May be nullptr.
 * \throws uhd::value_error if a required pointer is null, or the array lengths do not match the channel lists
 */
static inline void get_non_trigger_config(trigger_op_t* op,
    trigger_global_t* global,
    trigger_rx_t* rx,
    size_t rx_len,
    trigger_tx_t* tx,
    size_t tx_len,
    const std::vector<size_t>& rx_channels,
    const std::vector<size_t>& tx_channels,
    const std::vector<size_t>* preserved_rx_channels,
    const std::vector<size_t>* preserved_tx_channels)
{
    if (op == nullptr) {
        throw uhd::value_error("get_non_trigger_config: op must not be null");
    }
    detail::check_preset_array("get_non_trigger_config", "rx", rx, rx_len,
        rx_channels.size() + detail::preserved_count(preserved_rx_channels));
    detail::check_preset_array("get_non_trigger_config", "tx", tx, tx_len,
        tx_channels.size() + detail::preserved_count(preserved_tx_channels));

    *op = trigger_op_t{};

    if (global != nullptr) {
        // Every other global field keeps its default (get or skip if not relevant).
        *global               = trigger_global_t{};
        // Skip since it doesn't matter with channels disabled.
        global->sma_direction = pin_direction_t::must_specify_if_applicable;
    }

    // Every field not set here keeps its default: get, or skip if not relevant.
    for (size_t i = 0; i < rx_channels.size(); i++) {
        trigger_rx_t& chan = rx[i];
        chan               = trigger_rx_t{};
        chan.channel       = rx_channels[i];
        chan.hot_configure = false;
        chan.trig_sel      = active_trigger_t::none;
        chan.ufl_dir       = pin_direction_t::must_specify_if_applicable;
    }
    detail::fill_preserved(rx, rx_channels.size(), preserved_rx_channels);

    for (size_t i = 0; i < tx_channels.size(); i++) {
        trigger_tx_t& chan = tx[i];
        chan               = trigger_tx_t{};
        chan.channel       = tx_channels[i];
        chan.hot_configure = false;
        chan.trig_sel      = active_trigger_t::none;
        chan.ufl_dir       = pin_direction_t::must_specify_if_applicable;
    }
    detail::fill_preserved(tx, tx_channels.size(), preserved_tx_channels);
}

}} // namespace uhd::trigger_pv
