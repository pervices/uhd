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
 */

namespace uhd { namespace trigger {

/*!
 * How to apply a trigger configuration change
 * (e.g. whether to disable the trigger while changes are made).
 */
struct trigger_op_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 12;

    //! Set automatically to the caller's sizeof(trigger_op_t). Do not modify.
    uint32_t size = sizeof(trigger_op_t);

    int32_t dummy1 = 0;
    int32_t dummy2 = 0;
    // --- end of v1: append new fields below this line only ---
};

/*!
 * Trigger arguments that apply to every channel.
 */
struct trigger_global_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 12;

    //! Set automatically to the caller's sizeof(trigger_global_t). Do not modify.
    uint32_t size = sizeof(trigger_global_t);

    int32_t dummy1 = 0;
    int32_t dummy2 = 0;
    // --- end of v1: append new fields below this line only ---
};

/*!
 * Per-channel RX trigger arguments.
 */
struct trigger_rx_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 12;

    //! Set automatically to the caller's sizeof(trigger_rx_t). Do not modify.
    uint32_t size = sizeof(trigger_rx_t);

    int32_t dummy1 = 0;
    int32_t dummy2 = 0;
    // --- end of v1: append new fields below this line only ---
};

/*!
 * Per-channel TX trigger arguments.
 */
struct trigger_tx_t
{
    //! sizeof() of the first released version. Never change.
    static constexpr uint32_t min_size = 12;

    //! Set automatically to the caller's sizeof(trigger_tx_t). Do not modify.
    uint32_t size = sizeof(trigger_tx_t);

    int32_t dummy1 = 0;
    int32_t dummy2 = 0;
    // --- end of v1: append new fields below this line only ---
};

/***********************************************************************
 * Compile-time checks
 **********************************************************************/
// Required so the library can copy a caller's struct with memcpy and so that
// offsetof() is well-defined.
#define UHD_TRIGGER_CHECK_LAYOUT(T)                                              \
    static_assert(std::is_standard_layout_v<T>, #T " must be standard layout");  \
    static_assert(std::is_trivially_copyable_v<T>, #T " must be trivially copyable"); \
    static_assert(offsetof(T, size) == 0, #T "::size must be the first member"); \
    /* Any user-declared constructor (C++20) makes T a non-aggregate. This      \
     * forbids one entirely, which rules out a constructor defined out of line  \
     * in the library writing the library's sizeof() into a caller's object. */ \
    static_assert(std::is_aggregate_v<T>, #T " must not declare a constructor"); \
    /* Catches fields inserted or resized inside the v1 region. */               \
    static_assert(offsetof(T, dummy2) + sizeof(int32_t) <= T::min_size,          \
        #T " v1 fields no longer fit in min_size")

UHD_TRIGGER_CHECK_LAYOUT(trigger_op_t);
UHD_TRIGGER_CHECK_LAYOUT(trigger_global_t);
UHD_TRIGGER_CHECK_LAYOUT(trigger_rx_t);
UHD_TRIGGER_CHECK_LAYOUT(trigger_tx_t);

#undef UHD_TRIGGER_CHECK_LAYOUT

// Verifies min_size matches the v1 layout.
// DELETE these once the structs are released: they will (correctly) fail as
// soon as a field is appended.
static_assert(sizeof(trigger_op_t) == trigger_op_t::min_size);
static_assert(sizeof(trigger_global_t) == trigger_global_t::min_size);
static_assert(sizeof(trigger_rx_t) == trigger_rx_t::min_size);
static_assert(sizeof(trigger_tx_t) == trigger_tx_t::min_size);

}} // namespace uhd::trigger
