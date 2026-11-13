//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/types/trigger_options.hpp>
#include <concepts>
#include <cstddef>
#include <string>
#include <type_traits>
#include <vector>

/*
 * Library-internal helpers for converting the versioned trigger structs between
 * the caller's layout and the library's layout. Not part of the public API:
 * nothing here is exported, so it can change freely.
 *
 * Terminology:
 *  - "caller format": the struct as the caller compiled it. Its `size` field is
 *    the caller's sizeof(), which may be smaller (older header) or larger (newer
 *    header) than the library's sizeof(). Arrays in caller format must be
 *    walked using the caller's sizeof() as the stride.
 *  - "library format": the struct as this library was compiled. Arrays in
 *    library format are ordinary arrays/vectors.
 *  - channel: a physical channel, 0 to get_xx_num_channels() - 1.
 *  - channel_index: the position of an element within an array. There is no
 *    relationship between channel_index and channel.
 *
 * Functions throw uhd::value_error or uhd::index_error for invalid input, and
 * uhd::runtime_error if the device reports a value the library does not recognise.s
 */

namespace uhd { namespace trigger_pv { namespace detail {

/***********************************************************************
 * Caller-format array element addressing
 **********************************************************************/
/*!
 * Return a pointer to element `channel_index` of an array in caller format.
 *
 * The address is computed with the caller's stride (element 0's `size`), never
 * with the library's sizeof(). The element is validated: its `size` must be at
 * least min_size and must match element 0's.
 *
 * Only the first `->size` bytes of the returned element are valid. Read it with
 * deep_copy_from_caller() and write it with deep_copy_to_caller(); do not read fields
 * through the pointer directly, since an older caller's element may not have
 * them.
 */
const trigger_rx_t* caller_element(
    const trigger_rx_t* caller_array, size_t len, size_t channel_index);
trigger_rx_t* caller_element(trigger_rx_t* caller_array, size_t len, size_t channel_index);
const trigger_tx_t* caller_element(
    const trigger_tx_t* caller_array, size_t len, size_t channel_index);
trigger_tx_t* caller_element(trigger_tx_t* caller_array, size_t len, size_t channel_index);

/***********************************************************************
 * Single struct conversion
 **********************************************************************/
/*!
 * Deep copy one struct from caller format to library format.
 *
 * Fields the caller's (older) version lacks keep their default member
 * initializers. Fields from a newer version than the library knows must be
 * zero, otherwise the caller is asking for something this library can't do.
 *
 * The returned struct's `size` still holds the caller's size, so it records
 * which fields the caller actually provided.
 */
trigger_op_t deep_copy_from_caller(const trigger_op_t& caller);
trigger_global_t deep_copy_from_caller(const trigger_global_t& caller);
trigger_rx_t deep_copy_from_caller(const trigger_rx_t& caller);
trigger_tx_t deep_copy_from_caller(const trigger_tx_t& caller);

/*!
 * Deep copy one struct from library format back into the caller's struct.
 *
 * Writes only the bytes that exist in the caller's version (min of the
 * caller's `size` and the library's sizeof()). The caller's `size` field is
 * left untouched.
 */
void deep_copy_to_caller(const trigger_global_t& library, trigger_global_t& caller);
void deep_copy_to_caller(const trigger_rx_t& library, trigger_rx_t& caller);
void deep_copy_to_caller(const trigger_tx_t& library, trigger_tx_t& caller);

/***********************************************************************
 * Whole array conversion
 **********************************************************************/
/*!
 * Deep copy an array from caller format to library format.
 * `caller_array` may be nullptr only if `len` is 0.
 */
std::vector<trigger_rx_t> deep_copy_array_from_caller(const trigger_rx_t* caller_array, size_t len);
std::vector<trigger_tx_t> deep_copy_array_from_caller(const trigger_tx_t* caller_array, size_t len);

/*!
 * Deep copy an array in library format back into the caller's array.
 *
 * `library` must have exactly `len` elements, in the same order as the caller's
 * array. Every caller element is validated before anything is written, so an
 * exception leaves the caller's array unmodified.
 */
void deep_copy_array_to_caller(
    const std::vector<trigger_rx_t>& library, trigger_rx_t* caller_array, size_t len);
void deep_copy_array_to_caller(
    const std::vector<trigger_tx_t>& library, trigger_tx_t* caller_array, size_t len);

/***********************************************************************
 * Channel helpers
 **********************************************************************/
/*!
 * Return the physical channels in [0, num_channels) that no element of
 * `library` refers to, in ascending order.
 *
 * Throws if an element's channel is unset, does not exist, or is named by more
 * than one element.
 */
std::vector<size_t> find_unspecified_channels(
    const std::vector<trigger_rx_t>& library, size_t num_channels);
std::vector<size_t> find_unspecified_channels(
    const std::vector<trigger_tx_t>& library, size_t num_channels);

/***********************************************************************
 * Enum <-> device string conversion
 **********************************************************************/
//! An enum type that has an enumerator named `get`.
template <typename E>
concept enum_with_get = std::is_enum_v<E> && requires {
    { E::get } -> std::same_as<E>; // E::get must exist and be one of E's enumerators
};

/*!
 * Value the SDR expects in the property tree for an enum setting.
 * Throws uhd::value_error for E::get (get is never sent to the device) and for
 * numbers that are not a named enumerator.
 */
std::string to_device_string(pin_direction_t dir);
std::string to_device_string(polarity_t pol);
std::string to_device_string(trigger_mode_t mode);
std::string to_device_string(gating_t gating);

/*!
 * Inverse of to_device_string(), for reading the actual value back from the
 * device. Trailing whitespace (e.g. a newline) in the device's reply is ignored.
 * Throws uhd::runtime_error if the device reports a string with no match.
 */
pin_direction_t direction_from_device_string(const std::string& str);
polarity_t polarity_from_device_string(const std::string& str);
trigger_mode_t trigger_mode_from_device_string(const std::string& str);
gating_t gating_from_device_string(const std::string& str);

}}} // namespace uhd::trigger_pv::detail
