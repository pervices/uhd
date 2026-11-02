//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/exception.hpp>
#include <uhdlib/types/trigger_options_helpers.hpp>
#include <algorithm>
#include <cstring>
#include <format>
#include <string>

namespace uhd { namespace trigger_pv { namespace detail {

namespace {

/***********************************************************************
 * Type names for error messages
 **********************************************************************/
template <typename T>
constexpr const char* type_name();
template <>
constexpr const char* type_name<trigger_op_t>()
{
    return "trigger_op_t";
}
template <>
constexpr const char* type_name<trigger_global_t>()
{
    return "trigger_global_t";
}
template <>
constexpr const char* type_name<trigger_rx_t>()
{
    return "trigger_rx_t";
}
template <>
constexpr const char* type_name<trigger_tx_t>()
{
    return "trigger_tx_t";
}

/***********************************************************************
 * Generic implementations. The public overloads below forward to these.
 **********************************************************************/
//! Throw if a caller's `size` is too small to be any valid version of T.
template <typename T>
void check_min_size(uint32_t caller_size)
{
    if (caller_size < T::min_size) {
        throw uhd::value_error(std::format(
            "{}: size {} is smaller than the minimum {}. "
            "Was the struct zeroed or allocated with malloc instead of constructed?",
            type_name<T>(), caller_size, T::min_size));
    }
}

template <typename T>
const T* caller_element_impl(const T* caller_array, size_t len, size_t channel_index)
{
    if (caller_array == nullptr) {
        throw uhd::value_error(std::format("{}: array is null", type_name<T>()));
    }
    if (channel_index >= len) {
        throw uhd::index_error(std::format("{}: channel_index {} is out of range (array length {})",
            type_name<T>(), channel_index, len));
    }

    // caller_array[0] needs no stride, and `size` is at offset 0 in every
    // version, so this read is always valid.
    const uint32_t stride = caller_array[0].size;
    // Must be checked before stride is used to compute any address.
    check_min_size<T>(stride);

    const auto* base = reinterpret_cast<const unsigned char*>(caller_array);
    const T* element = reinterpret_cast<const T*>(base + channel_index * stride);

    if (element->size != stride) {
        throw uhd::value_error(std::format(
            "{}[{}]: size {} differs from element 0's size {}. "
            "All elements of one array must be the same struct version.",
            type_name<T>(), channel_index, element->size, stride));
    }
    return element;
}

template <typename T>
T copy_from_caller_impl(const T& caller)
{
    const uint32_t caller_size = caller.size;
    check_min_size<T>(caller_size);

    T library{};
    std::memcpy(&library, &caller, std::min<size_t>(caller_size, sizeof(T)));

    // Caller built against a newer header: reject non-zero fields we don't know.
    const auto* caller_bytes = reinterpret_cast<const unsigned char*>(&caller);
    for (size_t i = sizeof(T); i < caller_size; i++) {
        if (caller_bytes[i] != 0) {
            throw uhd::value_error(std::format(
                "{}: uses fields not supported by this version of libuhdpv", type_name<T>()));
        }
    }
    return library;
}

template <typename T>
void copy_to_caller_impl(const T& library, T& caller)
{
    const uint32_t caller_size = caller.size;
    check_min_size<T>(caller_size);

    // Skip `size` (offset 0): it describes the caller's layout and must keep
    // the caller's value even if the library's copy holds something else.
    constexpr size_t skip = sizeof(T::size);
    const size_t num_bytes = std::min<size_t>(caller_size, sizeof(T));

    auto* caller_bytes = reinterpret_cast<unsigned char*>(&caller);
    const auto* library_bytes = reinterpret_cast<const unsigned char*>(&library);
    std::memcpy(caller_bytes + skip, library_bytes + skip, num_bytes - skip);
    // Bytes past sizeof(T) in a newer caller's struct are left alone. They were
    // verified to be zero on the way in, so they still are.
}

template <typename T>
std::vector<T> copy_array_from_caller_impl(const T* caller_array, size_t len)
{
    std::vector<T> library;
    if (len == 0) {
        return library;
    }

    library.reserve(len);
    for (size_t channel_index = 0; channel_index < len; channel_index++) {
        library.push_back(
            copy_from_caller_impl(*caller_element_impl(caller_array, len, channel_index)));
    }
    return library;
}

template <typename T>
void copy_array_to_caller_impl(const std::vector<T>& library, T* caller_array, size_t len)
{
    if (library.size() != len) {
        throw uhd::value_error(std::format("{}: library array has {} elements but caller array has {}",
            type_name<T>(), library.size(), len));
    }
    if (len == 0) {
        return;
    }

    // Validate every caller element before writing any of them, so an exception
    // leaves the caller's array untouched.
    for (size_t channel_index = 0; channel_index < len; channel_index++) {
        caller_element_impl<T>(caller_array, len, channel_index);
    }

    for (size_t channel_index = 0; channel_index < len; channel_index++) {
        // Safe to cast away const: the array itself is non-const.
        T* caller =
            const_cast<T*>(caller_element_impl<T>(caller_array, len, channel_index));
        copy_to_caller_impl(library[channel_index], *caller);
    }
}

template <typename T>
std::vector<size_t> find_unspecified_channels_impl(
    const std::vector<T>& library, size_t num_channels)
{
    std::vector<bool> is_specified(num_channels, false);

    for (size_t channel_index = 0; channel_index < library.size(); channel_index++) {
        const size_t channel = library[channel_index].channel;
        if (channel == T::channel_unset) {
            throw uhd::value_error(std::format(
                "{}[{}]: channel was not set", type_name<T>(), channel_index));
        }
        if (channel >= num_channels) {
            throw uhd::index_error(std::format(
                "{}[{}]: channel {} does not exist (device has {} channels)",
                type_name<T>(), channel_index, channel, num_channels));
        }
        if (is_specified[channel]) {
            throw uhd::value_error(std::format("{}[{}]: channel {} is specified more than once",
                type_name<T>(), channel_index, channel));
        }
        is_specified[channel] = true;
    }

    std::vector<size_t> unspecified;
    for (size_t channel = 0; channel < num_channels; channel++) {
        if (!is_specified[channel]) {
            unspecified.push_back(channel);
        }
    }
    return unspecified;
}

} // namespace

/***********************************************************************
 * Public overloads
 **********************************************************************/
const trigger_rx_t* caller_element(
    const trigger_rx_t* caller_array, size_t len, size_t channel_index)
{
    return caller_element_impl(caller_array, len, channel_index);
}
trigger_rx_t* caller_element(trigger_rx_t* caller_array, size_t len, size_t channel_index)
{
    // Safe to cast away const: the array itself is non-const.
    return const_cast<trigger_rx_t*>(
        caller_element_impl<trigger_rx_t>(caller_array, len, channel_index));
}
const trigger_tx_t* caller_element(
    const trigger_tx_t* caller_array, size_t len, size_t channel_index)
{
    return caller_element_impl(caller_array, len, channel_index);
}
trigger_tx_t* caller_element(trigger_tx_t* caller_array, size_t len, size_t channel_index)
{
    return const_cast<trigger_tx_t*>(
        caller_element_impl<trigger_tx_t>(caller_array, len, channel_index));
}

trigger_op_t deep_copy_from_caller(const trigger_op_t& caller)
{
    return copy_from_caller_impl(caller);
}
trigger_global_t deep_copy_from_caller(const trigger_global_t& caller)
{
    return copy_from_caller_impl(caller);
}
trigger_rx_t deep_copy_from_caller(const trigger_rx_t& caller)
{
    return copy_from_caller_impl(caller);
}
trigger_tx_t deep_copy_from_caller(const trigger_tx_t& caller)
{
    return copy_from_caller_impl(caller);
}

void deep_copy_to_caller(const trigger_global_t& library, trigger_global_t& caller)
{
    copy_to_caller_impl(library, caller);
}
void deep_copy_to_caller(const trigger_rx_t& library, trigger_rx_t& caller)
{
    copy_to_caller_impl(library, caller);
}
void deep_copy_to_caller(const trigger_tx_t& library, trigger_tx_t& caller)
{
    copy_to_caller_impl(library, caller);
}

std::vector<trigger_rx_t> deep_copy_array_from_caller(const trigger_rx_t* caller_array, size_t len)
{
    return copy_array_from_caller_impl(caller_array, len);
}
std::vector<trigger_tx_t> deep_copy_array_from_caller(const trigger_tx_t* caller_array, size_t len)
{
    return copy_array_from_caller_impl(caller_array, len);
}

void deep_copy_array_to_caller(
    const std::vector<trigger_rx_t>& library, trigger_rx_t* caller_array, size_t len)
{
    copy_array_to_caller_impl(library, caller_array, len);
}
void deep_copy_array_to_caller(
    const std::vector<trigger_tx_t>& library, trigger_tx_t* caller_array, size_t len)
{
    copy_array_to_caller_impl(library, caller_array, len);
}

std::vector<size_t> find_unspecified_channels(
    const std::vector<trigger_rx_t>& library, size_t num_channels)
{
    return find_unspecified_channels_impl(library, num_channels);
}
std::vector<size_t> find_unspecified_channels(
    const std::vector<trigger_tx_t>& library, size_t num_channels)
{
    return find_unspecified_channels_impl(library, num_channels);
}

/***********************************************************************
 * Enum <-> device string conversion
 **********************************************************************/
namespace {

//! Copy of `str` without trailing whitespace, e.g. a newline from the device.
std::string without_trailing_whitespace(const std::string& str)
{
    const size_t last = str.find_last_not_of(" \t\r\n");
    if (last == std::string::npos) {
        return "";
    }
    return str.substr(0, last + 1);
}

} // namespace

std::string to_device_string(pin_direction_t dir)
{
    switch (dir) {
        case pin_direction_t::get:
            throw uhd::value_error("pin_direction_t::get cannot be sent to the device");
        case pin_direction_t::in:
            return "in";
        case pin_direction_t::out:
            return "out";
    }
    // Reached only if the caller cast an out-of-range number to pin_direction_t.
    throw uhd::value_error(std::format("invalid pin_direction_t value {}", static_cast<uint32_t>(dir)));
}

pin_direction_t direction_from_device_string(const std::string& str)
{
    const std::string trimmed = without_trailing_whitespace(str);
    if (trimmed == "in") {
        return pin_direction_t::in;
    }
    if (trimmed == "out") {
        return pin_direction_t::out;
    }
    throw uhd::runtime_error(std::format("device reported unknown direction \"{}\"", str));
}

std::string to_device_string(polarity_t pol)
{
    switch (pol) {
        case polarity_t::get:
            throw uhd::value_error("polarity_t::get cannot be sent to the device");
        case polarity_t::positive:
            return "positive";
        case polarity_t::negative:
            return "negative";
    }
    throw uhd::value_error(std::format("invalid polarity_t value {}", static_cast<uint32_t>(pol)));
}

polarity_t polarity_from_device_string(const std::string& str)
{
    const std::string trimmed = without_trailing_whitespace(str);
    if (trimmed == "positive") {
        return polarity_t::positive;
    }
    if (trimmed == "negative") {
        return polarity_t::negative;
    }
    throw uhd::runtime_error(std::format("device reported unknown polarity \"{}\"", str));
}

std::string to_device_string(trigger_mode_t mode)
{
    switch (mode) {
        case trigger_mode_t::get:
            throw uhd::value_error("trigger_mode_t::get cannot be sent to the device");
        case trigger_mode_t::edge:
            return "edge";
        case trigger_mode_t::level:
            return "level";
    }
    throw uhd::value_error(
        std::format("invalid trigger_mode_t value {}", static_cast<uint32_t>(mode)));
}

trigger_mode_t trigger_mode_from_device_string(const std::string& str)
{
    const std::string trimmed = without_trailing_whitespace(str);
    if (trimmed == "edge") {
        return trigger_mode_t::edge;
    }
    if (trimmed == "level") {
        return trigger_mode_t::level;
    }
    throw uhd::runtime_error(std::format("device reported unknown trigger mode \"{}\"", str));
}

std::string to_device_string(gating_t gating)
{
    switch (gating) {
        case gating_t::get:
            throw uhd::value_error("gating_t::get cannot be sent to the device");
        case gating_t::dsp:
            return "dsp";
        case gating_t::output:
            return "output";
    }
    throw uhd::value_error(std::format("invalid gating_t value {}", static_cast<uint32_t>(gating)));
}

gating_t gating_from_device_string(const std::string& str)
{
    const std::string trimmed = without_trailing_whitespace(str);
    if (trimmed == "dsp") {
        return gating_t::dsp;
    }
    if (trimmed == "output") {
        return gating_t::output;
    }
    throw uhd::runtime_error(std::format("device reported unknown gating \"{}\"", str));
}

}}} // namespace uhd::trigger_pv::detail
