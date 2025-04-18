//
// Copyright 2011-2012,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once


#include <uhd/config.hpp>
#include <uhd/types/ref_vector.hpp>

#include <boost/operators.hpp>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace uhd { namespace convert {

//! A conversion class that implements a conversion from inputs -> outputs.
class converter
{
public:
    typedef std::shared_ptr<converter> sptr;
    typedef uhd::ref_vector<void*> output_type;
    typedef uhd::ref_vector<const void*> input_type;

    virtual ~converter(void) = 0;

    //! Set the scale factor (used in floating point conversions)
    virtual void set_scalar(const double) = 0;

    //! The public conversion method to convert inputs -> outputs
    UHD_INLINE void conv(const input_type& in, const output_type& out, const size_t num)
    {
        if (num != 0)
            // Call the () operator. The conversation from sc16 to fc32 is in sse2_sc16_fc32.cpp
            (*this)(in, out, num);
    }

        //   By default, converters do not bypass conversion and pack all the 
        // data into a single buffer. But for increased performance, some converters
        // might try to bypass conversion (as the data has already been packed into the
        // input buffer) and use scatter gather to send VRT header and data.
        //
        //   Any derivative of this class that wants this behavior must overwrite this
        // function and return 1 instead of 0.
        virtual bool bypass_conversion_and_use_scatter_gather(void) {
            return 0;
        }

private:
    //! Callable method: input vectors, output vectors, num samples
    //
    // This is the guts of the converter. When deriving new converter types,
    // this is where the actual conversion routines go.
    //
    // \param in Pointers to the input buffers
    // \param out Pointers to the output buffers
    // \param num Number of items in the input buffers to convert
    virtual void operator()(
        const input_type& in, const output_type& out, const size_t num) = 0;
};

//! Conversion factory function typedef
typedef std::function<converter::sptr(void)> function_type;

//! Priority of conversion routines
typedef int priority_type;

//! Identify a conversion routine in the registry
struct UHD_API id_type : boost::equality_comparable<id_type>
{
    std::string input_format;
    size_t num_inputs;
    std::string output_format;
    size_t num_outputs;
    std::string to_pp_string(void) const;
    std::string to_string(void) const;
};

//! Implement equality_comparable interface
UHD_API bool operator==(const id_type&, const id_type&);

/*!
 * Register a converter function.
 *
 * Converters with higher priority are given preference.
 *
 * \param id identify the conversion
 * \param fcn makes a new converter
 * \param prio the function priority
 */
UHD_API void register_converter(
    const id_type& id, const function_type& fcn, const priority_type prio);


/*!
 * Get a converter factory function.
 * \param id identify the conversion
 * \param prio the desired prio or -1 for best
 * \return the converter factory function
 */
UHD_API function_type get_converter(const id_type& id, const priority_type prio = -1);


/*!
 * Register the size of a particular item.
 * \param format the item format
 * \param size the size in bytes
 */
UHD_API void register_bytes_per_item(const std::string& format, const size_t size);


//! Convert an item format to a size in bytes
UHD_API size_t get_bytes_per_item(const std::string& format);

}} // namespace uhd::convert
