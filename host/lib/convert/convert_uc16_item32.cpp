//
// Copyright 2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "convert_common.hpp"
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/log.hpp>
#include <boost/math/special_functions/round.hpp>
#include <vector>
#include <iostream>

using namespace uhd::convert;

typedef uint16_t (*to16_type)(uint16_t);

struct convert_uc16_item32_1_to_uc16_item32_be_1 : public converter
{
    convert_uc16_item32_1_to_uc16_item32_be_1(void):_scalar(1.0)
    {
        //NOP
    }

    void set_scalar(const double scalar)
    {
        _scalar = scalar;
    }

    void operator()(const input_type &inputs, const output_type &outputs, const size_t nsamps)
    {
        // NOTE: If you need to copy, uncomment the following 

        // const item32_t *input  = reinterpret_cast<const item32_t *>(inputs[0]);
        // item32_t *output = reinterpret_cast<item32_t *>(outputs[0]);
        // std::memcpy(output, input, nsamps*sizeof(item32_t));
    }

    // This converter will bypass conversion
    bool bypass_conversion_and_use_scatter_gather(void) {
        return 1;
    }

    double _scalar;
};

#define __make_registrations(itype, otype, fcn) \
static converter::sptr make_convert_ ## itype ## _1_ ## otype ## _1(void) \
{ \
    return converter::sptr(new fcn()); \
} \
UHD_STATIC_BLOCK(register_convert_ ## itype ## _1_ ## otype ## _1) \
{ \
    uhd::convert::id_type id; \
    id.num_inputs = 1; id.num_outputs = 1;  \
    id.input_format = #itype; id.output_format = #otype; \
    uhd::convert::register_converter(id, &make_convert_ ## itype ## _1_ ## otype ## _1, PRIORITY_GENERAL); \
}

__make_registrations(uc16_item32, uc16_item32_be, convert_uc16_item32_1_to_uc16_item32_be_1)
