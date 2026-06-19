//
// Copyright 2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/types/time_spec_hp.hpp>

using namespace uhd;

time_spec_hp::time_spec_hp(int64_t seconds, int64_t ticks, double tick_rate)
:
    _seconds(seconds),
    _ticks(ticks),
    _tick_rate(tick_rate)
{

}
