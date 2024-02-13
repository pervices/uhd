//
// Copyright 2011-2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/types/time_spec.hpp>
#include <cmath>
#include <time.h>

using namespace uhd;

/***********************************************************************
 * Time spec constructors
 **********************************************************************/

time_spec_t::time_spec_t(double secs)
{
    _tick_rate = 1e9;
    _full_secs = (int64_t) secs;
    if(_full_secs >= 0) {
        _ticks = (secs - _full_secs) * _tick_rate;
    }

    carry();
}

time_spec_t::time_spec_t(int64_t full_secs, double frac_secs)
{
    _tick_rate = 1e9;
    _full_secs = (int64_t) full_secs;
    _ticks = frac_secs * _tick_rate;

    carry();
}

time_spec_t::time_spec_t(int64_t full_secs, long tick_count, double tick_rate)
{
    _tick_rate = tick_rate;
    _full_secs = full_secs;
    _ticks = tick_count;

    carry();
}

time_spec_t time_spec_t::from_ticks(long long ticks, double tick_rate)
{
    int64_t full_secs = (int64_t) (ticks/tick_rate);
    long tick_count = (long) ticks - (full_secs * tick_rate);

    return time_spec_t(full_secs, tick_count, tick_rate);
}

/***********************************************************************
 * Time spec accessors
 **********************************************************************/
long time_spec_t::get_tick_count(double tick_rate) const
{
    return long(::round(_ticks * tick_rate / _tick_rate));
}

long long time_spec_t::to_ticks(double tick_rate) const
{
    return (long long)(_full_secs * tick_rate) + (_ticks * (tick_rate/_tick_rate));
}

double time_spec_t::get_real_secs(void) const
{
    return this->get_full_secs() + this->get_frac_secs();
}

void time_spec_t::sleep_for(void) const {

    struct timespec duration = { this->get_full_secs(), (int64_t) (this->get_frac_secs() * 1e9) };

    nanosleep(&duration, NULL);
}

UHD_INLINE void time_spec_t::carry(void) {

    int64_t amount_to_carry = (int64_t) (_ticks/_tick_rate);

    int64_t original_full_secs = _full_secs;
    double original_ticks = _ticks;

    _full_secs += amount_to_carry;
    _ticks -= amount_to_carry * _tick_rate;

    int tmp = 0;
    int64_t bring_over = 0;
    // NOTE: min is used because for very small values of _ticks, _ticks / _tick_rate can become 0
    if(_ticks < 0 && _full_secs > 0) {
        tmp = 1;
        bring_over = std::max(std::ceil( -_ticks / _tick_rate ), 1.0);
        _ticks += bring_over * _tick_rate;
        _full_secs -= bring_over;

    } else if(_ticks > 0 && _full_secs < 0) {
        tmp = 2;
        bring_over = std::max(std::ceil( _ticks / _tick_rate ), 1.0);
        _ticks -= bring_over * _tick_rate;
        _full_secs += bring_over;
    }

    if(::copysign(1, _ticks) != ::copysign(1, _full_secs) && _ticks != 0 && _full_secs != 0) {
        printf("original_full_secs: %li\n", original_full_secs);
        printf("original_ticks: %lf\n", original_ticks);
        printf("2_full_secs: %li\n", _full_secs);
        printf("2_ticks: %lf\n", _ticks);
        printf("original_ticks > 0: %hhu\n", original_ticks > 0);
        printf("original_ticks < 0: %hhu\n", original_ticks < 0);
        printf("_ticks > 0: %hhu\n", _ticks > 0);
        printf("_ticks < 0: %hhu\n", _ticks < 0);
        printf("bring_over: %li\n", bring_over);
        printf("_tick_rate: %lf\n", _tick_rate);
        printf("tmp: %i\n", tmp);
        printf("_ticks / _tick_rate == 0: %hhu\n", _ticks / _tick_rate == 0);
        printf("std::ceil( _ticks / _tick_rate ): %lf\n", std::ceil( _ticks / _tick_rate ));
        printf("error in carry\n");
        std::exit(0);
    }

}

/***********************************************************************
 * Time spec math overloads
 **********************************************************************/
time_spec_t& time_spec_t::operator+=(const time_spec_t& rhs)
{
    _full_secs += rhs._full_secs;

    // If the tick rates don't match, convert to the side with the higher tick rate
    double target_tick_rate;
    // Tick rates match
    if(_tick_rate == rhs._tick_rate) {
        target_tick_rate = _tick_rate;
        // lhs is within 1THz of being a multiple of rhs
    } else if(_tick_rate / rhs._tick_rate < 1e-12 && _tick_rate / rhs._tick_rate > -1e-12) {
        target_tick_rate = _tick_rate;
        // rhs is within 1THz of being a multiple of lhs
    } else if(rhs._tick_rate / _tick_rate < 1e-12 && rhs._tick_rate / _tick_rate > -1e-12) {
        target_tick_rate = rhs._tick_rate;
        // Since the product of the tick rates can perfectly prepresent them, use it if the tick count can be represented in int64_t
    } else if(_tick_rate * rhs._tick_rate < INT64_MAX) {
        target_tick_rate = _tick_rate * rhs._tick_rate;
        // If no other option works, convert to whichever side has the higher tick rate
    } else {
        target_tick_rate = std::max(_tick_rate, rhs._tick_rate);
    }

    // Tick rate conversion factors
    double lhs_conversion_factor = target_tick_rate / _tick_rate;
    double rhs_conversion_factor = target_tick_rate / _tick_rate;

    _ticks = (_ticks * lhs_conversion_factor) + (rhs._ticks * rhs_conversion_factor);
    _tick_rate = target_tick_rate;

    carry();

    return *this;
}

time_spec_t& time_spec_t::operator+=(double& rhs)
{
    int64_t rhs_full_secs = std::trunc(rhs);
    double rhs_total_ticks = (rhs - rhs_full_secs) * _tick_rate;

    _full_secs+=rhs_full_secs;
    _ticks+=rhs_total_ticks;

    carry();

    return *this;
}

time_spec_t& time_spec_t::operator-=(const time_spec_t& rhs)
{
    _full_secs -= rhs._full_secs;

    // If the tick rates don't match, convert to the side with the higher tick rate
    double target_tick_rate;
    // Tick rates match
    if(_tick_rate == rhs._tick_rate) {
        target_tick_rate = _tick_rate;
        // lhs is within 1THz of being a multiple of rhs
    } else if(_tick_rate / rhs._tick_rate < 1e-12 && _tick_rate / rhs._tick_rate > -1e-12) {
        target_tick_rate = _tick_rate;
        // rhs is within 1THz of being a multiple of lhs
    } else if(rhs._tick_rate / _tick_rate < 1e-12 && rhs._tick_rate / _tick_rate > -1e-12) {
        target_tick_rate = rhs._tick_rate;
        // Since the product of the tick rates can perfectly prepresent them, use it if the tick count can be represented in int64_t
    } else if(_tick_rate * rhs._tick_rate < INT64_MAX) {
        target_tick_rate = _tick_rate * rhs._tick_rate;
        // If no other option works, convert to whichever side has the higher tick rate
    } else {
        target_tick_rate = std::max(_tick_rate, rhs._tick_rate);
    }

    // Tick rate conversion factors
    double lhs_conversion_factor = target_tick_rate / _tick_rate;
    double rhs_conversion_factor = target_tick_rate / _tick_rate;

    _ticks = (_ticks * lhs_conversion_factor) - (rhs._ticks * rhs_conversion_factor);
    _tick_rate = target_tick_rate;

    carry();

    return *this;
}

time_spec_t& time_spec_t::operator-=(double& rhs)
{
    int64_t rhs_full_secs = std::trunc(rhs);
    double rhs_total_ticks = (rhs - rhs_full_secs) * _tick_rate;

    _full_secs-=rhs_full_secs;
    _ticks-=rhs_total_ticks;

    carry();

    return *this;
}

bool uhd::operator==(const time_spec_t& lhs, const time_spec_t& rhs)
{
    return lhs.get_full_secs() == rhs.get_full_secs()
           and lhs.get_frac_secs() == rhs.get_frac_secs();
}

bool uhd::operator<(const time_spec_t& lhs, const time_spec_t& rhs)
{
    return ((lhs.get_full_secs() < rhs.get_full_secs())
            or ((lhs.get_full_secs() == rhs.get_full_secs())
                   and (lhs.get_frac_secs() < rhs.get_frac_secs())));
}
