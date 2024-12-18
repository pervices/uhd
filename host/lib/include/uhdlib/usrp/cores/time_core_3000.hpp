//
// Copyright 2013-2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <uhd/types/time_spec.hpp>


#include <uhd/types/wb_iface.hpp>
#include <uhd/utils/noncopyable.hpp>
#include <memory>

class time_core_3000 : uhd::noncopyable
{
public:
    typedef std::shared_ptr<time_core_3000> sptr;

    struct readback_bases_type
    {
        size_t rb_now;
        size_t rb_pps;
    };

    virtual ~time_core_3000(void) = 0;

    //! makes a new time core from iface and slave base
    static sptr make(uhd::wb_iface::sptr iface,
        const size_t base,
        const readback_bases_type& readback_bases);

    virtual void self_test(void) = 0;

    virtual void set_tick_rate(const double rate) = 0;

    virtual uhd::time_spec_t get_time_now(void) = 0;

    virtual uhd::time_spec_t get_time_last_pps(void) = 0;

    virtual void set_time_now(const uhd::time_spec_t& time) = 0;

    virtual void set_time_sync(const uhd::time_spec_t& time) = 0;

    virtual void set_time_next_pps(const uhd::time_spec_t& time) = 0;
};
