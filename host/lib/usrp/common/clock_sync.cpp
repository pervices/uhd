//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/common/clock_sync.hpp>

// For getting the time on the host
#include <uhdlib/utils/system_time.hpp>

// For sending formatted error and warning message
#include <uhd/utils/log.hpp>

using namespace uhd;
using namespace uhd::usrp;

static constexpr size_t padded_clock_sync_shared_info_size = (size_t) ceil(sizeof(clock_sync_shared_info) / (double)CACHE_LINE_SIZE) * CACHE_LINE_SIZE;

std::shared_ptr<clock_sync_shared_info> clock_sync_shared_info::make() {
    // Create using placement new
    clock_sync_shared_info* raw_pointer = (clock_sync_shared_info*) aligned_alloc(CACHE_LINE_SIZE, padded_clock_sync_shared_info_size);
    new (raw_pointer) clock_sync_shared_info();

    std::shared_ptr<clock_sync_shared_info> ptr(raw_pointer, deleter());

    return ptr;
}

// Wait for convergence
void clock_sync_shared_info::wait_for_sync() {
    for(
        time_spec_t time_then = uhd::get_system_time(),
            time_now = time_then
            ;
        (!is_synced())
            ;
        time_now = uhd::get_system_time()
    ) {
        if ( (time_now - time_then).get_full_secs() > 20 ) {
            UHD_LOG_ERROR("CLOCK_SYNC", "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling the device?");
            throw std::runtime_error( "Clock domain synchronization taking unusually long. Are there more than 1 applications controlling the device?" );
        }
        ::usleep( 10000 );
    }
}
