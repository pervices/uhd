//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/common/clock_sync.hpp>

using namespace uhd;
using namespace uhd::usrp;

std::shared_ptr<clock_sync_shared_info> clock_sync_shared_info::make() {
    // Create using placement new
    clock_sync_shared_info* raw_pointer = (clock_sync_shared_info*) aligned_alloc(CACHE_LINE_SIZE, padded_clock_sync_shared_info_size);
    new (raw_pointer) clock_sync_shared_info();

    // TODO: add destructor
    std::shared_ptr<clock_sync_shared_info> ptr(raw_pointer, deleter());

    return ptr;
}
