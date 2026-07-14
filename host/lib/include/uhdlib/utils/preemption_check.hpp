//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

namespace uhd {

    // Copy of the enum from kernel/sched/core.c that defines preemption types
    typedef enum {
        PREEMPT_DYNAMIC_UNDEFINED = -1,
        PREEMPT_DYNAMIC_NONE      = 0,
        PREEMPT_DYNAMIC_VOLUNTARY = 1,
        PREEMPT_DYNAMIC_FULL      = 2,
        PREEMPT_DYNAMIC_LAZY      = 3
    } preempt_mode_t;

    /**
     * Check if preemption is enabled and warn the user if it isn't
     * @return Returns the preemption mode on success. Returns PREEMPT_DYNAMIC_UNDEFINED on failure
     */
    preempt_mode_t check_preemption();

}; /* namespace uhd */
