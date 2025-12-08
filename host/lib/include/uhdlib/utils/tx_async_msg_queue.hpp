//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

namespace uhd {

    /** A ring buffer optimized for storing tx async message
     *
     * Tx uses async messages to report overflows and underflows to the reader.
     * The old system used locking to control access, and would drop the last message when full.
     * This implementation is lockless, but at the risk of duplicate reads if reading from different files.
     * Also it skips the entire buffer when full instead of overwriting the last element.
     * The above compromises are deemed worth it to fix a bug that can cause programs to hang if reading and writing to much
     *
     * @param T The type of the element to store in the fifo
     */
    template <typename T> class async_lossy_fifo_swmr {

        std::vector<std::vector<uhd::async_metadata_t>> messages;


    }
}
