//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

namespace uhd {

    /**
     * A ring buffer optimized for storing tx async message.
     *
     * Tx uses async messages to report overflows and underflows to the reader.
     * The old system used locking to control access, and would drop the last message when full.
     * This implementation is lockless, but at the risk of duplicate reads if reading from different files.
     * Also it skips the entire buffer when full instead of overwriting the last element.
     * The above compromises are deemed worth it to fix a bug that can cause programs to hang if reading and writing to much
     */
    class async_lossy_fifo_swmr {
    public:

        /**
         * Constructor of async_lossy_fifo_swmr
         */
        async_lossy_fifo_swmr(size_t num_channels, size_t max_messages_per_channel);

        /**
         * Adds a message to the FIFO.
         *
         * It is thread safe with respect to pop but not with respect to itself
         *
         * @param ch The channel the message is for
         * @param msg The message to add to the fifo
         */
        void push(size_t ch, uhd::async_metadata_t msg);

        /**
         * Removeds a message from the FIFO.
         *
         * @param msg A pointer to where to store the message
         * @return 0 indicates success, non 0 indicates an error
         */
        int pop(uhd::async_metadata_t* msg);

    private:
        // The inner vector stores per channel messages, the outer vector is used to select the channel
        std::vector<std::vector<uhd::async_metadata_t>> messages;
    }
}
