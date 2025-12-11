//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstddef>
#include <vector>
#include <uhd/types/metadata.hpp>

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
    class pv_tx_async_msg_queue {
    public:

        /**
         * Constructor of tx_async_msg_queue
         *
         * @param num_channels The number of channels to manage
         * @param max_messages_per_channel The maximum number of messages per channel
         */
        pv_tx_async_msg_queue(size_t num_channels, size_t max_messages_per_channel = 1000);

        /**
         * Adds a message to the FIFO.
         *
         * It is thread safe with respect to pop but not with respect to itself
         *
         * @param ch The channel the message is for
         * @param msg The message to add to the fifo
         */
        void push(const size_t ch, const uhd::async_metadata_t msg);

        /**
         * Removes a message from the FIFO.
         *
         * It is thread safe with respect to push but not with respect to itself
         *
         * @param msg A pointer to where to store the message
         * @return 0 indicates success, non 0 indicates an error
         */
        int pop(uhd::async_metadata_t* msg);

    private:

        struct tracked_msg {
            // The message
            uhd::async_metadata_t msg;
            // The number of messages that have begun to be written to this buffer. Used to check if the message overflowed
            size_t message_writes_started = 0;
            // The number of messages that have been finished being written to this buffer. Used to check if a message is ready
            size_t message_writes_completed = 0;
        };
        // The inner vector stores per channel messages, the outer vector is used to select the channel
        std::vector<std::vector<tracked_msg>> messages;

        // Number of messages written to the buffer
        std::vector<size_t> messages_written;

        // The number of messages read for each channel's buffer
        std::vector<size_t> messages_read;

        // The maximum number of messages per channel
        const size_t _max_messages_per_channel;
    };
}
