//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>

// Fences
#include <immintrin.h>

namespace uhd {
    pv_tx_async_msg_queue::pv_tx_async_msg_queue(size_t max_messages)
    :
    messages(max_messages),
    _max_messages(max_messages)
    {

    }

    void pv_tx_async_msg_queue::push(const uhd::async_metadata_t* msg) {

        // 0 for the first message, 1 for the second, 2 for the third...
        const size_t message_number = messages_written;

        // Location in the buffer to store the message
        // Performance shouldn't matter where this is called. If it ends up mattering replace % with power of 2 bit mask
        const size_t message_location = message_number % _max_messages;

        // Increments count indicating that writing this message has begun
        messages[message_location].message_writes_started = message_number + 1;

        // Ensures that the write start counter is incremented before the write begins
        _mm_sfence();

        // Copys the message to the buffer
        messages[message_location].msg = *msg;

        // Ensures that the message is written before the write complete counter is incremented
        _mm_sfence();

        // Increment the count indicating that this message has been written
        messages[message_location].message_writes_completed = message_number + 1;

        // Record that a message was written for use by the writer (this function/thread)
        messages_written++;
    }

    int pv_tx_async_msg_queue::pop(async_metadata_t* msg, const double timeout) {

        // Buffer to store the next message in each channel
        async_metadata_t next_msg;

        // Maximum number of attempts to copy data from the buffer that can be interrupted by the message being modified.
        // This should never be encountered, since having it interrupted more than once would mean that in the time it takes
        // to copy a message in this thread, the pushing thread wrote the entire ring buffer
        constexpr size_t max_interrupted_copies = 3;

        // Location in the buffer to read the message
        // Performance shouldn't matter where this is called. If it ends up mattering replace % with power of 2 bit mask
        size_t message_location = messages_read % _max_messages;

        size_t writes_started;
        size_t writes_completed;
        size_t copy_attempts = 0;
        do {
            // TODO: check if a packet is ready (currently it only checks if the packet was modified while copying)


            writes_started = messages[message_location].message_writes_started;

            // Ensures that write started counter is read before the message is copied
            _mm_lfence();

            // Copy the message from the shared buffer to memory only used by this thread
            next_msg = messages[message_location].msg;

            // Ensures that write completed counter is read before the message is copied
            _mm_lfence();

            writes_completed = messages[message_location].message_writes_started;

            copy_attempts++;

            // Retry if the message was edited while copying
        } while(writes_started != writes_completed && copy_attempts <= max_interrupted_copies);

        if(copy_attempts > max_interrupted_copies) {
            // TODO: warning message

            return 1;
        }

        // Copy oldest message to caller specified location
        *msg = next_msg;
        // Record that we read this message
        messages_read++;

        return 0;
    }
}
