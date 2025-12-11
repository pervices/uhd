//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>

// Fences
#include <immintrin.h>

namespace uhd {
    pv_tx_async_msg_queue::pv_tx_async_msg_queue(size_t num_channels, size_t max_messages_per_channel)
    :
    messages(num_channels, std::vector<tracked_msg>(max_messages_per_channel)),
    messages_written(num_channels, 0),
    messages_read(num_channels, 0),
    _max_messages_per_channel(max_messages_per_channel)
    {

    }

    pv_tx_async_msg_queue::push(const size_t ch, const uhd::async_metadata_t msg) {

        // 0 for the first message, 1 for the second, 2 for the third...
        const size_t message_number = messages_written[ch];

        // Location in the buffer to store the message
        // Performance shouldn't matter where this is called. If it ends up mattering replace % with power of 2 bit mask
        const size_t message_location = message_number % _max_messages_per_channel;

        // Increments count indicating that writing this message has begun
        messages[ch][message_location].message_writes_started = message_number + 1;

        // Ensures that the write start counter is incremented before the write begins
        _mm_sfence();

        // Copys the message to the buffer
        messages[ch][message_location].msg = msg;

        // Ensures that the message is written before the write complete counter is incremented
        _mm_sfence();

        // Increment the count indicating that this message has been written
        messages[ch][message_location].message_writes_completed = message_number + 1;

        // Record that a message was written for use by the writer (this function/thread)
        messages_written[ch]++;
    }
}
