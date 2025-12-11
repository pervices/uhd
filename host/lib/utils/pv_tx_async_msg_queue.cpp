//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>

pv_tx_async_msg_queue::pv_tx_async_msg_queue(size_t num_channels, size_t max_messages_per_channel = 1000) {

    // The message vector of size num_channels by max_messages_per_channel
    messages = std::vector<std::vector<checksum_msg>>(num_channels, std::vector<checksum_msg>(max_messages_per_channel));
}
