//
// Copyright 2025 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/pv_tx_async_msg_queue.hpp>

namespace uhd {
    pv_tx_async_msg_queue::pv_tx_async_msg_queue(size_t num_channels, size_t max_messages_per_channel)
    :
    messages(num_channels, std::vector<tracked_msg>(max_messages_per_channel)),
    messages_read(num_channels, 0)
    {

    }
}
