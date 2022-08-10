//
// Copyright 2017 Ettus Research
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <functional>
#include <memory>

namespace uhd { namespace transport {

/*!
 * Flow control function.
 * \param buff buffer to be sent or receive buffer being released
 * \return true if OK, false if not
 */
typedef std::function<bool(managed_buffer::sptr buff)> send_flow_ctrl_func;

/*!
 * Flow control function.
 * \param buff buffer to be sent or receive buffer being released
 * \param error_code error code for sys called, most commonly used to check for EINTR
 * \return true if OK, false if not
 */
typedef std::function<bool(managed_buffer::sptr buff, int *error_code)> recv_flow_ctrl_func;

/*!
 * Adds flow control to any zero_copy_if transport.
 */
class UHD_API zero_copy_flow_ctrl : public virtual zero_copy_if
{
public:
    typedef std::shared_ptr<zero_copy_flow_ctrl> sptr;

    /*!
     * Make flow controlled transport.
     *
     * \param transport a shared pointer to the transport interface
     * \param send_flow_ctrl optional send flow control function called before buffer is
     * sent \param recv_flow_ctrl optional receive flow control function called after
     * buffer released
     */
    static sptr make(zero_copy_if::sptr transport,
        send_flow_ctrl_func send_flow_ctrl,
        recv_flow_ctrl_func recv_flow_ctrl);
};

}} // namespace uhd::transport
