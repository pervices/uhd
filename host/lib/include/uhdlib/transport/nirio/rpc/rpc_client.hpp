//
// Copyright 2013 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once


#include <uhd/utils/noncopyable.hpp>

#include "rpc_common.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/noncopyable.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <memory>
#include <mutex>

namespace uhd { namespace usrprio_rpc {

class rpc_client : private uhd::noncopyable
{
public:
    static const uint32_t CURRENT_VERSION           = 1;
    static const uint32_t OLDEST_COMPATIBLE_VERSION = 1;

    rpc_client(const std::string& server,

        const std::string& port,
        uint32_t process_id,
        uint32_t host_id);
    ~rpc_client();

    const boost::system::error_code& call(func_id_t func_id,

        const func_args_writer_t& in_args,
        func_args_reader_t& out_args,
        boost::posix_time::milliseconds timeout);

    inline const boost::system::error_code& status() const
    {
        return _exec_err;
    }

    /* [Possible boost::system::error_code values]
     * boost::asio::error::connection_aborted: Network connection failure
     * boost::asio::error::eof:                Network connection closed cleanly
     * boost::asio::error::connection_refused: Software handshake failure (version
     * mismatch, etc) boost::asio::error::timed_out:          RPC call timed out
     * boost::asio::error::operation_aborted:  RPC call aborted but connection alive
     */

private:
    void _handle_response_hdr(
        const boost::system::error_code& err, size_t transferred, size_t expected);
    void _handle_response_data(
        const boost::system::error_code& err, size_t transferred, size_t expected);
    void _wait_for_next_response_header();

    inline void _stop_io_context()
    {
        if (_io_context_thread.get()) {
            UHD_LOG_DEBUG("NIRIO", "rpc_client stopping...");
            _io_context.stop();
            _io_context_thread->join();
            _io_context_thread.reset();
            UHD_LOG_DEBUG("NIRIO", "rpc_client stopped.");
        }
    }

    // Services
    boost::asio::io_context _io_context;
    std::unique_ptr<boost::thread> _io_context_thread;
    boost::asio::ip::tcp::socket _socket;
    // Handshake info
    hshake_args_t _hshake_args_client;
    hshake_args_t _hshake_args_server;
    // In-out function args
    func_xport_buf_t _request;
    func_xport_buf_t _response;
    // Synchronization
    std::mutex _mutex;
    boost::condition _exec_gate;
    boost::system::error_code _exec_err;
};

}} // namespace uhd::usrprio_rpc
