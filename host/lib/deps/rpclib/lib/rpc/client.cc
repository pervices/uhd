#include "rpc/client.h"
#include "rpc/config.h"
#include "rpc/rpc_error.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <boost/asio.hpp>
#include <boost/format.hpp>

#include "rpc/detail/async_writer.h"
#include "rpc/detail/dev_utils.h"
#include "rpc/detail/response.h"

using namespace boost::asio;
using boost::asio::ip::tcp;
using namespace rpc::detail;

namespace rpc {

static constexpr uint32_t default_buffer_size = rpc::constants::DEFAULT_BUFFER_SIZE;

struct client::impl {
    impl(client *parent, std::string const &addr, uint16_t port)
        : parent_(parent),
          io_(),
          strand_(io_.get_executor()),
          call_idx_(0),
          addr_(addr),
          port_(port),
          is_connected_(false),
          state_(client::connection_state::initial),
          writer_(std::make_shared<detail::async_writer>(
              &io_, boost::asio::ip::tcp::socket(io_))),
          timeout_(5000) {
        pac_.reserve_buffer(default_buffer_size);
    }

    void do_connect(const tcp::resolver::results_type& endpoints) {
        LOG_INFO("Initiating connection.");
        boost::asio::async_connect(
            writer_->socket_, endpoints,
            [this](boost::system::error_code ec, tcp::endpoint) {
                std::unique_lock<std::mutex> lock(mut_connection_finished_);
                if (!ec) {
                    LOG_INFO("Client connected to {}:{}", addr_, port_);
                    is_connected_ = true;
                    state_ = client::connection_state::connected;
                    conn_finished_.notify_all();
                    do_read();
                } else {
                    LOG_ERROR("Error during connection: {}", ec);
                    is_connected_ = false;
                    state_ = client::connection_state::disconnected;
                    conn_finished_.notify_all();
                }
            });
    }

    void do_read() {
        LOG_TRACE("do_read");
        constexpr std::size_t max_read_bytes = default_buffer_size;
        writer_->socket_.async_read_some(
            boost::asio::buffer(pac_.buffer(), max_read_bytes),
            // I don't think max_read_bytes needs to be captured explicitly
            // (since it's constexpr), but MSVC insists.
            [this, max_read_bytes](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    LOG_TRACE("Read chunk of size {}", length);
                    pac_.buffer_consumed(length);

                    RPCLIB_MSGPACK::unpacked result;
                    while (pac_.next(&result)) {
                        auto r = response(std::move(result));
                        auto id = r.get_id();
                        auto &current_call = ongoing_calls_[id];
                        try {
                            if (r.get_error()) {
                                throw rpc_error("rpc::rpc_error during call",
                                                std::get<0>(current_call),
                                                RPCLIB_MSGPACK::clone(
                                                    r.get_error()->get()));
                            }
                            std::get<1>(current_call)
                                .set_value(std::move(*r.get_result()));
                        } catch (...) {
                            std::get<1>(current_call)
                                .set_exception(std::current_exception());
                        }
                        boost::asio::post(strand_,
                            [this, id]() { ongoing_calls_.erase(id); });
                    }

                    // resizing strategy: if the remaining buffer size is
                    // less than the maximum bytes requested from asio,
                    // then request max_read_bytes. This prompts the unpacker
                    // to resize its buffer doubling its size
                    // (https://github.com/msgpack/msgpack-c/issues/567#issuecomment-280810018)
                    if (pac_.buffer_capacity() < max_read_bytes) {
                        LOG_TRACE("Reserving extra buffer: {}", max_read_bytes);
                        pac_.reserve_buffer(max_read_bytes);
                    }
                    do_read();
                } else if (ec == boost::asio::error::eof) {
                    LOG_WARN("The server closed the connection.");
                    state_ = client::connection_state::disconnected;
                } else if (ec == boost::asio::error::connection_reset) {
                    // Yes, this should be connection_state::reset,
                    // but on windows, disconnection results in reset. May be
                    // asio bug, may be a windows socket pecularity. Should be
                    // investigated later.
                    state_ = client::connection_state::disconnected;
                    LOG_WARN("The connection was reset.");
                } else {
                    LOG_ERROR("Unhandled error code: {} | '{}'", ec,
                              ec.message());
                }
            });
    }

    client::connection_state get_connection_state() const { return state_; }

    //! \brief Waits for the write queue and writes any buffers to the network
    //! connection. Should be executed throught strand_.
    void write(RPCLIB_MSGPACK::sbuffer item) {
        writer_->write(std::move(item));
    }

    using call_t =
        std::pair<std::string, std::promise<RPCLIB_MSGPACK::object_handle>>;

    client *parent_;
    boost::asio::io_context io_;
    boost::asio::strand<boost::asio::io_context::executor_type> strand_;
    std::atomic<int> call_idx_; /// The index of the last call made
    std::unordered_map<uint32_t, call_t> ongoing_calls_;
    std::string addr_;
    uint16_t port_;
    RPCLIB_MSGPACK::unpacker pac_;
    std::atomic_bool is_connected_;
    std::condition_variable conn_finished_;
    std::mutex mut_connection_finished_;
    std::thread io_thread_;
    std::atomic<client::connection_state> state_;
    std::shared_ptr<detail::async_writer> writer_;
    uint64_t timeout_;
    RPCLIB_CREATE_LOG_CHANNEL(client)
};

client::client(std::string const &addr, uint16_t port)
    : pimpl(new client::impl(this, addr, port)) {
    tcp::resolver resolver(pimpl->io_);
    auto endpoints =
        resolver.resolve(pimpl->addr_, std::to_string(pimpl->port_));
    pimpl->do_connect(endpoints);
    std::thread io_thread([this]() {
        RPCLIB_CREATE_LOG_CHANNEL(client)
        name_thread("client");
        pimpl->io_.run();
    });
    pimpl->io_thread_ = std::move(io_thread);
}

void client::wait_conn() {
    std::unique_lock<std::mutex> lock(pimpl->mut_connection_finished_);
    if (!pimpl->is_connected_) {
        pimpl->conn_finished_.wait(lock);
    }
}

int client::get_next_call_idx() {
    ++(pimpl->call_idx_);
    return pimpl->call_idx_;
}

void client::post(std::shared_ptr<RPCLIB_MSGPACK::sbuffer> buffer, int idx,
                  std::string const &func_name,
                  std::shared_ptr<rsp_promise> p) {
    boost::asio::post(pimpl->strand_, [buffer, idx, func_name, p, this]() {
        pimpl->ongoing_calls_.insert(
            std::make_pair(idx, std::make_pair(func_name, std::move(*p))));
        pimpl->write(std::move(*buffer));
    });
}

void client::post(RPCLIB_MSGPACK::sbuffer *buffer) {
    boost::asio::post(pimpl->strand_, [buffer, this]() {
        pimpl->write(std::move(*buffer));
        delete buffer;
    });
}

client::connection_state client::get_connection_state() const {
    return pimpl->get_connection_state();
}

uint64_t client::get_timeout() const {
    return pimpl->timeout_;
}

void client::set_timeout(uint64_t value) {
    pimpl->timeout_ = value;
}

void client::wait_all_responses() {
    for (auto &c : pimpl->ongoing_calls_) {
        c.second.second.get_future().wait();
    }
}

RPCLIB_NORETURN void client::throw_timeout(std::string const& func_name) {
    throw rpc::timeout(
        str(boost::format("Timeout of %dms while calling RPC function '%s'") %
                           get_timeout() % func_name));
}

client::~client() {
    pimpl->io_.stop();
    pimpl->io_thread_.join();
}
}
