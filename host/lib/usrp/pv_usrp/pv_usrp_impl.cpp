//
// Copyright 2014-2015 Per Vices Corporation
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <boost/assign.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <boost/foreach.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/endian/conversion.hpp>

#include "pv_usrp_impl.hpp"
#include "pv_usrp_fw_common.h"

#include "uhd/transport/if_addrs.hpp"
#include "uhd/transport/udp_simple.hpp"
#include "uhd/types/stream_cmd.hpp"
#include "uhd/utils/static.hpp"

#include <uhdlib/transport/udp_common.hpp>

namespace link_pv_usrp {
    const char *mtu_ref = "9000";
}

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace ph = std::placeholders;
namespace asio = boost::asio;

#ifndef DEFAULT_NUM_FRAMES
#define DEFAULT_NUM_FRAMES 32
#endif

#if 0
#ifndef
#define DEBUG_COUT
#endif
#endif

/***********************************************************************
 * Helper Functions
 **********************************************************************/
static void do_samp_rate_warning_message(
    double target_rate, double actual_rate, const std::string& xx, const size_t chan)
{
    static const double max_allowed_error = 1.0; // Sps
    if (std::abs(target_rate - actual_rate) > max_allowed_error) {
        UHD_LOGGER_WARNING("MULTI_USRP")
        << boost::format(
            "The hardware does not support the requested %s sample rate on ch %li:\n"
            "Target sample rate: %f MSps\n"
            "Actual sample rate: %f MSps\n")
        % xx % chan % (target_rate / 1e6) % (actual_rate / 1e6);
    }
}

static std::string mb_root(const size_t mboard = 0) {
    return "/mboards/" + std::to_string(mboard);
}

static std::string rx_dsp_root(const size_t channel, const size_t mboard = 0) {
    return mb_root(mboard) + "/rx_dsps/" + std::to_string(channel);
}

static std::string rx_rf_fe_root(const size_t channel, const size_t mboard = 0) {
    auto letter = std::string(1, 'A' + channel);
    return mb_root(mboard) + "/dboards/" + letter + "/rx_frontends/Channel_" + letter;
}

std::string pv_usrp_impl::rx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/rx_link/" + std::to_string(channel);
}

std::string pv_usrp_impl::tx_link_root(const size_t channel, const size_t mboard)
{
    return mb_root(mboard) + "/tx_link/" + std::to_string(channel);
}

std::string pv_usrp_impl::tx_dsp_root(const size_t channel, const size_t mboard) {
    return mb_root(mboard) + "/tx_dsps/" + std::to_string(channel);
}

static std::string tx_rf_fe_root(const size_t channel, const size_t mboard = 0) {
    auto letter = std::string(1, 'A' + channel);
    return mb_root(mboard) + "/dboards/" + letter + "/tx_frontends/Channel_" + letter;
}

//figures out the channel number from the rx stream path
static size_t pre_to_ch( const std::string & pre ) {
    char x = -1;
    if ( 1 != sscanf( pre.c_str(), "rx_%c/stream", & x) ) {
        throw value_error( "Invalid 'pre' argument '" + pre + "'" );
    }
    size_t ch = x - 'a';

    return ch;
}

// TODO: set_stream_cmd here


