//
// Copyright 2014 Per Vices Corporation
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

#ifndef INCLUDED_CYAN_8R_IFACE_HPP
#define INCLUDED_CYAN_8R_IFACE_HPP

#include <uhd/transport/udp_simple.hpp>
#include <uhd/types/serial.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include <uhd/types/wb_iface.hpp>
#include <string>
#include "cyan_8r_fw_common.h"

namespace uhd {

/*!
 * The cyan_8r interface class:
 * Provides a set of functions to UDP implementation layer.
 */
class cyan_8r_iface : public uhd::wb_iface
{
public:
    typedef boost::shared_ptr<cyan_8r_iface> sptr;
    /*!
     * Make a new cyan_8r interface with the control transport.
     * \param ctrl_transport the udp transport object
     * \return a new cyan_8r interface object
     */
    cyan_8r_iface(uhd::transport::udp_simple::sptr ctrl_transport);

    static cyan_8r_iface::sptr make(uhd::transport::udp_simple::sptr ctrl_transport);

    // Send/write a data packet (string), null terminated
    virtual void poke_str(std::string data);

    // Recieve/read a data packet (string), null terminated
    virtual std::string peek_str(void);

    // Recieve/read a data packet (string), null terminated
    virtual std::string peek_str( float timeout_s );

private:
    //this lovely lady makes it all possible
    uhd::transport::udp_simple::sptr _ctrl_transport;

    // add another transport for streaming

    // internal function for tokenizing the inputs
    void parse(std::vector<std::string> &tokens, char* data, const char delim);

    //used in send/recv
    boost::uint32_t _ctrl_seq_num;
    boost::uint32_t _protocol_compat;

    // buffer for in and out
    char _buff[ CYAN_8R_MAX_MTU ];
};

}

#endif /* INCLUDED_CYAN_8R_IFACE_HPP */
