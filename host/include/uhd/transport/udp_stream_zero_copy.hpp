//
// Copyright 2010 Ettus Research LLC
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

#ifndef INCLUDED_UHD_TRANSPORT_udp_stream_zero_copy_HPP
#define INCLUDED_UHD_TRANSPORT_udp_stream_zero_copy_HPP

#include <uhd/config.hpp>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/types/device_addr.hpp>
#include <boost/shared_ptr.hpp>

namespace uhd{ namespace transport{

/*!
 *
 */
class UHD_API udp_stream_zero_copy : public virtual udp_zero_copy{
public:
    typedef std::shared_ptr<udp_stream_zero_copy> sptr;

    /**
     *
     * @param local_addr
     * @param local_port
     * @param remote_addr
     * @param remote_port
     * @param default_buff_args
     * @param buff_params_out
     * @param hints
     * @return
     */
    static sptr make(
        const std::string &local_addr,
        const uint16_t local_port,
        const std::string &remote_addr,
        const uint16_t remote_port,
        const zero_copy_xport_params &default_buff_args,
        udp_zero_copy::buff_params& buff_params_out,
        const device_addr_t &hints = device_addr_t()
    );
};

}} //namespace

#endif /* INCLUDED_UHD_TRANSPORT_udp_stream_zero_copy_HPP */
