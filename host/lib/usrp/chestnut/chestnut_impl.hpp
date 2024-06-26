//
// Copyright 2024 Per Vices Corporation
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

#ifndef INCLUDED_CHESTNUT_IMPL_HPP
#define INCLUDED_CHESTNUT_IMPL_HPP

#include "../cyan_nrnt/cyan_nrnt_impl.hpp"


namespace uhd {
namespace usrp {

class chestnut_impl : public uhd::usrp::cyan_nrnt_impl
{

public:
    // This is the core constructor to be called when a cyan_nrnt device is found
    chestnut_impl(const uhd::device_addr_t &, const bool use_dpdk);
    ~chestnut_impl(void);

    static device_addrs_t chestnut_find_with_addr(const device_addr_t &hint);
    static device_addrs_t chestnut_find(const device_addr_t &hint_);
};

}
}

#endif /* INCLUDED_CHESTNUT_IMPL_HPP */
