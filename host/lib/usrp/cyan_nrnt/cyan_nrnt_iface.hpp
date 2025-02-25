//
// Copyright 2022, 2024 Per Vices Corporation
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

#ifndef INCLUDED_CYAN_NRNT_IFACE_HPP
#define INCLUDED_CYAN_NRNT_IFACE_HPP

#include <uhd/transport/udp_simple.hpp>
#include <string>
#include "cyan_nrnt_fw_common.h"
#include <mutex>

// Include types that can be accessed
#include "uhd/usrp/mboard_eeprom.hpp"
#include "uhd/usrp/dboard_eeprom.hpp"
#include <uhd/types/sensors.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/utils/noncopyable.hpp>

namespace uhd {

/*!
 * The cyan_nrnt interface class:
 * Provides a set of functions access the state tree
 */
class cyan_nrnt_iface : uhd::noncopyable
{
public:
    typedef std::shared_ptr<cyan_nrnt_iface> sptr;
    /*!
     * Make a new cyan_nrnt interface with the control transport.
     * \param ctrl_transport the udp transport object
     * \return a new cyan_nrnt interface object
     */
    cyan_nrnt_iface(uhd::transport::udp_simple::sptr ctrl_transport);

    static cyan_nrnt_iface::sptr make(uhd::transport::udp_simple::sptr ctrl_transport);

    // Helper functions to wrap peek_str and poke_str as get and set
    //
    //
    //

    std::string get_string(std::string req);
    void set_string(const std::string pre, std::string data);

    // wrapper for type <double> through the ASCII Crimson interface
    double get_double(std::string req);
    void set_double(const std::string pre, double data);

    // wrapper for type <bool> through the ASCII Crimson interface
    bool get_bool(std::string req);
    void set_bool(const std::string pre, bool data);

    // wrapper for type <int> through the ASCII Crimson interface
    int get_int(std::string req);
    void set_int(const std::string pre, int data);

    // wrapper for type <mboard_eeprom_t> through the ASCII Crimson interface
    uhd::usrp::mboard_eeprom_t get_mboard_eeprom(std::string req);
    void set_mboard_eeprom(const std::string pre, uhd::usrp::mboard_eeprom_t data);

    // wrapper for type <dboard_eeprom_t> through the ASCII Crimson interface
    uhd::usrp::dboard_eeprom_t get_dboard_eeprom(std::string req);
    void set_dboard_eeprom(const std::string pre, uhd::usrp::dboard_eeprom_t data);

    // wrapper for type <sensor_value_t> through the ASCII Crimson interface
    uhd::sensor_value_t get_sensor_value(std::string req);
    void set_sensor_value(const std::string pre, uhd::sensor_value_t data);

    // wrapper for type <time_spec_t> through the ASCII Crimson interface
    uhd::time_spec_t get_time_spec(std::string req);
    void set_time_spec(const std::string pre, uhd::time_spec_t data);

private:
    // Mutex for controlling access to the management port
    std::mutex _iface_lock;

    // Send/write a data packet (string), null terminated
    void poke_str(std::string data);

    // Recieve/read a data packet (string), null terminated
    std::string peek_str(void);

    // Recieve/read a data packet (string), null terminated
    std::string peek_str( float timeout_s );

    //this lovely lady makes it all possible
    uhd::transport::udp_simple::sptr _ctrl_transport;

    // add another transport for streaming

    // internal function for tokenizing the inputs
    void parse(std::vector<std::string> &tokens, char* data, size_t const data_len, const char delim);

    //used in send/recv
    uint32_t _ctrl_seq_num;
    uint32_t _protocol_compat;

    // buffer for in and out
    char _buff[ CYAN_NRNT_MAX_MTU ];
};

}

#endif /* INCLUDED_CYAN_NRNT_IFACE_HPP */
