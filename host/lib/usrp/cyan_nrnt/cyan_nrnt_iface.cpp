//
// Copyright 2014-2015 Per Vices Corporation
// Copyright 2022 Per Vices Corporation
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

#include <uhd/exception.hpp>
#include <uhd/utils/paths.hpp>
#include <uhd/utils/tasks.hpp>
//#include <uhd/utils/images.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/types/dict.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp> //used for htonl and ntohl
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <functional>
#include <boost/tokenizer.hpp>
#include <boost/functional/hash.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iostream>
#include <inttypes.h>
#include <uhd/utils/platform.hpp>
#include "cyan_nrnt_fw_common.h"
#include "cyan_nrnt_iface.hpp"

using namespace uhd;
using namespace uhd::transport;

static uint32_t seq = 1;

/***********************************************************************
 * Structors
 **********************************************************************/
cyan_nrnt_iface::cyan_nrnt_iface(udp_simple::sptr ctrl_transport):
    _ctrl_transport(ctrl_transport),
    _ctrl_seq_num(0),
    _protocol_compat(0)
{
    memset( _buff, '\0', sizeof( _buff ) );
}

/***********************************************************************
 * Peek and Poke
 **********************************************************************/
// Never call this function by itself, always call through cyan_nrnt_impl::get/set()
// else it will mess up the protocol with the sequencing and will contian no error checks.
void cyan_nrnt_iface::poke_str(std::string data) {
    // populate the command string with sequence number
    data = data.insert(0, (boost::lexical_cast<std::string>(seq++) + ","));
    _ctrl_transport->send( boost::asio::buffer(data, data.length()) );
    return;
}

// Never call this function by itself, always call through cyan_nrnt_impl::get/set(),
// else it will mess up the protocol with the sequencing and will contian no error checks.
// Format: <sequence number>,<error code>,<data>
std::string cyan_nrnt_iface::peek_str( float timeout_s ) {
    uint32_t iseq;
    std::vector<std::string> tokens;
    uint8_t tries = 0;
    uint8_t num_tries = 5;

    do {
        // clears the buffer and receives the message
        memset( _buff, 0, sizeof( _buff ) );
        const size_t nbytes = _ctrl_transport -> recv(boost::asio::buffer(_buff), timeout_s );
        if (nbytes == 0) return "TIMEOUT";

        // parses it through tokens: seq, status, [data]
        this -> parse(tokens, _buff, ',');

        // Malformed packet
        if (tokens.size() < 2) {
            return "ERROR";
        // Error while reading property
        } else if(tokens[1].c_str()[0] == CMD_ERROR) {
            return "GET_ERROR";
        }

        // if seq is incorrect, return an error
        sscanf(tokens[0].c_str(), "%" SCNd32, &iseq);

    } while(iseq != seq - 1 && tries++ < num_tries);

    // exits with an error if can't find a matching sequence
    if (tries == num_tries) return "INVLD_SEQ";

    // Return the message (tokens[2])
    // If the error checking passed and tokens is size 2, then the intended data is "" (since parse will not add "" if the packets ends with a ","
    if(tokens.size() < 3) {
        return "";
    } else {
        return tokens[2];
    }
}

std::string cyan_nrnt_iface::peek_str() {
	return peek_str( 8 );
}

/***********************************************************************
 * Public make function for cyan_nrnt interface
 **********************************************************************/
cyan_nrnt_iface::sptr cyan_nrnt_iface::make(udp_simple::sptr ctrl_transport){
    return cyan_nrnt_iface::sptr(new cyan_nrnt_iface(ctrl_transport));
}

/***********************************************************************
 * Helper Functions
 **********************************************************************/
void cyan_nrnt_iface::parse(std::vector<std::string> &tokens, char* data, const char delim) {
	int i = 0;
	while (data[i]) {
		std::string token = "";
		while (data[i] && data[i] != delim) {
			token.push_back(data[i]);
			if (data[i+1] == 0 || data[i+1] == delim)
				tokens.push_back(token);
			i++;
		}
		i++;
	}
	return;
}
