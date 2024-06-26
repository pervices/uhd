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

#ifndef INCLUDED_CHESTNUT_FW_COMMON_H
#define INCLUDED_CHESTNUT_FW_COMMON_H

#include <stdint.h>

/*!
 * Structs and constants for chestnut communication.
 * This header is shared by the firmware and host code.
 * Therefore, this header may only contain valid C code.
 */

#define CMD_SUCCESS 	'0'
#define CMD_ERROR	'1'

#define CHESTNUT_FW_COMMS_MTU (1 << 13) //8k

#define CHESTNUT_FW_COMMS_UDP_PORT	42799

#define CHESTNUT_DEBUG_NAME_C "CHESTNUT"

#endif /* INCLUDED_CHESTNUT_FW_COMMON_H */
