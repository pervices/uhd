//
// Copyright 2025 Per Vices Corporation
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

#pragma once

#include <stdint.h>

#define PV_USRP_MB_PATH     fs_path("/mboards/0")
#define PV_USRP_TIME_PATH   fs_path(PV_USRP_MB_PATH / "time")

#define PV_USRP_FW_COMMS_MTU (1 << 13) //8k

#define PV_USRP_FW_COMMS_UDP_PORT 42799

// Common VITA settings
#define PV_USRP_VITA_TLR_EN     0xe00
#define PV_USRP_VITA_TLR_IND    0x0
#define PV_USRP_VITA_TLR_E      0x0
#define PV_USRP_VITA_TLR_PCKCNT 0x0
#define PV_USRP_VITA_TLR        ((PV_USRP_VITA_TLR_EN     << 20)|\
                                ( PV_USRP_VITA_TLR_IND    << 8) |\
                                ( PV_USRP_VITA_TLR_E      << 7) |\
                                ( PV_USRP_VITA_TLR_PCKCNT << 0) )



