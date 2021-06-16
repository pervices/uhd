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

#ifndef INCLUDED_CYAN_8R_FW_COMMON_H
#define INCLUDED_CYAN_8R_FW_COMMON_H

#include <stdint.h>

/*!
 * Structs and constants for crimson_tng communication.
 * This header is shared by the firmware and host code.
 * Therefore, this header may only contain valid C code.
 */

#define CMD_SUCCESS 	'0'
#define CMD_ERROR	'1'

#define CYAN_8R_FW_COMPAT_MAJOR 1
#define CYAN_8R_FW_COMPAT_MINOR 0
#define CYAN_8R_FPGA_COMPAT_MAJOR 1

#define CYAN_8R_FW_NUM_BYTES (1 << 15) //64k
#define CYAN_8R_FW_COMMS_MTU (1 << 13) //8k

#define CYAN_8R_FW_COMMS_UDP_PORT	42799
#define CYAN_8R_FLOW_CNTRL_UDP_PORT	42808

#define CYAN_8R_DEFAULT_MAC_ADDR_0         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xff}
#define CYAN_8R_DEFAULT_MAC_ADDR_1         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x33}
#define CYAN_8R_DEFAULT_MAC_ADDR_2         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xaa}
#define CYAN_8R_DEFAULT_MAC_ADDR_3         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x22}

#define CYAN_8R_DEFAULT_GATEWAY            (192 << 24 | 168 << 16 | 10  << 8  | 1 << 0)

#define CYAN_8R_DEFAULT_IP_ETH0_1G         (192 << 24 | 168 << 16 | 10  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH0_1G    (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH0_10G        (10 << 24 | 10 << 16 | 10  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH0_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH1_10G        (10 << 24 | 10 << 16 | 11  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH1_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH2_10G        (10 << 24 | 10 << 16 | 12  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH2_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH3_10G        (10 << 24 | 10 << 16 | 13  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH3_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_MASTER_CLOCK_RATE	        1000000000
#define CYAN_8R_DSP_CLOCK_RATE              250000000
#define CYAN_8R_SAMPS_PER_DSP_TICK  4
#define CYAN_8R_RX_CHANNELS 8
#define CYAN_8R_TX_CHANNELS 8 //TODO: add warning to user if they try to use tx on and r variant
#define CYAN_8R_DSP_FREQ_RANGE_START (-(CYAN_8R_MASTER_CLOCK_RATE/2))
#define CYAN_8R_DSP_FREQ_RANGE_STOP	CYAN_8R_MASTER_CLOCK_RATE/2
#define CYAN_8R_RATE_RANGE_START	CYAN_8R_MASTER_CLOCK_RATE/65536
#define CYAN_8R_RATE_RANGE_STOP		CYAN_8R_MASTER_CLOCK_RATE
#define CYAN_8R_FREQ_RANGE_STOP		18000000000.0

// Crimson DSP Freq Settings
#define CYAN_8R_DSP_FREQ_RANGE_STEP	        1.0

// Crimson Rate Settings
#define CYAN_8R_RATE_RANGE_STEP		1.0

// Crimson Freq Settings
#define CYAN_8R_FREQ_RANGE_START	0
#define CYAN_8R_FREQ_RANGE_STEP		1.0

// Crimson Ext Ref Clock
#define CYAN_8R_EXT_CLK_RATE		10000000	// only 10 MHz input sources allowed

#define CYAN_8R_FW_COMMS_FLAGS_ACK        (1 << 0)
#define CYAN_8R_FW_COMMS_FLAGS_ERROR      (1 << 1)
#define CYAN_8R_FW_COMMS_FLAGS_POKE32     (1 << 2)
#define CYAN_8R_FW_COMMS_FLAGS_PEEK32     (1 << 3)

// Crimson min MTU size (typical ethernet frame)
#define CYAN_8R_MIN_MTU		1500
// Crimson max MTU size (jumbo ethernet frame is 9000 bytes)
#define CYAN_8R_MAX_MTU		9000

// Crimson Flowcontrol Update Per Second
#define CYAN_8R_UPDATE_PER_SEC	100
#define CYAN_8R_SS_FIFOLVL_THRESHOLD 107421875

// Crimson Buffer Size
#define CYAN_8R_BUFF_SIZE	(65536*4)

// Crimson RF Settings
#define CYAN_8R_RF_TX_GAIN_RANGE_START	0.0
#define CYAN_8R_RF_TX_GAIN_RANGE_STOP	31.75
#define CYAN_8R_RF_TX_GAIN_RANGE_STEP	0.25

// Ranges from, 0dB to 83.25dB
#define CYAN_8R_RF_RX_GAIN_RANGE_START	0.0
#define CYAN_8R_RF_RX_GAIN_RANGE_STOP	83.25
#define CYAN_8R_RF_RX_GAIN_RANGE_STEP	0.25

// Crimson VITA settings
#define CYAN_8R_VITA_HDR_TYPE	0x1

#define CYAN_8R_VITA_HDR
#define CYAN_8R_VITA_STREAM

#define CYAN_8R_VITA_TLR_EN	0xe00
#define CYAN_8R_VITA_TLR_IND	0x0
#define CYAN_8R_VITA_TLR_E	0x0
#define CYAN_8R_VITA_TLR_PCKCNT	0x0
#define CYAN_8R_VITA_TLR	((CYAN_8R_VITA_TLR_EN     << 20)|\
				( CYAN_8R_VITA_TLR_IND    << 8) |\
				( CYAN_8R_VITA_TLR_E      << 7) |\
				( CYAN_8R_VITA_TLR_PCKCNT << 0) )

#endif /* INCLUDED_CYAN_8R_FW_COMMON_H */
