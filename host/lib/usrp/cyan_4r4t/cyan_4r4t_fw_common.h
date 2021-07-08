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

#ifndef INCLUDED_CYAN_4R4T_FW_COMMON_H
#define INCLUDED_CYAN_4R4T_FW_COMMON_H

#include <stdint.h>

/*!
 * Structs and constants for cyan_4r4t communication.
 * This header is shared by the firmware and host code.
 * Therefore, this header may only contain valid C code.
 */

#define CMD_SUCCESS 	'0'
#define CMD_ERROR	'1'

#define CYAN_4R4T_FW_COMPAT_MAJOR 1
#define CYAN_4R4T_FW_COMPAT_MINOR 0
#define CYAN_4R4T_FPGA_COMPAT_MAJOR 1

#define CYAN_4R4T_FW_NUM_BYTES (1 << 15) //64k
#define CYAN_4R4T_FW_COMMS_MTU (1 << 13) //8k

#define CYAN_4R4T_FW_COMMS_UDP_PORT	42799
#define CYAN_4R4T_FLOW_CNTRL_UDP_PORT	42808

#define CYAN_4R4T_DEFAULT_MAC_ADDR_0         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xff}
#define CYAN_4R4T_DEFAULT_MAC_ADDR_1         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x33}
#define CYAN_4R4T_DEFAULT_MAC_ADDR_2          {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xaa}
#define CYAN_4R4T_DEFAULT_MAC_ADDR_3          {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x22}

#define CYAN_4R4T_DEFAULT_GATEWAY            (192 << 24 | 168 << 16 | 10  << 8  | 1 << 0)

#define CYAN_4R4T_DEFAULT_IP_ETH0_1G         (192 << 24 | 168 << 16 | 10  << 8  | 2 << 0)
#define CYAN_4R4T_DEFAULT_NETMASK_ETH0_1G    (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_DEFAULT_IP_ETH0_10G        (192 << 24 | 168 << 16 | 30  << 8  | 2 << 0)
#define CYAN_4R4T_DEFAULT_NETMASK_ETH0_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_DEFAULT_IP_ETH1_10G        (192 << 24 | 168 << 16 | 40  << 8  | 2 << 0)
#define CYAN_4R4T_DEFAULT_NETMASK_ETH1_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_DEFAULT_IP_ETH2_10G        (10 << 24 | 10 << 16 | 12  << 8  | 2 << 0)
#define CYAN_4R4T_DEFAULT_NETMASK_ETH2_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_DEFAULT_IP_ETH3_10G        (10 << 24 | 10 << 16 | 13  << 8  | 2 << 0)
#define CYAN_4R4T_DEFAULT_NETMASK_ETH3_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_RX_CHANNELS 4
#define CYAN_4R4T_TX_CHANNELS 4

#define CYAN_4R4T_FW_COMMS_FLAGS_ACK        (1 << 0)
#define CYAN_4R4T_FW_COMMS_FLAGS_ERROR      (1 << 1)
#define CYAN_4R4T_FW_COMMS_FLAGS_POKE32     (1 << 2)
#define CYAN_4R4T_FW_COMMS_FLAGS_PEEK32     (1 << 3)

// Crimson min MTU size (typical ethernet frame)
#define CYAN_4R4T_MIN_MTU		1500
// Crimson max MTU size (jumbo ethernet frame is 9000 bytes)
#define CYAN_4R4T_MAX_MTU		9000

// Crimson Flowcontrol Update Per Second
//DWF
#define CYAN_4R4T_UPDATE_PER_SEC	100
#define CYAN_4R4T_SS_FIFOLVL_THRESHOLD 107421875

// Cyan_4r4t Buffer Size
#define CYAN_4R4T_BUFF_SIZE	2048*140*512/32

// Crimson RF Settings
//DWF
#define CYAN_4R4T_RF_TX_GAIN_RANGE_START	0.0
#define CYAN_4R4T_RF_TX_GAIN_RANGE_STOP	31.75
#define CYAN_4R4T_RF_TX_GAIN_RANGE_STEP	0.25

// Ranges from, 0dB to 83.25dB
//DWF
#define CYAN_4R4T_RF_RX_GAIN_RANGE_START	0.0
#define CYAN_4R4T_RF_RX_GAIN_RANGE_STOP	83.25
#define CYAN_4R4T_RF_RX_GAIN_RANGE_STEP	0.25

// Cyan_4r4t Clk Settings
#define CYAN_4R4T_MASTER_CLOCK_RATE	1000000000
#define CYAN_4R4T_EXT_CLK_RATE	10000000	// only 10 MHz input sources allowed
// Crimson Tuning Range Settings
#define CYAN_4R4T_FREQ_RANGE_START	0
//DWFC
#define CYAN_4R4T_FREQ_RANGE_STOP	18000000000.0
#define CYAN_4R4T_FREQ_RANGE_STEP	1
//Crimson LO Tuning Range Step Size
#define CYAN_4R4T_LO_STEPSIZE         25000000
#define CYAN_4R4T_LO_GUARDBAND	5000000
#define CYAN_4R4T_LO_OFFSET           25000000

// Crimson Sample Rate Settings
#define CYAN_4R4T_RATE_RANGE_START	CYAN_4R4T_MASTER_CLOCK_RATE/65536
#define CYAN_4R4T_RATE_RANGE_STOP_FULL	CYAN_4R4T_MASTER_CLOCK_RATE
#define CYAN_4R4T_RATE_RANGE_STOP_QUARTER     CYAN_4R4T_MASTER_CLOCK_RATE/4.0
#define CYAN_4R4T_RATE_RANGE_STEP	1.0

// All ADCs and DACs take complex sample at 325MSPS,
// and so all share a static front end bandwidth of 325MHz
// However, for user throughput, DACs A/B have a max user complex samplerate of
// 162.5MSPS, and DACs C/D have 81.25MSPS due to backhaul bandwidth limitations
// and FPGA transciever clocking limitaions.
#define CYAN_4R4T_ADC_BW                  CYAN_4R4T_MASTER_CLOCK_RATE/2.0
#define CYAN_4R4T_ADC_FREQ_RANGE_ROLLOFF      0.8*CYAN_4R4T_ADC_BW
#define CYAN_4R4T_BW_FULL                 CYAN_4R4T_RATE_RANGE_STOP_FULL/2.0
#define CYAN_4R4T_BW_QUARTER              CYAN_4R4T_RATE_RANGE_STOP_QUARTER
#define CYAN_4R4T_BW_RANGE_STEP	1.0
#define CYAN_4R4T_ADC_FREQ_RANGE_STEP	        1.0

// There's a lower limit on the DC component we can pass. This is just an approximation.
//DWF
#define CYAN_4R4T_DC_LOWERLIMIT	3000000
#define CYAN_4R4T_FM_LOWERLIMIT	86900000
#define CYAN_4R4T_FM_UPPERLIMIT	107900000

// Crimson DSP Freq Settings
// NCO mixing occurs after upconversion, limited by the FPGA/DAC bandwidth
//DWF
#define CYAN_4R4T_DSP_BW_START    0
#define CYAN_4R4T_DSP_BW_STOP_FULL            CYAN_4R4T_BW_FULL
#define CYAN_4R4T_DSP_BW_STOP_QUARTER         CYAN_4R4T_BW_QUARTER
#define CYAN_4R4T_DSP_BW_STEPSIZE     1.0
//DWF
#define CYAN_4R4T_DSP_FREQ_RANGE_START_FULL	-CYAN_4R4T_RATE_RANGE_STOP_FULL/2.0
#define CYAN_4R4T_DSP_FREQ_RANGE_STOP_FULL	CYAN_4R4T_RATE_RANGE_STOP_FULL/2.0
#define CYAN_4R4T_DSP_FREQ_RANGE_START_QUARTER	-CYAN_4R4T_RATE_RANGE_STOP_QUARTER/2.0
#define CYAN_4R4T_DSP_FREQ_RANGE_STOP_QUARTER	CYAN_4R4T_RATE_RANGE_STOP_QUARTER/2.0
#define CYAN_4R4T_DSP_FREQ_RANGE_STEP	1.0

// Crimson VITA settings
#define CYAN_4R4T_VITA_HDR_TYPE	0x1

#define CYAN_4R4T_VITA_HDR
#define CYAN_4R4T_VITA_STREAM

#define CYAN_4R4T_VITA_TLR_EN	0xe00
#define CYAN_4R4T_VITA_TLR_IND	0x0
#define CYAN_4R4T_VITA_TLR_E	0x0
#define CYAN_4R4T_VITA_TLR_PCKCNT	0x0
#define CYAN_4R4T_VITA_TLR	((CYAN_4R4T_VITA_TLR_EN     << 20)|\
				( CYAN_4R4T_VITA_TLR_IND    << 8) |\
				( CYAN_4R4T_VITA_TLR_E      << 7) |\
				( CYAN_4R4T_VITA_TLR_PCKCNT << 0) )

#endif /* INCLUDED_CYAN_4R4T_FW_COMMON_H */
