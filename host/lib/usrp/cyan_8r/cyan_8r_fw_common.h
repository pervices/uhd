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
 * Structs and constants for cyan_8r communication.
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
#define CYAN_8R_DEFAULT_MAC_ADDR_2          {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xaa}
#define CYAN_8R_DEFAULT_MAC_ADDR_3          {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x22}

#define CYAN_8R_DEFAULT_GATEWAY            (192 << 24 | 168 << 16 | 10  << 8  | 1 << 0)

#define CYAN_8R_DEFAULT_IP_ETH0_1G         (192 << 24 | 168 << 16 | 10  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH0_1G    (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH0_10G        (192 << 24 | 168 << 16 | 30  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH0_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH1_10G        (192 << 24 | 168 << 16 | 40  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH1_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH2_10G        (10 << 24 | 10 << 16 | 12  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH2_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_DEFAULT_IP_ETH3_10G        (10 << 24 | 10 << 16 | 13  << 8  | 2 << 0)
#define CYAN_8R_DEFAULT_NETMASK_ETH3_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_8R_RX_CHANNELS 8
#define CYAN_8R_TX_CHANNELS 0

#define CYAN_8R_FW_COMMS_FLAGS_ACK        (1 << 0)
#define CYAN_8R_FW_COMMS_FLAGS_ERROR      (1 << 1)
#define CYAN_8R_FW_COMMS_FLAGS_POKE32     (1 << 2)
#define CYAN_8R_FW_COMMS_FLAGS_PEEK32     (1 << 3)

// Cyan 8r min MTU size (typical ethernet frame)
#define CYAN_8R_MIN_MTU		1500
// Cyan 8r max MTU size (jumbo ethernet frame is 9000 bytes)
#define CYAN_8R_MAX_MTU		9000

//Cyan 8r Flowcontrol Update Per Second
#define CYAN_8R_UPDATE_PER_SEC	100
#define CYAN_8R_SS_FIFOLVL_THRESHOLD 107421875

// Cyan 8r Buffer Size
#define CYAN_8R_BUFF_SIZE	(2048*140*512/32)

// Cyan 8r RF Settings
#define CYAN_8R_RF_TX_GAIN_RANGE_START	0.0
#define CYAN_8R_RF_TX_GAIN_RANGE_STOP	31.75
#define CYAN_8R_RF_TX_GAIN_RANGE_STEP	0.25

// Ranges from, 0dB to 83.25dB
#define CYAN_8R_RF_RX_GAIN_RANGE_START -6
#define CYAN_8R_RF_RX_GAIN_RANGE_STOP	 33
#define CYAN_8R_RF_RX_GAIN_RANGE_STEP	 1

// Cyan_4r4t Clk Settings
#define CYAN_8R_MASTER_CLOCK_RATE	1000000000
#define CYAN_8R_DSP_CLOCK_RATE (CYAN_8R_MASTER_CLOCK_RATE/4)
#define CYAN_8R_TICK_RATE (CYAN_8R_MASTER_CLOCK_RATE/4)
#define CYAN_8R_EXT_CLK_RATE	10000000	// only 10 MHz input sources allowed
// Crimson Tuning Range Settings
#define CYAN_8R_FREQ_RANGE_START	0
#define CYAN_8R_FREQ_RANGE_STOP	18000000000.0
#define CYAN_8R_FREQ_RANGE_STEP	1.0
//Crimson LO Tuning Range Step Size
#define CYAN_8R_LO_STEPSIZE         25000000
#define CYAN_8R_LO_GUARDBAND	5000000
#define CYAN_8R_LO_OFFSET           25000000

// Crimson Sample Rate Settings
#define CYAN_8R_RATE_RANGE_START	(CYAN_8R_MASTER_CLOCK_RATE/65536)
#define CYAN_8R_RATE_RANGE_STOP_FULL	CYAN_8R_MASTER_CLOCK_RATE
#define CYAN_8R_RATE_RANGE_STOP_QUARTER     (CYAN_8R_MASTER_CLOCK_RATE/4.0)
#define CYAN_8R_RATE_RANGE_STEP	1.0

// All ADCs and DACs take complex sample at 325MSPS,
// and so all share a static front end bandwidth of 325MHz
// However, for user throughput, DACs A/B have a max user complex samplerate of
// 162.5MSPS, and DACs C/D have 81.25MSPS due to backhaul bandwidth limitations
// and FPGA transciever clocking limitaions.
#define CYAN_8R_ADC_BW                  (CYAN_8R_MASTER_CLOCK_RATE/2.0)
#define CYAN_8R_ADC_FREQ_RANGE_ROLLOFF      (0.8*CYAN_8R_ADC_BW)
#define CYAN_8R_BW_FULL                 (CYAN_8R_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_8R_BW_QUARTER              CYAN_8R_RATE_RANGE_STOP_QUARTER
#define CYAN_8R_BW_RANGE_STEP	1.0
#define CYAN_8R_ADC_FREQ_RANGE_STEP	        1.0

// There's a lower limit on the DC component we can pass. This is just an approximation.
#define CYAN_8R_DC_LOWERLIMIT	3000000
#define CYAN_8R_FM_LOWERLIMIT	86900000
#define CYAN_8R_FM_UPPERLIMIT	107900000

// Crimson DSP Freq Settings
// NCO mixing occurs after upconversion, limited by the FPGA/DAC bandwidth
#define CYAN_8R_DSP_BW_START    0
#define CYAN_8R_DSP_BW_STOP_FULL            CYAN_8R_BW_FULL
#define CYAN_8R_DSP_BW_STOP_QUARTER         CYAN_8R_BW_QUARTER
#define CYAN_8R_DSP_BW_STEPSIZE     1.0
#define CYAN_8R_DSP_FREQ_RANGE_START_FULL	(-CYAN_8R_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_8R_DSP_FREQ_RANGE_STOP_FULL	(CYAN_8R_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_8R_DSP_FREQ_RANGE_START_QUARTER	(-CYAN_8R_RATE_RANGE_STOP_QUARTER/2.0)
#define CYAN_8R_DSP_FREQ_RANGE_STOP_QUARTER	(CYAN_8R_RATE_RANGE_STOP_QUARTER/2.0)
#define CYAN_8R_DSP_FREQ_RANGE_STEP	1.0

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

// Constants to determine which frequency band to use
#define CYAN_8R_LOW_MID_BARRIER 500000000
#define CYAN_8R_MID_HIGH_BARRIER 6000000000

//the device name to get used in print messages
#define CYAN_8R_DEBUG_NAME_S "Cyan 8r"
#define CYAN_8R_DEBUG_NAME_C "CYAN_8R"

#define CYAN_8R_SUBDEV_SPEC_RX "A:Channel_A B:Channel_B C:Channel_C D:Channel_D E:Channel_E F:Channel_F G:Channel_G H:Channel_H"
#define CYAN_8R_SUBDEV_SPEC_TX ""

enum {
    LOW_BAND = 0,
    MID_BAND = 1,
    HIGH_BAND = 2,
};

#endif /* INCLUDED_CYAN_8R_FW_COMMON_H */
