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

#ifndef INCLUDED_CYAN_4R4T_3G_FW_COMMON_H
#define INCLUDED_CYAN_4R4T_3G_FW_COMMON_H

#include <stdint.h>

/*!
 * Structs and constants for cyan_4r4t_3g communication.
 * This header is shared by the firmware and host code.
 * Therefore, this header may only contain valid C code.
 */

#define CMD_SUCCESS 	'0'
#define CMD_ERROR	'1'

#define CYAN_4R4T_3G_FW_COMPAT_MAJOR 1
#define CYAN_4R4T_3G_FW_COMPAT_MINOR 0
#define CYAN_4R4T_3G_FPGA_COMPAT_MAJOR 1

#define CYAN_4R4T_3G_FW_NUM_BYTES (1 << 15) //64k
#define CYAN_4R4T_3G_FW_COMMS_MTU (1 << 13) //8k

#define CYAN_4R4T_3G_FW_COMMS_UDP_PORT	42799
#define CYAN_4R4T_3G_FLOW_CNTRL_UDP_PORT	42808

#define CYAN_4R4T_3G_DEFAULT_MAC_ADDR_0         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xff}
#define CYAN_4R4T_3G_DEFAULT_MAC_ADDR_1         {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x33}
#define CYAN_4R4T_3G_DEFAULT_MAC_ADDR_2          {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xaa}
#define CYAN_4R4T_3G_DEFAULT_MAC_ADDR_3          {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x22}

#define CYAN_4R4T_3G_DEFAULT_GATEWAY            (192 << 24 | 168 << 16 | 10  << 8  | 1 << 0)

#define CYAN_4R4T_3G_DEFAULT_IP_ETH0_1G         (192 << 24 | 168 << 16 | 10  << 8  | 2 << 0)
#define CYAN_4R4T_3G_DEFAULT_NETMASK_ETH0_1G    (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_3G_DEFAULT_IP_ETH0_10G        (192 << 24 | 168 << 16 | 30  << 8  | 2 << 0)
#define CYAN_4R4T_3G_DEFAULT_NETMASK_ETH0_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_3G_DEFAULT_IP_ETH1_10G        (192 << 24 | 168 << 16 | 40  << 8  | 2 << 0)
#define CYAN_4R4T_3G_DEFAULT_NETMASK_ETH1_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_3G_DEFAULT_IP_ETH2_10G        (10 << 24 | 10 << 16 | 12  << 8  | 2 << 0)
#define CYAN_4R4T_3G_DEFAULT_NETMASK_ETH2_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_3G_DEFAULT_IP_ETH3_10G        (10 << 24 | 10 << 16 | 13  << 8  | 2 << 0)
#define CYAN_4R4T_3G_DEFAULT_NETMASK_ETH3_10G   (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0)

#define CYAN_4R4T_3G_RX_CHANNELS 4
#define CYAN_4R4T_3G_TX_CHANNELS 4

#define CYAN_4R4T_3G_FW_COMMS_FLAGS_ACK        (1 << 0)
#define CYAN_4R4T_3G_FW_COMMS_FLAGS_ERROR      (1 << 1)
#define CYAN_4R4T_3G_FW_COMMS_FLAGS_POKE32     (1 << 2)
#define CYAN_4R4T_3G_FW_COMMS_FLAGS_PEEK32     (1 << 3)

// Cyan 4r4t min MTU size (typical ethernet frame)
#define CYAN_4R4T_3G_MIN_MTU		1500
// Cyan 4r4t max MTU size (jumbo ethernet frame is 9000 bytes)
// 3G is to fast for the FPGA to combine stuff across packets, a limit of 9000 would
#define CYAN_4R4T_3G_MAX_MTU		8962

// Cyan 4r4t Flowcontrol Update Per Second
#define CYAN_4R4T_3G_UPDATE_PER_SEC	100
#define CYAN_4R4T_3G_SS_FIFOLVL_THRESHOLD 107421875

// Cyan 4r4t Buffer Size
#define CYAN_4R4T_3G_BUFF_SIZE	(2048*140*512/32)
//how full the system shoudl try to keep the buffer
#define CYAN_4R4T_3G_BUFF_PERCENT 0.7

// Cyan 4r4t RF Settings
#define CYAN_4R4T_3G_RF_TX_GAIN_RANGE_START	0.0
#define CYAN_4R4T_3G_RF_TX_GAIN_RANGE_STOP	31.75
#define CYAN_4R4T_3G_RF_TX_GAIN_RANGE_STEP	0.25

//Most of the stuff relying on this has been moved to the server
#define CYAN_4R4T_3G_RF_RX_GAIN_RANGE_START -6
#define CYAN_4R4T_3G_RF_RX_GAIN_RANGE_STOP	 33
#define CYAN_4R4T_3G_RF_RX_GAIN_RANGE_STEP	 1

// Cyan 4r4t Clk Settings
// max number of samples per second
#define CYAN_4R4T_3G_MASTER_CLOCK_RATE	3000000000
#define CYAN_4R4T_3G_SAMPLES_PER_CLOCK 8
#define CYAN_4R4T_3G_DSP_CLOCK_RATE (CYAN_4R4T_3G_MASTER_CLOCK_RATE/CYAN_4R4T_3G_SAMPLES_PER_CLOCK)
// number of clock cycles per second
#define CYAN_4R4T_3G_TICK_RATE 250000000
#define CYAN_4R4T_3G_EXT_CLK_RATE	10000000	// only 10 MHz input sources allowed
// Cyan 4r4t Tuning Range Settings
#define CYAN_4R4T_3G_FREQ_RANGE_START	0
#define CYAN_4R4T_3G_FREQ_RANGE_STOP	20000000000.0
#define CYAN_4R4T_3G_FREQ_RANGE_STEP	1.0

// Cyan 4r4t Sample Rate Settings
#define CYAN_4R4T_3G_RATE_RANGE_START	(CYAN_4R4T_3G_MASTER_CLOCK_RATE/65536)
#define CYAN_4R4T_3G_RATE_RANGE_STOP_FULL	(CYAN_4R4T_3G_MASTER_CLOCK_RATE)
#define CYAN_4R4T_3G_RATE_RANGE_STOP_QUARTER     (CYAN_4R4T_3G_MASTER_CLOCK_RATE/4.0)
#define CYAN_4R4T_3G_RATE_RANGE_STEP	1.0

// All ADCs and DACs take complex sample at 325MSPS,
// and so all share a static front end bandwidth of 325MHz
// However, for user throughput, DACs A/B have a max user complex samplerate of
// 162.5MSPS, and DACs C/D have 81.25MSPS due to backhaul bandwidth limitations
// and FPGA transciever clocking limitaions.
#define CYAN_4R4T_3G_ADC_BW                  (CYAN_4R4T_3G_MASTER_CLOCK_RATE/2.0)
#define CYAN_4R4T_3G_ADC_FREQ_RANGE_ROLLOFF      (0.8*CYAN_4R4T_3G_ADC_BW)
#define CYAN_4R4T_3G_BW_FULL                 (CYAN_4R4T_3G_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_4R4T_3G_BW_QUARTER              CYAN_4R4T_3G_RATE_RANGE_STOP_QUARTER
#define CYAN_4R4T_3G_BW_RANGE_STEP	1.0
#define CYAN_4R4T_3G_ADC_FREQ_RANGE_STEP	        1.0

// There's a lower limit on the DC component we can pass. This is just an approximation.
#define CYAN_4R4T_3G_DC_LOWERLIMIT	3000000
#define CYAN_4R4T_3G_FM_LOWERLIMIT	86900000
#define CYAN_4R4T_3G_FM_UPPERLIMIT	107900000

// Cyan 4r4t DSP Freq Settings
// NCO mixing occurs after upconversion, limited by the FPGA/DAC bandwidth
#define CYAN_4R4T_3G_DSP_BW_START    0
#define CYAN_4R4T_3G_DSP_BW_STOP_FULL            CYAN_4R4T_3G_BW_FULL
#define CYAN_4R4T_3G_DSP_BW_STOP_QUARTER         CYAN_4R4T_3G_BW_QUARTER
#define CYAN_4R4T_3G_DSP_BW_STEPSIZE     1.0
#define CYAN_4R4T_3G_DSP_FREQ_RANGE_START_FULL	(-CYAN_4R4T_3G_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_4R4T_3G_DSP_FREQ_RANGE_STOP_FULL	(CYAN_4R4T_3G_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_4R4T_3G_DSP_FREQ_RANGE_START_QUARTER	(-CYAN_4R4T_3G_RATE_RANGE_STOP_QUARTER/2.0)
#define CYAN_4R4T_3G_DSP_FREQ_RANGE_STOP_QUARTER	(CYAN_4R4T_3G_RATE_RANGE_STOP_QUARTER/2.0)
#define CYAN_4R4T_3G_DSP_FREQ_RANGE_STEP	1.0

//Cyan 4r4t 3g LO Tuning Range Step Size
#define CYAN_4R4T_3G_LO_STEPSIZE         100000000
//sample rate will between 0 and CYAN_4R4T_3G_LO_DIFF_RANGE_START[0] will result in the lo being a close to CYAN_4R4T_3G_LO_DIFF[0] away from target frequency, while not excedding CYAN_4R4T_3G_LO_DIFF[0]
//ranges go from previous number (inclusive to the maximum of the range -1)
#define CYAN_4R4T_3G_LO_DIFF             {0}
#define CYAN_4R4T_3G_LO_DIFF_RANGE       {3000000001}
#define CYAN_4R4T_3G_MIN_LO CYAN_4R4T_3G_LO_STEPSIZE
#define CYAN_4R4T_3G_MAX_LO CYAN_4R4T_3G_FREQ_RANGE_STOP
#define CYAN_4R4T_3G_LO_GUARDBAND	5000000 //probably no longer needed
#define CYAN_4R4T_3G_LO_OFFSET           25000000 //probably no longer neded

//Cyan 4r4t VITA settings
#define CYAN_4R4T_3G_VITA_HDR_TYPE	0x1

#define CYAN_4R4T_3G_VITA_HDR
#define CYAN_4R4T_3G_VITA_STREAM

#define CYAN_4R4T_3G_VITA_TLR_EN	0xe00
#define CYAN_4R4T_3G_VITA_TLR_IND	0x0
#define CYAN_4R4T_3G_VITA_TLR_E	0x0
#define CYAN_4R4T_3G_VITA_TLR_PCKCNT	0x0
#define CYAN_4R4T_3G_VITA_TLR	((CYAN_4R4T_3G_VITA_TLR_EN     << 20)|\
				( CYAN_4R4T_3G_VITA_TLR_IND    << 8) |\
				( CYAN_4R4T_3G_VITA_TLR_E      << 7) |\
				( CYAN_4R4T_3G_VITA_TLR_PCKCNT << 0) )

// Constants to determine which frequency band to use
#define CYAN_4R4T_3G_LOW_MID_BARRIER 800000000
#define CYAN_4R4T_3G_MID_HIGH_BARRIER 6000000000

//the device name to get used in print messages
#define CYAN_4R4T_3G_DEBUG_NAME_S "Cyan 4r4t 3g"
#define CYAN_4R4T_3G_DEBUG_NAME_C "CYAN_4R4T_3G"

#define CYAN_4R4T_3G_SUBDEV_SPEC_RX "A:Channel_A B:Channel_B C:Channel_C D:Channel_D"
#define CYAN_4R4T_3G_SUBDEV_SPEC_TX "A:Channel_A B:Channel_B C:Channel_C D:Channel_D"

//The cpu scale factor still needs to be changed in io_impl
#define CYAN_4R4T_3G_OTW_RX "sc12"
#define CYAN_4R4T_3G_OTW_TX "sc16"

//The number corresponding to each band, eventually most of the stuff that relies on this should be moved to the server
enum {
    LOW_BAND = 0,
    MID_BAND = 1,
    HIGH_BAND = 2,
};

#endif /* INCLUDED_CYAN_4R4T_3G_FW_COMMON_H */
