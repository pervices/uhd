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

#ifndef INCLUDED_CYAN_NRNT_FW_COMMON_H
#define INCLUDED_CYAN_NRNT_FW_COMMON_H

#include <stdint.h>

/*!
 * Structs and constants for cyan_nrnt communication.
 * This header is shared by the firmware and host code.
 * Therefore, this header may only contain valid C code.
 */

#define CMD_SUCCESS 	'0'
#define CMD_ERROR	'1'

#define CYAN_NRNT_FW_COMMS_MTU (1 << 13) //8k

#define CYAN_NRNT_FW_COMMS_UDP_PORT	42799

// Cyan NrNt max MTU size (jumbo ethernet frame is 9000 bytes)
#define CYAN_NRNT_MAX_MTU		8992

// Maximum number of bytes used to store samples in a tx packet
#define CYAN_NRNT_MAX_SEND_SAMPLE_BYTES 8912

// Maximum number of bytes used to store sample data when receiving
#define CYAN_NRNT_MAX_NBYTES 8880

// Size of the vrt header in bytes
#define CYAN_NRNT_HEADER_SIZE 16

// Size of the vrt trailer in bytes
#define CYAN_NRNT_TRAILER_SIZE 0

// Packaets send to the unit must have a multiple of this many samples
#define CYAN_NRNT_PACKET_NSAMP_MULTIPLE 4
// Amount of the UDP packet used for overhead in data packets
// 60 bit ipv4 header, 8 bit UDP header
#define CYAN_NRNT_UDP_OVERHEAD 68

// Cyan NrNt Flowcontrol Update Per Second
#define CYAN_NRNT_UPDATE_PER_SEC	100

//how full UHD should try to keep the buffer
#define CYAN_NRNT_BUFF_PERCENT 0.7

// Cyan NrNt tx RF gain limits, this is definitely wrong
// TODO: get this info from device
#define CYAN_NRNT_RF_TX_GAIN_RANGE_START	0.0
#define CYAN_NRNT_RF_TX_GAIN_RANGE_STOP	31.75
#define CYAN_NRNT_RF_TX_GAIN_RANGE_STEP	0.25

// Cyan NrNt rx RF gain limits, this is definitely wrong
//TODO:get this info from device
#define CYAN_NRNT_RF_RX_GAIN_RANGE_START -6
#define CYAN_NRNT_RF_RX_GAIN_RANGE_STOP	 33
#define CYAN_NRNT_RF_RX_GAIN_RANGE_STEP	 1

// Cyan NrNt Clk Settings
// number of clock cycles per second
#define CYAN_NRNT_TICK_RATE 250000000
#define CYAN_NRNT_EXT_CLK_RATE	10000000	// only 10 MHz input sources allowed
// Cyan NrNt Tuning Range Settings
#define CYAN_NRNT_FREQ_RANGE_START	0
#define CYAN_NRNT_FREQ_RANGE_STOP	20000000000.0
// The step is wrong, the nco will often end ep with decimal results
#define CYAN_NRNT_FREQ_RANGE_STEP	1.0

// Cyan NrNt Sample Rate Settings
#define CYAN_NRNT_RATE_RANGE_START	(max_sample_rate/65536)
#define CYAN_NRNT_RATE_RANGE_STOP_FULL	max_sample_rate
#define CYAN_NRNT_RATE_RANGE_STOP_QUARTER     (max_sample_rate/4.0)
#define CYAN_NRNT_RATE_RANGE_STEP	(max_sample_rate/65536 - (max_sample_rate/65536))

// Cyan NrNt DSP Freq Settings
// NCO mixing occurs after upconversion, limited by the FPGA/DAC bandwidth
#define CYAN_NRNT_BW_FULL                 (CYAN_NRNT_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_NRNT_DSP_BW_START    0
#define CYAN_NRNT_DSP_BW_STOP_FULL            CYAN_NRNT_BW_FULL
#define CYAN_NRNT_DSP_BW_STEPSIZE     1.0
#define CYAN_NRNT_DSP_FREQ_RANGE_START_FULL	(-CYAN_NRNT_RATE_RANGE_STOP_FULL/2.0)
#define CYAN_NRNT_DSP_FREQ_RANGE_STOP_FULL	(CYAN_NRNT_RATE_RANGE_STOP_FULL/2.0)
// The step is wrong, the nco will often end ep with decimal results
#define CYAN_NRNT_DSP_FREQ_RANGE_STEP	1.0

//Cyan NrNt LO Tuning Range Step Size
#define CYAN_NRNT_LO_STEPSIZE         100000000
//sample rate will between 0 and CYAN_NRNT_LO_DIFF_RANGE_START[0] will result in the lo being a close to CYAN_NRNT_LO_DIFF[0] away from target frequency, while not excedding CYAN_NRNT_LO_DIFF[0]
//ranges go from previous number (inclusive to the maximum of the range -1)
#define CYAN_NRNT_LO_DIFF             {200000000, 100000000, 0}
#define CYAN_NRNT_LO_DIFF_RANGE       {200000000, 500000000, 1000000001}
#define CYAN_NRNT_MIN_LO CYAN_NRNT_LO_STEPSIZE
#define CYAN_NRNT_MAX_LO CYAN_NRNT_FREQ_RANGE_STOP

//Cyan NrNt VITA settings

#define CYAN_NRNT_VITA_TLR_EN	0xe00
#define CYAN_NRNT_VITA_TLR_IND	0x0
#define CYAN_NRNT_VITA_TLR_E	0x0
#define CYAN_NRNT_VITA_TLR_PCKCNT	0x0
#define CYAN_NRNT_VITA_TLR	((CYAN_NRNT_VITA_TLR_EN     << 20)|\
				( CYAN_NRNT_VITA_TLR_IND    << 8) |\
				( CYAN_NRNT_VITA_TLR_E      << 7) |\
				( CYAN_NRNT_VITA_TLR_PCKCNT << 0) )

// Constants to determine which frequency band to use
#define CYAN_NRNT_LOW_MID_BARRIER 400000000
#define CYAN_NRNT_MID_HIGH_BARRIER 5800000000

//the device name to get used in print messages
#define CYAN_NRNT_DEBUG_NAME_S "Cyan NrNt"
#define CYAN_NRNT_DEBUG_NAME_C "CYAN_NRNT"

#define CYAN_NRNT_MB_PATH   fs_path("/mboards/0")
#define CYAN_NRNT_TIME_PATH fs_path(CYAN_NRNT_MB_PATH / "time")

// Constants for special flags
// Note: make sure this matches the value on the server
// The minimnum rate where the server cannont compensate for the NCO shift in the ADC
#define CYAN_NRNT_RX_NCO_SHIFT_3G_TO_1G_MIN_RATE 500000000
// THe NCO shift in the ADC
#define CYAN_NRNT_RX_NCO_SHIFT_3G_TO_1G 250000000

//The number corresponding to each band, eventually most of the stuff that relies on this should be moved to the server
enum {
    LOW_BAND = 0,
    MID_BAND = 1,
    HIGH_BAND = 2,
};

#endif /* INCLUDED_CYAN_NRNT_FW_COMMON_H */
