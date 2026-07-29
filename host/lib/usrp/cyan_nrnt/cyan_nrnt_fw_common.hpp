//
// Copyright 2014 Per Vices Corporation
// Copyright 2022-2023 Per Vices Corporation
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
#include <string>
#include <uhd/property_tree.hpp>

constexpr int CYAN_NRNT_FW_COMMS_MTU = 1 << 13; //8k

constexpr uint16_t CYAN_NRNT_FW_COMMS_UDP_PORT = 42799;

// Cyan NrNt max MTU size (jumbo ethernet frame is 9000 bytes)
constexpr int CYAN_NRNT_MAX_MTU = 8992;

// Maximum number of bytes used to store samples in a tx packet
constexpr int CYAN_NRNT_MAX_SEND_SAMPLE_BYTES = 8912;

// Target number of sample bytes per packets for recv
// WIP: must follow certain constraints, likely to be either a multiple of page size of multiple of 512
constexpr int CYAN_NRNT_TARGET_RECV_SAMPLE_BYTES = 8192;

// Maximum number of bytes used to store sample data when receiving when unable to get it from the server
constexpr int CYAN_NRNT_FALLBACK_MAX_NBYTES = 8880;

// Size of the vrt header in bytes
constexpr int CYAN_NRNT_HEADER_SIZE = 16;

// Size of the vrt trailer in bytes
constexpr int CYAN_NRNT_TRAILER_SIZE = 0;

// Packets send to the unit must have a multiple of this many samples
// A multiple of 12 to avoid loosing up to 7 samples when sending 2 consecutive packets with less than 12 samples in the FPGA
// A multiple of 4 is required even if the above is fixed since there are up to 4 samples per clock
constexpr int CYAN_NRNT_PACKET_NSAMP_MULTIPLE = 12;
// Amount of the UDP packet used for overhead in data packets
// 60 bit ipv4 header, 8 bit UDP header
constexpr int CYAN_NRNT_UDP_OVERHEAD = 68;

// Cyan NrNt Flowcontrol Update Per Second
constexpr int CYAN_NRNT_UPDATE_PER_SEC = 100;

//how full UHD should try to keep the buffer
constexpr double CYAN_NRNT_BUFF_PERCENT = 0.7;

// Cyan NrNt tx RF gain limits, this is definitely wrong
// TODO: get this info from device
constexpr double CYAN_NRNT_RF_TX_GAIN_RANGE_START = 0.0;
constexpr double CYAN_NRNT_RF_TX_GAIN_RANGE_STOP  = 31.75;
constexpr double CYAN_NRNT_RF_TX_GAIN_RANGE_STEP  = 0.25;

// Cyan NrNt rx RF gain limits, this is definitely wrong
//TODO:get this info from device
constexpr double CYAN_NRNT_RF_RX_GAIN_RANGE_START = -6;
constexpr double CYAN_NRNT_RF_RX_GAIN_RANGE_STOP  = 33;
constexpr double CYAN_NRNT_RF_RX_GAIN_RANGE_STEP  = 1;

// Cyan NrNt Clk Settings
// number of clock cycles per second
constexpr double CYAN_NRNT_TICK_RATE = 250000000;
constexpr double CYAN_NRNT_EXT_CLK_RATE = 10000000; // only 10 MHz input sources allowed
// Cyan NrNt Tuning Range Settings
constexpr double CYAN_NRNT_FREQ_RANGE_START = 0;
constexpr double CYAN_NRNT_FREQ_RANGE_STOP  = 20000000000.0;
// The step is wrong, the nco will often end ep with decimal results
constexpr double CYAN_NRNT_FREQ_RANGE_STEP = 1.0;

// Desired seperation between band of interest to the user and the lo
constexpr double CYAN_NRNT_LO_TARGET_SEPERATION = 5000000;

// Cyan NrNt Sample Rate Settings
// These depend on the unit's maximum sample rate, which is only known at runtime.
inline double CYAN_NRNT_RATE_RANGE_START(double max_sample_rate)
{
    return max_sample_rate / 65536;
}
inline double CYAN_NRNT_RATE_RANGE_STOP_FULL(double max_sample_rate)
{
    return max_sample_rate;
}
inline double CYAN_NRNT_RATE_RANGE_STOP_QUARTER(double max_sample_rate)
{
    return max_sample_rate / 4.0;
}
inline double CYAN_NRNT_RATE_RANGE_STEP(double max_sample_rate)
{
    return max_sample_rate / 65536 - (max_sample_rate / 65536);
}

// Cyan NrNt DSP Freq Settings
// NCO mixing occurs after upconversion, limited by the FPGA/DAC bandwidth
inline double CYAN_NRNT_BW_FULL(double max_sample_rate)
{
    return CYAN_NRNT_RATE_RANGE_STOP_FULL(max_sample_rate) / 2.0;
}
constexpr double CYAN_NRNT_DSP_BW_START = 0;
inline double CYAN_NRNT_DSP_BW_STOP_FULL(double max_sample_rate)
{
    return CYAN_NRNT_BW_FULL(max_sample_rate);
}
constexpr double CYAN_NRNT_DSP_BW_STEPSIZE = 1.0;
inline double CYAN_NRNT_DSP_FREQ_RANGE_START_FULL(double max_sample_rate)
{
    return -CYAN_NRNT_RATE_RANGE_STOP_FULL(max_sample_rate) / 2.0;
}
inline double CYAN_NRNT_DSP_FREQ_RANGE_STOP_FULL(double max_sample_rate)
{
    return CYAN_NRNT_RATE_RANGE_STOP_FULL(max_sample_rate) / 2.0;
}
// The step is wrong, the nco will often end ep with decimal results
constexpr double CYAN_NRNT_DSP_FREQ_RANGE_STEP = 1.0;

//Cyan NrNt LO Tuning Range Step Size
constexpr double CYAN_NRNT_LO_STEPSIZE = 100000000;
//sample rate will between 0 and CYAN_NRNT_LO_DIFF_RANGE_START[0] will result in the lo being a close to CYAN_NRNT_LO_DIFF[0] away from target frequency, while not excedding CYAN_NRNT_LO_DIFF[0]
//ranges go from previous number (inclusive to the maximum of the range -1)
constexpr double CYAN_NRNT_MIN_LO = CYAN_NRNT_LO_STEPSIZE;
inline double CYAN_NRNT_MAX_LO(double freq_range_stop)
{
    return freq_range_stop;
}

//Cyan NrNt VITA settings

constexpr uint32_t CYAN_NRNT_VITA_TLR_EN     = 0xe00;
constexpr uint32_t CYAN_NRNT_VITA_TLR_IND    = 0x0;
constexpr uint32_t CYAN_NRNT_VITA_TLR_E      = 0x0;
constexpr uint32_t CYAN_NRNT_VITA_TLR_PCKCNT = 0x0;
constexpr uint32_t CYAN_NRNT_VITA_TLR =
    ((CYAN_NRNT_VITA_TLR_EN << 20) | (CYAN_NRNT_VITA_TLR_IND << 8) | (CYAN_NRNT_VITA_TLR_E << 7)
        | (CYAN_NRNT_VITA_TLR_PCKCNT << 0));

// Constants to determine which frequency band to use
constexpr double CYAN_NRNT_LOW_MID_BARRIER_RX = 250000000;
constexpr double CYAN_NRNT_LOW_MID_BARRIER_TX = 400000000;
constexpr double CYAN_NRNT_MID_HIGH_BARRIER   = 5800000000;
constexpr double CYAN_NRNT_MID_MAX_LO         = 5600000000;

//the device name to get used in print messages
// std::string is not usable as a namespace-scope constexpr in a portable way, so these
// remain plain (non-constexpr) constants.
inline const std::string CYAN_NRNT_DEBUG_NAME_S = "Cyan NrNt";
inline const std::string CYAN_NRNT_DEBUG_NAME_C = "CYAN_NRNT";

// fs_path is not a literal type (its constructors are defined out-of-line and it derives from
// std::string), so it cannot be a constexpr. These remain plain (non-constexpr) constants, kept
// as objects (rather than functions) so all existing "CYAN_NRNT_MB_PATH / ..." usages still work.
inline const uhd::fs_path CYAN_NRNT_MB_PATH{"/mboards/0"};
inline const uhd::fs_path CYAN_NRNT_TIME_PATH{CYAN_NRNT_MB_PATH / "time"};

// max_sample_rate / CYAN_NRNT_MAX_DSP_RATE_FACTOR = maximum rate the NCO operates at
constexpr double CYAN_NRNT_MAX_DSP_RATE_FACTOR = 4.0;

// Constants for special flags
// There is a fixed NCO shift (250MHz) in the ADC in this mode. It is mostly compensated for by another NCO in the ADC (249.98MHz)
constexpr double CYAN_NRNT_RX_NCO_SHIFT_3G_TO_1G = (250000000 - 249984741.211);

//The number corresponding to each band, eventually most of the stuff that relies on this should be moved to the server
enum {
    LOW_BAND = 0,
    MID_BAND = 1,
    HIGH_BAND = 2,
};

constexpr double CYAN_NRNT_MIN_TX_DELAY = 0.001;