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

#pragma once

#include <stdint.h>
#include <uhd/property_tree.hpp>

constexpr int CRIMSON_TNG_FW_COMPAT_MAJOR = 1;
constexpr int CRIMSON_TNG_FW_COMPAT_MINOR = 0;
constexpr int CRIMSON_TNG_FPGA_COMPAT_MAJOR = 1;

constexpr int CRIMSON_TNG_FW_NUM_BYTES = 1 << 15; //64k
constexpr int CRIMSON_TNG_FW_COMMS_MTU = 1 << 13; //8k

constexpr uint16_t CRIMSON_TNG_FW_COMMS_UDP_PORT = 42799;
constexpr uint16_t CRIMSON_TNG_FLOW_CNTRL_UDP_PORT = 42808;

inline const uint8_t CRIMSON_TNG_DEFAULT_MAC_ADDR_0[] = {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xff};
inline const uint8_t CRIMSON_TNG_DEFAULT_MAC_ADDR_1[] = {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x33};

constexpr uint32_t CRIMSON_TNG_DEFAULT_GATEWAY = (192 << 24 | 168 << 16 | 10 << 8 | 1 << 0);

constexpr uint32_t CRIMSON_TNG_DEFAULT_IP_ETH0_1G  = (192 << 24 | 168 << 16 | 10 << 8 | 2 << 0);
constexpr uint32_t CRIMSON_TNG_DEFAULT_IP_ETH0_10G = (192 << 24 | 168 << 16 | 30 << 8 | 2 << 0);
constexpr uint32_t CRIMSON_TNG_DEFAULT_IP_ETH1_10G = (192 << 24 | 168 << 16 | 40 << 8 | 2 << 0);

constexpr uint32_t CRIMSON_TNG_DEFAULT_NETMASK_ETH0_1G  = (255 << 24 | 255 << 16 | 255 << 8 | 0 << 0);
constexpr uint32_t CRIMSON_TNG_DEFAULT_NETMASK_ETH0_10G = (255 << 24 | 255 << 16 | 255 << 8 | 0 << 0);
constexpr uint32_t CRIMSON_TNG_DEFAULT_NETMASK_ETH1_10G = (255 << 24 | 255 << 16 | 255 << 8 | 0 << 0);

// Channel count to fall back to if the server has not been updated to provide them
constexpr int CRIMSON_TNG_FALLBACK_RX_CHANNELS = 4;
constexpr int CRIMSON_TNG_FALLBACK_TX_CHANNELS = 4;

// Numer of bits per half of the iq pair (i.e. sc16 mean 16 bits for the i half of the iq pair)
constexpr int CRIMSON_TNG_RX_SAMPLE_BITS = 16;
constexpr int CRIMSON_TNG_TX_SAMPLE_BITS = 16;

constexpr uint32_t CRIMSON_TNG_FW_COMMS_FLAGS_ACK    = (1 << 0);
constexpr uint32_t CRIMSON_TNG_FW_COMMS_FLAGS_ERROR  = (1 << 1);
constexpr uint32_t CRIMSON_TNG_FW_COMMS_FLAGS_POKE32 = (1 << 2);
constexpr uint32_t CRIMSON_TNG_FW_COMMS_FLAGS_PEEK32 = (1 << 3);

// Crimson min MTU size (typical ethernet frame)
constexpr int CRIMSON_TNG_MIN_MTU = 1500;
// Crimson max MTU size (jumbo ethernet frame is 9000 bytes)
constexpr int CRIMSON_TNG_MAX_MTU = 9000;

// Maximum number of bytes used to store samples in a tx packet
constexpr int CRIMSON_TNG_MAX_SEND_SAMPLE_BYTES = 8920;

// Amount of the UDP packet used for overhead in data packets
// 60 bit ipv4 header, 8 bit UDP header
constexpr int CRIMSON_TNG_UDP_OVERHEAD = 68;

// Size of the vrt header in bytes
constexpr int CRIMSON_TNG_HEADER_SIZE = 16;

// Size of the vrt trailer in bytes
constexpr int CRIMSON_TNG_TRAILER_SIZE = 4;

// Packaets send to the unit must have a multiple of this many samples
constexpr int CRIMSON_TNG_PACKET_NSAMP_MULTIPLE = 2;

// Crimson Flowcontrol Update Per Second
constexpr int CRIMSON_TNG_UPDATE_PER_SEC = 100;
constexpr int CRIMSON_TNG_SS_FIFOLVL_THRESHOLD = 107421875;

// Crimson Buffer Size
constexpr int CRIMSON_TNG_BUFF_SIZE = 65536;
//Target buffer level percentage
constexpr double CRIMSON_TNG_BUFF_PERCENT = 0.8;
// conversion factor between the number sent by the udp fifo checks and the number of samples in the buffer
constexpr int CRIMSON_TNG_BUFF_SCALE = 1;

// Crimson RF Settings
constexpr double CRIMSON_TNG_RF_TX_GAIN_RANGE_START = 0.0;
constexpr double CRIMSON_TNG_RF_TX_GAIN_RANGE_STOP  = 31.75;
constexpr double CRIMSON_TNG_RF_TX_GAIN_RANGE_STEP  = 0.25;

// Ranges from, 0dB to 83.25dB
constexpr double CRIMSON_TNG_RF_RX_GAIN_RANGE_START = 0.0;
constexpr double CRIMSON_TNG_RF_RX_GAIN_RANGE_STOP  = 83.25;
constexpr double CRIMSON_TNG_RF_RX_GAIN_RANGE_STEP  = 0.25;

// Crimson Clk Settings
// Old default max sample rate, fallback if the server hasn't been updated
constexpr double CRIMSON_TNG_FALLBACK_MASTER_CLOCK_RATE = 325000000;
constexpr double CRIMSON_TNG_EXT_CLK_RATE = 10000000; // only 10 MHz input sources allowed
// Crimson Tuning Range Settings
// Lower frequency Crimson can tune to
constexpr double CRIMSON_TNG_FREQ_RANGE_START = 0;
// Fallback max freq for older servers, the latest server will report this instead of UHD knowing
constexpr double CRIMSON_TNG_FALLBACK_FREQ_RANGE_STOP = 6000000000.0;
// Minimum tuning increment, using 0 since the DSP's increment is not constant (due to using a divisor)
constexpr double CRIMSON_TNG_FREQ_RANGE_STEP = 0;
//Crimson LO Tuning Range Step Size
constexpr double CRIMSON_TNG_LO_STEPSIZE_FALLBACK = 25000000;
constexpr double CRIMSON_TNG_LO_OFFSET = 25000000;
// Desired seperation between band of interest to the user and the lo
constexpr double CRIMSON_TNG_LO_TARGET_SEPERATION = 5000000;

// Crimson Sample Rate Settings
// These depend on the unit's maximum sample rate, which is only known at runtime.
inline double CRIMSON_TNG_RATE_RANGE_START(double max_rate)
{
    return max_rate / 65536;
}
inline double CRIMSON_TNG_RATE_RANGE_STOP_FULL(double max_rate)
{
    return max_rate;
}
inline double CRIMSON_TNG_RATE_RANGE_STOP_QUARTER(double max_rate)
{
    return max_rate / 4.0;
}
constexpr double CRIMSON_TNG_RATE_RANGE_STEP = 1.0;

// All ADCs and DACs take complex sample at 325MSPS,
// and so all share a static front end bandwidth of 325MHz
// However, for user throughput, DACs A/B have a max user complex samplerate of
// 162.5MSPS, and DACs C/D have 81.25MSPS due to backhaul bandwidth limitations
// and FPGA transciever clocking limitaions.
inline double CRIMSON_TNG_ADC_BW(double max_rate)
{
    return max_rate / 2.0;
}
inline double CRIMSON_TNG_ADC_FREQ_RANGE_ROLLOFF(double max_rate)
{
    return 0.8 * CRIMSON_TNG_ADC_BW(max_rate);
}
inline double CRIMSON_TNG_BW_FULL(double max_rate)
{
    return CRIMSON_TNG_RATE_RANGE_STOP_FULL(max_rate) / 2.0;
}
inline double CRIMSON_TNG_BW_QUARTER(double max_rate)
{
    return CRIMSON_TNG_RATE_RANGE_STOP_QUARTER(max_rate);
}
constexpr double CRIMSON_TNG_BW_RANGE_STEP = 1.0;
constexpr double CRIMSON_TNG_ADC_FREQ_RANGE_STEP = 1.0;

// There's a lower limit on the DC component we can pass. This is just an approximation.
constexpr double CRIMSON_TNG_DC_LOWERLIMIT = 3000000;
constexpr double CRIMSON_TNG_FM_LOWERLIMIT = 86900000;
constexpr double CRIMSON_TNG_FM_UPPERLIMIT = 107900000;

// Crimson DSP Freq Settings
// NCO mixing occurs after upconversion, limited by the FPGA/DAC bandwidth
constexpr double CRIMSON_TNG_DSP_BW_START = 0;
inline double CRIMSON_TNG_DSP_BW_STOP_FULL(double max_rate)
{
    return CRIMSON_TNG_BW_FULL(max_rate);
}
inline double CRIMSON_TNG_DSP_BW_STOP_QUARTER(double max_rate)
{
    return CRIMSON_TNG_BW_QUARTER(max_rate);
}
constexpr double CRIMSON_TNG_DSP_BW_STEPSIZE = 1.0;
inline double CRIMSON_TNG_DSP_FREQ_RANGE_START_FULL(double max_rate)
{
    return -CRIMSON_TNG_RATE_RANGE_STOP_FULL(max_rate) / 2.0;
}
inline double CRIMSON_TNG_DSP_FREQ_RANGE_STOP_FULL(double max_rate)
{
    return CRIMSON_TNG_RATE_RANGE_STOP_FULL(max_rate) / 2.0;
}
inline double CRIMSON_TNG_DSP_FREQ_RANGE_START_QUARTER(double max_rate)
{
    return -CRIMSON_TNG_RATE_RANGE_STOP_QUARTER(max_rate) / 2.0;
}
inline double CRIMSON_TNG_DSP_FREQ_RANGE_STOP_QUARTER(double max_rate)
{
    return CRIMSON_TNG_RATE_RANGE_STOP_QUARTER(max_rate) / 2.0;
}
constexpr double CRIMSON_TNG_DSP_FREQ_RANGE_STEP = 1.0;

// Crimson VITA settings
constexpr uint32_t CRIMSON_TNG_VITA_HDR_TYPE = 0x1;

constexpr uint32_t CRIMSON_TNG_VITA_TLR_EN     = 0xe00;
constexpr uint32_t CRIMSON_TNG_VITA_TLR_IND    = 0x0;
constexpr uint32_t CRIMSON_TNG_VITA_TLR_E      = 0x0;
constexpr uint32_t CRIMSON_TNG_VITA_TLR_PCKCNT = 0x0;
constexpr uint32_t CRIMSON_TNG_VITA_TLR =
    ((CRIMSON_TNG_VITA_TLR_EN << 20) | (CRIMSON_TNG_VITA_TLR_IND << 8) | (CRIMSON_TNG_VITA_TLR_E << 7)
        | (CRIMSON_TNG_VITA_TLR_PCKCNT << 0));

// fs_path is not a literal type, so it cannot be constexpr.
inline const uhd::fs_path CRIMSON_TNG_MB_PATH{"/mboards/0"};
inline const uhd::fs_path CRIMSON_TNG_TIME_PATH{CRIMSON_TNG_MB_PATH / "time"};

constexpr double CRIMSON_TNG_MIN_TX_DELAY = 0.001;