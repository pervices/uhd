//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Per Vices Corporation
// Copyright 2022-2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/usrp/pv_device/pv_device_recv_packet_streamer.hpp>

#include <uhd/utils/log.hpp>

#include <cerrno>
#include <cstring>
#include <sys/file.h>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

pv_device_recv_packet_streamer::pv_device_recv_packet_streamer(
    const std::string product_name_c,
    const std::vector<size_t> channels,
    const std::vector<int>& recv_sockets,
    const std::vector<std::string>& dst_ip,
    const size_t max_sample_bytes_per_packet,
    const size_t header_size,
    const size_t trailer_size,
    const std::string& cpu_format,
    const std::string& wire_format,
    bool wire_little_endian,
    std::shared_ptr<std::vector<bool>> rx_channel_in_use,
    size_t device_total_rx_channels,
    pv_iface::sptr iface,
    std::vector<uhd::usrp::stream_cmd_issuer> cmd_issuer,
    std::vector<int> channel_locks,
    std::vector<int> streaming_locks
)
: sph::recv_packet_streamer_mmsg(channels, recv_sockets, dst_ip, max_sample_bytes_per_packet, header_size, trailer_size, cpu_format, wire_format, wire_little_endian, device_total_rx_channels, cmd_issuer, streaming_locks),
_product_name_c(product_name_c),
_channels(channels),
_rx_streamer_channel_in_use(rx_channel_in_use),
_channel_locks(channel_locks),
_streaming_locks(streaming_locks),
_iface(iface)
{
    // Attempt to lock each channel used by this streamer. This channel will remain locked for the lifetime of this streamer.
    // This is just advisory so an error will be printed if it was locked or could not get the lock but the program will continue.
    for (size_t n = 0; n < channels.size(); n++) {
        int lock_fd = _channel_locks[channels[n]];
        int r = flock(lock_fd, LOCK_EX | LOCK_NB);
        if (r == -1) {
            int err = errno;
            // Since flock was run with the nonblocking flag, errno will be EWOULDBLOCK if this channel was already locked
            if (err == EWOULDBLOCK) {
                std::string err_msg = "Another UHD streamer has already been created using channel " + std::to_string(_channels[n]) + " which may cause unexpected behaviour.";
                UHD_LOG_ERROR(_product_name_c, err_msg);
            } else {
                UHD_LOG_ERROR(_product_name_c, "Failed to place lock on channel " + std::to_string(_channels[n]) + " lockfile.\nflock failed with error: " + std::string(strerror(err)));
            }
        }
    }

    _rx_streamer_channel_in_use = rx_channel_in_use;
    for(size_t n = 0; n < channels.size(); n++) {
        _rx_streamer_channel_in_use->at(channels[n]) = true;
    }
}

pv_device_recv_packet_streamer::~pv_device_recv_packet_streamer() {
    for(size_t n = 0; n < _NUM_CHANNELS; n++) {
        // Deactivates the channel. Mutes rf, puts the dsp in reset, and turns off the outward facing LED on the board
        // Does not actually turn off board
        _iface->set_string("rx/" + std::string(1, (char) (_channels[n] + 'a')) + "/stream", "0");
        _iface->set_string("rx/" + std::string(1, (char) (_channels[n] + 'a')) + "/pwr", "0");

        // Marks this channel as not in use for the purposes of the check if the SFP can handle the combined rates on it
        _rx_streamer_channel_in_use->at(_channels[n]) = false;

        // Release channel locks
        flock(_channel_locks[_channels[n]], LOCK_UN);
    }
}

void pv_device_recv_packet_streamer::if_hdr_unpack(const uint32_t* packet_buff, vrt::if_packet_info_t& if_packet_info) {
    vrt::if_hdr_unpack_be(packet_buff, if_packet_info);
}
