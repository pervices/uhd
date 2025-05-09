//
// Copyright 2014,2016 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_OCTOCLOCK_IMPL_HPP
#define INCLUDED_OCTOCLOCK_IMPL_HPP

#include "common.h"
#include <uhd/device.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp_clock/octoclock_eeprom.hpp>
#include <uhdlib/usrp/gps_ctrl.hpp>
#include <memory>
#include <mutex>

uhd::device_addrs_t octoclock_find(const uhd::device_addr_t& hint);

//! Create an empty octoclock packet with a random sequence number
octoclock_packet_t make_octoclock_packet();
//! Create an empty octoclock packet with a given sequence number
octoclock_packet_t make_octoclock_packet(const uint32_t sequence);

/*!
 * OctoClock implementation guts
 */
class octoclock_impl : public uhd::device
{
public:
    octoclock_impl(const uhd::device_addr_t&);
    ~octoclock_impl(void) override{};

    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t& args) override;

    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t& args) override;

    bool recv_async_msg(uhd::async_metadata_t&, double) override;

private:
    struct oc_container_type
    {
        uhd::usrp_clock::octoclock_eeprom_t eeprom;
        octoclock_state_t state;
        uhd::transport::udp_simple::sptr ctrl_xport;
        uhd::transport::udp_simple::sptr gpsdo_xport;
        uhd::gps_ctrl::sptr gps;
    };
    uhd::dict<std::string, oc_container_type> _oc_dict;
    uint32_t _sequence;
    uint32_t _proto_ver;

    void _set_eeprom(const std::string& oc, const uhd::usrp::mboard_eeprom_t& oc_eeprom);

    uint32_t _get_fw_version(const std::string& oc);

    void _get_state(const std::string& oc);

    uhd::sensor_value_t _ext_ref_detected(const std::string& oc);

    uhd::sensor_value_t _gps_detected(const std::string& oc);

    uhd::sensor_value_t _which_ref(const std::string& oc);

    uhd::sensor_value_t _switch_pos(const std::string& oc);

    uint32_t _get_time(const std::string& oc);

    std::string _get_images_help_message(const std::string& addr);

    std::mutex _device_mutex;
};

#endif /* INCLUDED_OCTOCLOCK_IMPL_HPP */
