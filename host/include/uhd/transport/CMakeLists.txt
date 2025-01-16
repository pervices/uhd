#
# Copyright 2010-2014 Ettus Research LLC
# Copyright 2018 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

UHD_INSTALL(FILES
    adapter_id.hpp
    async_recv_manager.hpp
    bounded_buffer.hpp
    bounded_buffer.ipp
    buffer_pool.hpp
    buffer_tracker.hpp
    frame_buff.hpp
    if_addrs.hpp
    udp_constants.hpp
    udp_simple.hpp
    udp_stream_zero_copy.hpp
    udp_zero_copy.hpp
    usb_control.hpp
    usb_zero_copy.hpp
    usb_device_handle.hpp
    user_recv_manager.hpp
    vrt_if_packet.hpp
    zero_copy.hpp
    DESTINATION ${INCLUDE_DIR}/uhd/transport
    COMPONENT headers
)

if(ENABLE_LIBURING)
    UHD_INSTALL(FILES
    io_uring_recv_manager.hpp
    DESTINATION ${INCLUDE_DIR}/uhd/transport
    COMPONENT headers
    )
endif()
