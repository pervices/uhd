//
// Copyright 2013-2016 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_X300_IMPL_HPP
#define INCLUDED_X300_IMPL_HPP


#include "x300_clock_ctrl.hpp"
#include "x300_conn_mgr.hpp"

#include "x300_defaults.hpp"
#include "x300_device_args.hpp"
#include "x300_fw_common.h"
#include "x300_mb_controller.hpp"
#include "x300_mboard_type.hpp"
#include "x300_regs.hpp"
#include <uhd/property_tree.hpp>
#include <uhd/rfnoc/chdr_types.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/wb_iface.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhdlib/rfnoc/clock_iface.hpp>
#include <uhdlib/rfnoc/mb_iface.hpp>
#include <uhdlib/rfnoc/mgmt_portal.hpp>
#include <uhdlib/rfnoc/rfnoc_common.hpp>
#include <uhdlib/rfnoc/rfnoc_device.hpp>
#include <uhdlib/transport/links.hpp>
#include <uhdlib/usrp/cores/i2c_core_100_wb32.hpp>
#include <uhdlib/usrp/cores/spi_core_3000.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

static const std::string X300_FW_FILE_NAME  = "usrp_x300_fw.bin";
static const std::string X300_DEFAULT_CLOCK_SOURCE  = "internal";

static const double X300_DEFAULT_TICK_RATE          = 200e6;   //Hz
static const double X300_BUS_CLOCK_RATE             = 187.5e6; //Hz

static const size_t X300_RX_SW_BUFF_SIZE_ETH        = 0x2000000;//32MiB    For an ~8k frame size any size >32MiB is just wasted buffer space
static const size_t X300_RX_SW_BUFF_SIZE_ETH_MACOS  = 0x100000; //1Mib

//The FIFO closest to the DMA controller is 1023 elements deep for RX and 1029 elements deep for TX
//where an element is 8 bytes. The buffers (number of frames * frame size) must be aligned to the
//memory page size.  For the control, we are getting lucky because 64 frames * 256 bytes each aligns
//with the typical page size of 4096 bytes.  Since most page sizes are 4096 bytes or some multiple of
//that, keep the number of frames * frame size aligned to it.
static const size_t X300_PCIE_RX_DATA_FRAME_SIZE        = 4096;     //bytes
static const size_t X300_PCIE_RX_DATA_NUM_FRAMES        = 4096;
static const size_t X300_PCIE_TX_DATA_FRAME_SIZE        = 4096;     //bytes
static const size_t X300_PCIE_TX_DATA_NUM_FRAMES	    = 4096;
static const size_t X300_PCIE_MSG_FRAME_SIZE            = 256;      //bytes
static const size_t X300_PCIE_MSG_NUM_FRAMES            = 64;
static const size_t X300_PCIE_MAX_CHANNELS              = 6;
static const size_t X300_PCIE_MAX_MUXED_CTRL_XPORTS     = 32;
static const size_t X300_PCIE_MAX_MUXED_ASYNC_XPORTS    = 4;

static const size_t X300_10GE_DATA_FRAME_MAX_SIZE   = 8000;     // CHDR packet size in bytes
static const size_t X300_1GE_DATA_FRAME_MAX_SIZE    = 1472;     // CHDR packet size in bytes
static const size_t X300_ETH_MSG_FRAME_SIZE         = uhd::transport::udp_simple::mtu;  //bytes
// MTU throttling for ethernet/TX (see above):
static const size_t X300_ETH_DATA_FRAME_MAX_TX_SIZE = 8000;

static const double X300_THREAD_BUFFER_TIMEOUT      = 0.1;   // Time in seconds

static const size_t X300_ETH_MSG_NUM_FRAMES         = 64;
static const size_t X300_ETH_DATA_NUM_FRAMES        = 32;
static const double X300_DEFAULT_SYSREF_RATE        = 10e6;

// Limit the number of initialization threads
static const size_t X300_MAX_INIT_THREADS           = 10;

static const size_t X300_MAX_RATE_PCIE              = 800000000; // bytes/s
static const size_t X300_MAX_RATE_10GIGE            = (size_t)(  // bytes/s
        10e9 / 8 *                                               // wire speed multiplied by percentage of packets that is sample data
        ( float(X300_10GE_DATA_FRAME_MAX_SIZE - uhd::usrp::DEVICE3_TX_MAX_HDR_LEN) /
          float(X300_10GE_DATA_FRAME_MAX_SIZE + 8 /* UDP header */ + 20 /* Ethernet header length */ )));
static const size_t X300_MAX_RATE_1GIGE            = (size_t)(  // bytes/s
        1e9 / 8 *                                               // wire speed multiplied by percentage of packets that is sample data
        ( float(X300_1GE_DATA_FRAME_MAX_SIZE - uhd::usrp::DEVICE3_TX_MAX_HDR_LEN) /
          float(X300_1GE_DATA_FRAME_MAX_SIZE + 8 /* UDP header */ + 20 /* Ethernet header length */ )));

#define X300_RADIO_DEST_PREFIX_TX 0

#define X300_XB_DST_E0  0
#define X300_XB_DST_E1  1
#define X300_XB_DST_PCI 2
#define X300_XB_DST_R0  3 // Radio 0 -> Slot A
#define X300_XB_DST_R1  4 // Radio 1 -> Slot B
#define X300_XB_DST_CE0 5

#define X300_SRC_ADDR0  0
#define X300_SRC_ADDR1  1
#define X300_DST_ADDR   2



uhd::device_addrs_t x300_find(const uhd::device_addr_t& hint_);

class x300_impl : public uhd::rfnoc::detail::rfnoc_device
{
public:

    x300_impl(const uhd::device_addr_t&);
    void setup_mb(const size_t which, const uhd::device_addr_t&);
    ~x300_impl(void) override;


    /**************************************************************************
     * rfnoc_device API
     *************************************************************************/
    uhd::rfnoc::mb_iface& get_mb_iface(const size_t mb_idx) override
    {
        if (mb_idx >= _mb_ifaces.size()) {
            throw uhd::index_error(
                std::string("Cannot get mb_iface, invalid motherboard index: ")
                + std::to_string(mb_idx));
        }
        return _mb_ifaces.at(mb_idx);
    }

private:
    /**************************************************************************
     * Types
     *************************************************************************/
    // vector of member objects per motherboard
    struct mboard_members_t
    {
        uhd::usrp::x300::x300_device_args_t args;

        //! Remote Device ID for this motherboard
        uhd::rfnoc::device_id_t device_id;


        bool initialization_done = false;
        uhd::task::sptr claimer_task;
        uhd::usrp::x300::xport_path_t xport_path;
        uhd::device_addr_t send_args;
        uhd::device_addr_t recv_args;


        // perifs in the zpu
        uhd::wb_iface::sptr zpu_ctrl;
        spi_core_3000::sptr zpu_spi;
        i2c_core_100_wb32::sptr zpu_i2c;

        // other perifs on mboard
        x300_clock_ctrl::sptr clock;


        // which FPGA image is loaded
        std::string loaded_fpga_image;

        size_t hw_rev;


        uhd::usrp::x300::conn_manager::sptr conn_mgr;

    };


    //! X300-Specific Implementation of rfnoc::mb_iface
    class x300_mb_iface : public uhd::rfnoc::mb_iface
    {
    public:
        x300_mb_iface(uhd::usrp::x300::conn_manager::sptr conn_mgr,
            const double radio_clk_freq,
            const uhd::rfnoc::device_id_t remote_dev_id);
        ~x300_mb_iface() override;
        uint16_t get_proto_ver() override;
        uhd::rfnoc::chdr_w_t get_chdr_w() override;
        uhd::endianness_t get_endianness(
            const uhd::rfnoc::device_id_t local_device_id) override;
        uhd::rfnoc::device_id_t get_remote_device_id() override;
        std::vector<uhd::rfnoc::device_id_t> get_local_device_ids() override;
        uhd::transport::adapter_id_t get_adapter_id(
            const uhd::rfnoc::device_id_t local_device_id) override;
        void reset_network() override;
        uhd::rfnoc::clock_iface::sptr get_clock_iface(
            const std::string& clock_name) override;
        uhd::rfnoc::chdr_ctrl_xport::sptr make_ctrl_transport(
            uhd::rfnoc::device_id_t local_device_id,
            const uhd::rfnoc::sep_id_t& local_epid) override;
        uhd::rfnoc::chdr_rx_data_xport::uptr make_rx_data_transport(
            uhd::rfnoc::mgmt::mgmt_portal& mgmt_portal,
            const uhd::rfnoc::sep_addr_pair_t& addrs,
            const uhd::rfnoc::sep_id_pair_t& epids,
            const uhd::rfnoc::sw_buff_t pyld_buff_fmt,
            const uhd::rfnoc::sw_buff_t mdata_buff_fmt,
            const uhd::device_addr_t& xport_args,
            const std::string& streamer_id) override;
        uhd::rfnoc::chdr_tx_data_xport::uptr make_tx_data_transport(
            uhd::rfnoc::mgmt::mgmt_portal& mgmt_portal,
            const uhd::rfnoc::sep_addr_pair_t& addrs,
            const uhd::rfnoc::sep_id_pair_t& epids,
            const uhd::rfnoc::sw_buff_t pyld_buff_fmt,
            const uhd::rfnoc::sw_buff_t mdata_buff_fmt,
            const uhd::device_addr_t& xport_args,
            const std::string& streamer_id) override;


    private:
        const uhd::rfnoc::device_id_t _remote_dev_id;
        std::unordered_map<uhd::rfnoc::device_id_t, uhd::transport::adapter_id_t>
            _adapter_map;
        uhd::rfnoc::clock_iface::sptr _bus_clk;
        uhd::rfnoc::clock_iface::sptr _radio_clk;
        uhd::usrp::x300::conn_manager::sptr _conn_mgr;
    };

    /**************************************************************************
     * Private Methods
     *************************************************************************/
    void check_fw_compat(const uhd::fs_path& mb_path, const mboard_members_t& members);
    void check_fpga_compat(const uhd::fs_path& mb_path, const mboard_members_t& members);

    /**************************************************************************
     * Private Attributes
     *************************************************************************/
    std::vector<mboard_members_t> _mb;

    std::mutex _mb_iface_mutex;
    std::unordered_map<size_t, x300_mb_iface> _mb_ifaces;

    static const uhd::rfnoc::chdr::chdr_packet_factory _pkt_factory;
};

#endif /* INCLUDED_X300_IMPL_HPP */
