//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>

#include <uhd/property_tree.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/utils/noncopyable.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/types/tune_result.hpp>
#include <functional>
#include <memory>

namespace uhd {

class property_tree; // forward declaration

/*!
 * The device interface represents the hardware.
 * The API allows for discovery, configuration, and streaming.
 */
class UHD_API device : uhd::noncopyable
{
public:
    typedef std::shared_ptr<device> sptr;
    typedef std::function<device_addrs_t(const device_addr_t&)> find_t;
    typedef std::function<sptr(const device_addr_t&)> make_t;

    //! Device type, used as a filter in make
    enum device_filter_t {
        ANY,
        USRP,
        CLOCK,
        CRIMSON_TNG,
        CYAN_8T,
        CYAN_4R4T,
        CYAN_9R7T,
        CYAN_4R4T_3G,
        CYAN_8R,
        CYAN_16T,
        CYAN_P1HDR16T,
        CYAN_P1HDR32T,
        CYAN_64T,
    };



    virtual ~device(void) = 0;

    /*!
     * Register a device into the discovery and factory system.
     *
     * \param find a function that discovers devices
     * \param make a factory function that makes a device
     * \param filter include only USRP devices, clock devices, or both
     */
    static void register_device(
        const find_t& find, const make_t& make, const device_filter_t filter);

    /*!
     * \brief Find devices attached to the host.
     *
     * The hint device address should be used to narrow down the search
     * to particular transport types and/or transport arguments.
     *
     * \param hint a partially (or fully) filled in device address
     * \param filter an optional filter to exclude USRP or clock devices
     * \return a vector of device addresses for all devices on the system
     */
    static device_addrs_t find(const device_addr_t& hint, device_filter_t filter = ANY);

    /*!
     * \brief Create a new device from the device address hint.
     *
     * The method will go through the registered device types and pick one of
     * the discovered devices.
     *
     * By default, the first result will be used to create a new device.
     * Use the which parameter as an index into the list of results.
     *
     * \param hint a partially (or fully) filled in device address
     * \param filter an optional filter to exclude USRP or clock devices
     * \param which which address to use when multiple are found
     * \return a shared pointer to a new device instance
     */
    static sptr make(
        const device_addr_t& hint, device_filter_t filter = ANY, size_t which = 0);

    /*! \brief Make a new receive streamer from the streamer arguments
     *
     * Note: There can always only be one streamer. When calling get_rx_stream()
     * a second time, the first streamer must be destroyed beforehand.
     */
    virtual rx_streamer::sptr get_rx_stream(const stream_args_t& args) = 0;

    /*! \brief Make a new transmit streamer from the streamer arguments
     *
     * Note: There can always only be one streamer. When calling get_tx_stream()
     * a second time, the first streamer must be destroyed beforehand.
     */
    virtual tx_streamer::sptr get_tx_stream(const stream_args_t& args) = 0;

    /*! DEPRECATED: Receive asynchronous message from the device
     *
     * Prefer calling recv_async_msg on the associated TX streamer. This method
     * has the problem that it doesn't necessarily know which Tx streamer is
     * being addressed, and thus might not be delivering the expected outcome.
     *
     * \param async_metadata the metadata to be filled in
     * \param timeout the timeout in seconds to wait for a message
     * \return true when the async_metadata is valid, false for timeout
     */

    virtual bool recv_async_msg(
        async_metadata_t& async_metadata, double timeout = 0.1) = 0;

    //! Get access to the underlying property structure
    uhd::property_tree::sptr get_tree(void) const;

    //! Get device type
    device_filter_t get_device_type() const;

    /*!
     * Set the RX center frequency.
     * \param tune_request tune request instructions
     * \param chan the channel index 0 to N-1
     * \return a tune result object
     */
    virtual uhd::tune_result_t set_rx_freq(
        const uhd::tune_request_t &tune_request, size_t chan = 0
    ) {
        (void) tune_request;
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Get the RX center frequency.
     * \param chan the channel index 0 to N-1
     * \return the frequency in Hz
     */
    virtual double get_rx_freq(size_t chan = 0) {
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Set the RX center frequency.
     * \param tune_request tune request instructions
     * \param chan the channel index 0 to N-1
     * \return a tune result object
     */

    virtual void tx_trigger_setup(
        std::vector<size_t> channels,
        uint64_t num_samples_per_trigger
    ) {
        (void) channels;
        (void) num_samples_per_trigger;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Configures the unit for trigger streaming. Note that is will cause the selected channels to stop using buffer level prediction
     * \param channels list of channels
     * \param buffer_setpoint target buffer level
     * \param num_samples_per_trigger number of samples to send per trigger event
     */

    virtual void tx_trigger_cleanup(
        std::vector<size_t> channels
    ) {
        (void) channels;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    virtual void rx_trigger_setup(
        std::vector<size_t> channels,
        uint64_t num_samples_per_trigger
    ) {
        (void) channels;
        (void) num_samples_per_trigger;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Configures the unit to stream when the trigger is activated
     * \param channels list of channels
     * \param num_samples_per_trigger number of samples to send per trigger event
     */

    virtual void rx_trigger_cleanup(
        std::vector<size_t> channels
    ) {
        (void) channels;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Cleans up changes from normal mode made by rx_trigger_setup
     * \param channels list of channels
     */
    
    virtual std::string get_tx_sfp(size_t chan ) {
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }
    /*! Gets the sfp port to use when interacting with that channel
     * \param chan the channel index 0 to N-1
     */
    
    virtual std::string get_tx_ip(size_t chan ) {
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }
    /*! Gets the IP adress to send the main tx datastream to
     * \param chan the channel index 0 to N-1
     */
    
    virtual uint16_t get_tx_fc_port(size_t chan ) {
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }
    /*! Gets the port used for flow control (getting fifo lvl and clock synchronization
     * \param chan the channel index 0 to N-1
     */

    virtual uint16_t get_tx_udp_port(size_t chan ) {
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }
    /*! Gets the udp port to send the main tx datastream to
     * \param chan the channel index 0 to N-1
     */
    virtual uhd::tune_result_t set_tx_freq(
        const uhd::tune_request_t &tune_request, size_t chan = 0
    ) {
        (void) tune_request;
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    /*!
     * Get the TX center frequency.
     * \param chan the channel index 0 to N-1
     * \return the frequency in Hz
     */
    virtual double get_tx_freq(size_t chan = 0) {
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    virtual void set_tx_gain(double gain, const std::string &name, size_t chan){
        (void) gain;
        (void) name;
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");

    }

    virtual double get_tx_gain(const std::string &name, size_t chan){
        (void) name;
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");

    }
    
    /*!
     * Get the number of samples in the buffer per number reported in packets sent to fif udp port
     */
    virtual int64_t get_tx_buff_scale(){
        throw std::runtime_error("concrete classes are expected to override this method");
    }

    virtual void set_rx_gain(double gain, const std::string &name, size_t chan){
        (void) gain;
        (void) name;
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");

    }
    
    virtual double get_rx_gain(const std::string &name, size_t chan){
        (void) name;
        (void) chan;
        throw std::runtime_error("concrete classes are expected to override this method");

    }

protected:
    uhd::property_tree::sptr _tree;
    device_filter_t _type;
};

} // namespace uhd
