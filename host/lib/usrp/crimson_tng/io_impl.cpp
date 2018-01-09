//
// Copyright 2010-2012 Ettus Research LLC
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

#include <iomanip>
#include <iostream>
#include <boost/bind.hpp>

#include "uhd/stream.hpp"



#include "../../transport/super_recv_packet_handler.hpp"

#include "crimson_tng_impl.hpp"

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

/*
static managed_recv_buffer::sptr get_recv_buff_nop( double timeout ) {
	std::cout
		<< __FILE__ << ": " << __func__ << "(): " << __LINE__ << ":"
		<< std::endl;
	return
}
*/

static void issue_stream_command_nop( const uhd::stream_cmd_t &stream_cmd ) {
	std::cout
		<< __FILE__ << ": " << __func__ << "(): " << __LINE__ << ":"
		<< "would issue stream cmd { "
		<< "stream_mode: " << (char) stream_cmd.stream_mode << ", "
		<< "stream_now: " << stream_cmd.stream_now << ", "
		<< "time_spec: " << std::setprecision( 6 ) << stream_cmd.time_spec.get_real_secs() << " }"
		<< std::endl;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr crimson_tng_impl::get_rx_stream(const uhd::stream_args_t &args_){
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = 9000 - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp);

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_unpacker(&vrt::if_hdr_unpack_be);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.otw_format + "_item32_be";
    id.num_inputs = 1;
    id.output_format = args.cpu_format;
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    //bind callbacks for the handler
    for ( size_t i = 0; i < args.channels.size(); i++ ) {

//        my_streamer->set_xport_chan_get_buff(
//			i,
//			//boost::bind( & get_recv_buff_nop ),
//			boost::bind( &zero_copy_if::get_recv_buff ),
//        	true /* flush */
//		);
//

//    	my_streamer->set_issue_stream_cmd(
//        	i,
//			boost::bind(
//				& issue_stream_command_nop
//			)
//        );
    }

    //set the packet threshold to be an entire socket buffer's worth
//    const size_t packets_per_sock_buff = size_t(50e6/_mbc[_mbc.keys().front()].rx_dsp_xports[0]->get_recv_frame_size());
//    my_streamer->set_alignment_failure_threshold( packets_per_sock_buff );

    //sets all tick and samp rates on this streamer
//    this->update_rates();

    return my_streamer;
}
