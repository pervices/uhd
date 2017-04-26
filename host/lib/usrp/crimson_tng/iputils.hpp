#ifndef HOST_LIB_USRP_CRIMSON_TNG_IPUTILS_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_IPUTILS_HPP_

extern "C" {

#include <sys/types.h>
#include <sys/ioctl.h>

#include <ifaddrs.h>
#include <net/if.h>

}

#include <string>

namespace uhd {


void get_route( const std::string remote_addr, std::string & local_addr );
void get_iface( const std::string remote_addr, std::string & iface, std::string & local_addr );
size_t get_mtu( const std::string iface, const std::string local_addr );

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_IPUTILS_HPP_ */
