#ifndef HOST_LIB_USRP_CRIMSON_TNG_IPUTILS_HPP_
#define HOST_LIB_USRP_CRIMSON_TNG_IPUTILS_HPP_

#include <cstdio>
#include <vector>
#include <string>
#include <sstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <uhd/exception.hpp>

extern "C" {

#include <sys/types.h>
#include <sys/ioctl.h>

#include <ifaddrs.h>
#include <net/if.h>

#include <netdb.h>
extern int h_errno;

}

namespace uhd {

class iputils {

public:
	static void get_route( const std::string remote_addr, std::string & iface ) {
		FILE *fp;

		char buf[ 256 ];
		size_t sz;
		std::vector<std::string> strs;
		hostent *he;

		//ip route show to match 10.10.10.2

		std::string cmd = ( boost::format( "ip route show to match %s" ) % remote_addr ).str();

		fp = popen( cmd.c_str() , "r" );
		if ( NULL == fp ) {
			throw runtime_error(
				( boost::format( "failed to run command '%s'" )
				  % cmd
				).str()
			);
		}

		memset( buf, '\0', sizeof( buf ) );
		sz = fread( buf, 1, sizeof( buf ), fp );
		pclose( fp );

		std::string bufstr( buf, sz );
		std::stringstream ss( bufstr );
		std::string line;
		std::string last_addr;
		std::string last_iface;

		for( ; getline( ss, line ); ) {
			if ( boost::starts_with( line, "default via " ) ) {
				line = line.substr( strlen( "default via " ) , line.length() - strlen( "default via " ) );
			}
			boost::split( strs, line, boost::is_any_of( "\t " ) );
			if ( strs.size() >= 3 ) {
				last_addr = strs[ 0 ];
				last_iface = strs[ 2 ];
			}

			ssize_t slash = last_addr.find( "/" );
			if ( -1 != slash ) {
				last_addr = last_addr.substr( 0, slash );
			}
		}

		he = gethostbyname( last_addr.c_str() );
		if ( NULL == he ) {
			throw runtime_error(
				( boost::format( "gethostbyname( '%s' ) failed" )
				  % strs[ 0 ]
				).str()
			);
		}

		iface = last_iface;
	}

	static size_t get_mtu( const std::string iface ) {
		int r;
		size_t mtu;
		int sockfd;
		ifreq req;

		snprintf( (char *)& req.ifr_ifrn, sizeof( req.ifr_ifrn ), "%s", iface.c_str() );
		r = socket( AF_INET, SOCK_DGRAM, 0 );
		if ( -1 == r ) {
			throw runtime_error(
				( boost::format( "socket(): %s ( %d )" )
				  % strerror( errno )
				  % errno
				).str()
			);
		}
		sockfd = r;

		r = ioctl( sockfd, SIOCGIFMTU, & req );
		if ( -1 == r ) {
			close( sockfd );
			throw runtime_error(
				( boost::format( "ioctl( SIOCGIFMTU, '%s' ): %s ( %d )" )
					% iface
					% strerror( errno )
					% errno
				).str()
			);
		}

		close( sockfd );

		mtu = req.ifr_ifru.ifru_mtu;

		return mtu;
	}
};

}

#endif /* HOST_LIB_USRP_CRIMSON_TNG_IPUTILS_HPP_ */
