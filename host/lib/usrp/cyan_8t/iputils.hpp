#ifndef HOST_LIB_USRP_CYAN_8T_IPUTILS_HPP_
#define HOST_LIB_USRP_CYAN_8T_IPUTILS_HPP_

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <uhd/exception.hpp>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <ifaddrs.h>
#include <net/if.h>

#include <netdb.h>
#ifndef __APPLE__
extern int h_errno;
#endif

namespace uhd {

class iputils {

public:
	static void get_route( const std::string remote_addr, std::string & iface, std::string & local_addr ) {
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
			last_addr = strs[ 0 ];
			last_iface = strs[ 2 ];

			if ( strs.size() >= 3 ) {
				for( size_t i = 3; i < strs.size(); i++ ) {
					he = gethostbyname( strs[ i ].c_str() );
					if ( NULL == he ) {
						continue;
					}
					last_addr = strs[ i ];
				}
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
		local_addr = last_addr;
	}

	static size_t get_mtu( const std::string iface ) {
		int r;
		size_t mtu;
		int sockfd;
		ifreq req;

#ifdef __APPLE__
		mtu = 1400;
#else
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
#endif
		return mtu;
	}

	static void to_sockaddr( const std::string & host, sockaddr *addr, socklen_t & addr_len ) {

		int r;

		int fd;
		sockaddr_storage a;
		socklen_t alen;
		addrinfo *res, *rp;

		addrinfo hints;

		memset( & hints, 0, sizeof( hints ) );

		hints.ai_family = AF_INET,
		hints.ai_socktype = SOCK_DGRAM,

		r = getaddrinfo( host.c_str(), NULL, & hints, & res );
		if ( 0 != r ) {
			std::string es;
			int e;
			if ( EAI_SYSTEM == r ) {
				e = errno;
				es = std::string( strerror( e ) );
			} else {
				e = r;
				es = std::string( gai_strerror( e ) );
			}
			throw runtime_error(
				( boost::format( "getaddrinfo( '%s' ): %s ( %d )" )
					% host
					% es
					% e
				).str()
			);
		}
		for ( rp = res; rp != NULL; rp = rp->ai_next ) {
			// should really only need the first
			break;
		}

		if ( !( NULL == addr || addr_len < rp->ai_addrlen ) ) {
			memcpy( addr, rp->ai_addr, rp->ai_addrlen );
			addr_len = rp->ai_addrlen;
		}
		freeaddrinfo( res );

		if ( NULL == addr || addr_len < rp->ai_addrlen ) {
			throw value_error( "invalid arguments" );
		}
	}

	static int connect_udp(
		const sockaddr *local_addr, const socklen_t local_addr_len,
		const sockaddr *remote_addr, const socklen_t remote_addr_len
	) {

		int r;
		int fd;

		r = socket( AF_INET, SOCK_DGRAM, 0 );
		if ( -1 == r ) {
			throw runtime_error(
				( boost::format( "socket: %s ( %d )" )
					% strerror( errno )
					% errno
				).str()
			);
		}
		fd = r;

		r = connect( fd, remote_addr, remote_addr_len );
		if ( -1 == r ) {
			close( fd );
			fd = -1;
			throw runtime_error(
				( boost::format( "connect: %s ( %d )" )
					% strerror( errno )
					% errno
				).str()
			);
		}

		return fd;
	}

	static uint32_t get_if_mtu( const std::string & remote_addr ) {

		std::string iface;
		std::string local_addr;

		size_t mtu = 1500;

		try {
			get_route( remote_addr, iface, local_addr );
			mtu = get_mtu( iface );
		} catch( ... ) {
			std::cerr << "Unable to determine default route to " << remote_addr << " and, subsequently, interface mtu. Defaulting to " << mtu << std::endl;
		}

		return mtu;
	}
};

}

#endif /* HOST_LIB_USRP_CYAN_8T_IPUTILS_HPP_ */
