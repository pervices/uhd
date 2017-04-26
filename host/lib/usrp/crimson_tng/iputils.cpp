#include <cstdio>
#include <vector>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <uhd/exception.hpp>

#include "iputils.hpp"

extern "C" {

#include <netdb.h>
extern int h_errno;

}

using namespace std;
using namespace boost;
using namespace uhd;

void uhd::get_route( const std::string remote_addr, std::string & local_addr ) {
	FILE *fp;

	char buf[ 64 ];
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
	boost::split( strs, bufstr, boost::is_any_of( "\t " ) );

	if ( strs.size() < 1 ) {
		throw runtime_error(
			( boost::format( "could not get IP address from '%s'" )
			  % buf
			).str()
		);
	}

	he = gethostbyname( strs[ 0 ].c_str() );
	if ( NULL == he ) {
		throw runtime_error(
			( boost::format( "gethostbyname( '%s' ) failed" )
			  % strs[ 0 ]
			).str()
		);
	}

	local_addr = strs[ 0 ];
}

void uhd::get_iface( const std::string remote_addr, std::string & iface, std::string & local_addr ) {
}

size_t uhd::get_mtu( const std::string iface, const std::string local_addr ) {
	return 0;
}
