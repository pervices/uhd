#include <boost/test/unit_test.hpp>

#include "../lib/usrp/crimson_tng/iputils.hpp"

using namespace std;
using namespace uhd;

BOOST_AUTO_TEST_CASE( test_get_route ) {

	std::string local_addr;
	std::string remote_addr( "8.8.8.8" );

	std::string expected_iface = "";
	std::string actual_iface;

	uhd::iputils::get_route( remote_addr, actual_iface, local_addr );

	BOOST_CHECK_NE( expected_iface, actual_iface );

}

BOOST_AUTO_TEST_CASE( test_get_mtu ) {

	std::string local_addr;
	std::string remote_addr( "8.8.8.8" );

	std::string expected_iface = "";
	std::string actual_iface;

	size_t expected_mtu = 1472;
	size_t actual_mtu;

	uhd::iputils::get_route( remote_addr, actual_iface, local_addr );

	BOOST_CHECK_NE( expected_iface, actual_iface );

	actual_mtu = uhd::iputils::get_mtu( actual_iface );

	BOOST_CHECK_GE( actual_mtu, expected_mtu );
}
