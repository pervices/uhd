#include <boost/test/unit_test.hpp>

#include "../lib/usrp/crimson_tng/iputils.hpp"

using namespace std;
using namespace uhd;

BOOST_AUTO_TEST_CASE( test_get_route_info ) {

	std::string local_addr;
	std::string remote_addr( "8.8.8.8" );

	std::string expected_iface;
	std::string actual_iface;

	std::string expected_route_info;
	std::string actual_route_info;

	actual_route_info = iputils::get_route_info( remote_addr );
	BOOST_CHECK_NE( expected_route_info, actual_route_info );

	iputils::get_route( actual_route_info, actual_iface, local_addr );
	BOOST_CHECK_NE( expected_iface, actual_iface );
}

BOOST_AUTO_TEST_CASE( test_get_route ) {

	std::string local_addr;
	std::string remote_addr( "8.8.8.8" );

	std::string expected_iface = "";
	std::string actual_iface;

	uhd::iputils::get_route( iputils::get_route_info( remote_addr ), actual_iface, local_addr );

	BOOST_CHECK_NE( expected_iface, actual_iface );

}

BOOST_AUTO_TEST_CASE( test_get_mtu ) {

	std::string local_addr;
	std::string remote_addr( "8.8.8.8" );

	std::string expected_iface = "";
	std::string actual_iface;

	size_t expected_mtu = 1472;
	size_t actual_mtu;

	uhd::iputils::get_route( iputils::get_route_info( remote_addr ), actual_iface, local_addr );

	BOOST_CHECK_NE( expected_iface, actual_iface );

	actual_mtu = uhd::iputils::get_mtu( actual_iface );

	BOOST_CHECK_GE( actual_mtu, expected_mtu );
}


BOOST_AUTO_TEST_CASE( prueba_uno ) {

	// ip route show to match 10.10.10.2
	std::stringstream ss;
	ss << "default via 192.168.128.1 dev enp6s0 src 192.168.128.203 metric 203" << std::endl;
	ss << "10.10.10.0/24 dev enp1s0f1 proto kernel scope link src 10.10.10.10" << std::endl;

	std::string route_info = ss.str();

	std::string remote_addr( "10.10.10.2" );

	std::string expected_iface = "enp1s0f1";
	std::string actual_iface;

	std::string expected_addr( "10.10.10.10" );
	std::string actual_addr;

	uhd::iputils::get_route( route_info, actual_iface, actual_addr );

	BOOST_CHECK_EQUAL( expected_iface, actual_iface );
}

BOOST_AUTO_TEST_CASE( prueba_dos ) {

	// ip route show to match 10.10.10.2
	std::stringstream ss;
	ss << "default via 172.19.50.5 dev enp10s0  proto static  metric 100" << std::endl;
	ss << "default via 192.168.10.2 dev enp0s31f6  proto static  metric 101" << std::endl;
	ss << "default via 10.10.11.2 dev enp1s0f0  proto static  metric 102" << std::endl;
	ss << "default via 10.10.10.2 dev enp1s0f1  proto static  metric 103" << std::endl;
	ss << "10.10.10.0/24 dev enp1s0f1  proto kernel  scope link  src 10.10.10.10  metric 100" << std::endl;

	std::string route_info = ss.str();

	std::string remote_addr( "10.10.10.2" );

	std::string expected_iface = "enp1s0f1";
	std::string actual_iface;

	std::string expected_addr( "10.10.10.10" );
	std::string actual_addr;

	uhd::iputils::get_route( route_info, actual_iface, actual_addr );

	BOOST_CHECK_EQUAL( expected_iface, actual_iface );
}
