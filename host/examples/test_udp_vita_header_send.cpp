#include <uhd/transport/udp_simple.hpp>
#include <boost/endian/conversion.hpp>
#include <iostream>

void puthead(uint32_t load[], const int size, const int count)
{
    load[0] =
        ((size  & 0xFFFF) <<  0) | // Packet Size.
        ((count & 0x000F) << 16) | // Packet Count.
        ((0x00  & 0x0003) << 20) | // Time Stamp Fractional.
        ((0x00  & 0x0003) << 22) | // Time Stamp Integer.
        ((0x00  & 0x0001) << 26) | // Trailer.
        ((0x00  & 0x0001) << 27) | // Class ID.
        ((0x00  & 0x000F) << 28);  // Packet Type.
}

void putdata(uint32_t load[], const int size)
{
    for(int i = 1; i < size; i++)
        load[i] = i;
}

void endflip(uint32_t load[], const int size)
{
    for(int i = 0; i < size; i++)
	    boost::endian::native_to_big_inplace(load[i]);
}

void setup(uint32_t load[], const int size, const int count)
{
    puthead(load, size, count);
    putdata(load, size);
    endflip(load, size);
}

int main(int argc, char* argv[])
{
    // Parse command line args.
    if(argc != 4)
    {
        std::cout << "./test_udp_vita_header_send ip port packets" << std::endl;
        std::cout << "eg:" << std::endl;
        std::cout << "./test_udp_vita_header_send 10.10.10.2 42809 16" << std::endl;
        std::exit(1);
    }
    const std::string addr = argv[1];
    const std::string port = argv[2];
    const int packets = std::stoi(argv[3]);

    for(int packet = 0; packet < packets; packet++)
    {
        uint32_t load[2233];
        setup(load, sizeof(load) / sizeof(load[0]), packet);

        uhd::transport::udp_simple::sptr udp = uhd::transport::udp_simple::make_connected(addr, port);
        udp->send(boost::asio::const_buffer(&load, sizeof(load)));
    }
}
