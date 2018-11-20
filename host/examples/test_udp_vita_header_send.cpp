#include <uhd/transport/udp_simple.hpp>
#include <boost/endian/conversion.hpp>
#include <iostream>

enum class Packet
{
    IF_WITHOUT_STREAM_ID, IF_WITH_STREAM_ID, EXTENSION_WITHOUT_STREAM_ID, EXTENSION_WITH_STREAM_ID, CONTEXT, EXTENSION_CONTEX, RESERVED
};

enum class TSI
{
    NONE, UTC, GPS, OTHER
};

enum class TSF
{
    NONE, SAMPLE_COUNT, PICOSECOND, FREE_RUNNING
};

class Header
{
public:
    const int words;
    const int count;
    const TSF tsf;
    const TSI tsi;
    const bool has_tlr;
    const bool has_cid;
    const Packet type;
    const bool has_hdr;
    const bool has_tsf;
    const bool has_tsi;
    const bool has_sid;
    Header(const int words, const int count, const TSF tsf, const TSI tsi, const bool has_cid, const bool has_tlr, const Packet type)
    :
    words{words},
    count{count},
    tsf{tsf},
    tsi{tsi},
    has_tlr{has_tlr},
    has_cid{has_cid},
    type{type},
    has_hdr{true},
    has_tsf{tsf != TSF::NONE},
    has_tsi{tsi != TSI::NONE},
    has_sid{(type == Packet::IF_WITH_STREAM_ID) || (type == Packet::EXTENSION_WITH_STREAM_ID)}
    {
    }

    int get_data_start() const
    {
        return (1 * has_hdr) + (1 * has_sid) + (2 * has_cid) + (1 * has_tsi) + (2 * has_tsf);
    }

    int get_data_length() const
    {
        return words - get_data_start() - (1 * has_tlr);
    }
};

class Buffer
{
public:
    uint32_t load[2233] = {};
    const int bytes = sizeof(load);
    const int words = bytes / sizeof(load[0]);

    void stamp(const Header header)
    {
        load[0] =
            ((header.words                  & 0xFFFF) <<  0) |
            ((header.count                  & 0x000F) << 16) |
            ((static_cast<int>(header.tsf)  & 0x0003) << 20) |
            ((static_cast<int>(header.tsi)  & 0x0003) << 22) |
            ((header.has_tlr                & 0x0001) << 26) |
            ((header.has_cid                & 0x0001) << 27) |
            ((static_cast<int>(header.type) & 0x000F) << 28);

        load[1] = header.has_sid ? 0xCafeBabe : 0x0;
        load[2] = header.has_cid ? 0xCafeBabe : 0x0;
        load[3] = header.has_cid ? 0xCafeBabe : 0x0;
        load[4] = header.has_tsi ? 0xCafeBabe : 0x0;
        load[5] = header.has_tsf ? 0xCafeBabe : 0x0;
        load[words - 1] = header.has_tlr ? 0xCafeBabe : 0x0;
    }

    void increment(const Header header)
    {
        const int a = header.get_data_start();
        const int b = header.get_data_start() + header.get_data_length();
        for(int i = a; i < b; i++)
            load[i] = i;
    }

    void flip_endian(const Header header)
    {
        for(int i = 0; i < header.words; i++)
            boost::endian::native_to_big_inplace(load[i]);
    }
};

int main(int argc, char* argv[])
{
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
        Buffer buffer;
        const Header header(buffer.words, packet, TSF::SAMPLE_COUNT, TSI::UTC, true, true, Packet::IF_WITH_STREAM_ID);

        buffer.stamp(header);
        buffer.increment(header);
        buffer.flip_endian(header);

        uhd::transport::udp_simple::sptr udp = uhd::transport::udp_simple::make_connected(addr, port);
        udp->send(boost::asio::const_buffer(&buffer.load, buffer.bytes));
    }
}
