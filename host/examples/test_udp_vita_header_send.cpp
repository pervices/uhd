#include <boost/program_options.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/endian/conversion.hpp>
#include <boost/format.hpp>
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

template <typename T>
struct Load
{
    std::vector<T> data;
    const int bytes;

    Load(const int packet_size)
    :
    data(packet_size),
    bytes(packet_size * sizeof(T))
    {}
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

    Header(const int words, const int count, const TSF tsf, const TSI tsi, const bool has_tlr, const bool has_cid, const Packet type)
    :
    words {words},
    count {count},
    tsf {tsf},
    tsi {tsi},
    has_tlr {has_tlr},
    has_cid {has_cid},
    type {type},
    has_hdr {true},
    has_tsf {
        tsf != TSF::NONE
    },
    has_tsi {
        tsi != TSI::NONE
    },
    has_sid {
        type == Packet::IF_WITH_STREAM_ID || type == Packet::EXTENSION_WITH_STREAM_ID
    }
    {}

private:
    int get_data_start() const
    {
        return 1 * has_hdr + 1 * has_sid + 2 * has_cid + 1 * has_tsi + 2 * has_tsf;
    }

    int get_data_length() const
    {
        return words - get_data_start() - 1 * has_tlr;
    }

public:
    template <typename T>
    void stamp(Load<T>& load) const
    {
        int index = 0;
        load.data[index++] =
            ((static_cast<int>(words  ) & 0xFFFF) <<  0) |
            ((static_cast<int>(count  ) & 0x000F) << 16) |
            ((static_cast<int>(tsf    ) & 0x0003) << 20) |
            ((static_cast<int>(tsi    ) & 0x0003) << 22) |
            ((static_cast<int>(has_tlr) & 0x0001) << 26) |
            ((static_cast<int>(has_cid) & 0x0001) << 27) |
            ((static_cast<int>(type   ) & 0x000F) << 28);

        if(has_sid)
            load.data[index++] = 0xBabeCafe;

        if(has_cid)
        {
            load.data[index++] = 0xDeadBeef;
            load.data[index++] = 0xBeefDead;
        }

        if(has_tsi)
            load.data[index++] = 0xDeadFace;

        if(has_tsf)
            load.data[index++] = 0xDeadDead;

        if(has_tlr)
            load.data[words-1] = 0xDeadCafe;
    }

    template <typename T>
    void increment(Load<T>& load) const
    {
        const int a = get_data_start();
        const int b = get_data_start() + get_data_length();
        for(int i = a; i < b; i++)
            load.data[i] = i;
    }

    template <typename T>
    void flip_endian(Load<T>& load) const
    {
        for(int i = 0; i < words; i++)
            boost::endian::native_to_big_inplace(load.data[i]);
    }
};

int main(int argc, char* argv[])
{
    std::string addr;
    std::string port;
    int packet_count;
    int packet_size;
    bool has_tlr;
    bool has_cid;
    bool has_tsf;
    bool has_tsi;
    bool has_sid;
    boost::program_options::options_description desc("Command line arguments");
    desc.add_options()
        ("help"        , "This help screen")
        ("addr"        , boost::program_options::value<std::string>(&addr)->default_value("10.10.10.2"), "IP address of target machine")
        ("port"        , boost::program_options::value<std::string>(&port)->default_value("42809"     ), "Port of target machine"      )
        ("packet_count", boost::program_options::value<int>(&packet_count)->default_value(32          ), "Number of packets to send"   )
        ("packet_size" , boost::program_options::value<int>(&packet_size )->default_value(2233        ), "Number of samples per packet")
        ("has_tlr"     , boost::program_options::value<bool>(&has_tlr    )->default_value(1           ), "Use a Trailer?"              )
        ("has_cid"     , boost::program_options::value<bool>(&has_cid    )->default_value(1           ), "Use a Class ID?"             )
        ("has_tsf"     , boost::program_options::value<bool>(&has_tsf    )->default_value(1           ), "Use a Fractional Time Stamp?")
        ("has_tsi"     , boost::program_options::value<bool>(&has_tsi    )->default_value(1           ), "Use an Integer Time Stamp?"  )
        ("has_sid"     , boost::program_options::value<bool>(&has_sid    )->default_value(1           ), "Use a Stream ID"             )
    ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if(vm.count("help"))
    {
        std::cout << boost::format("%s") % desc << std::endl;
        std::cout << "Exiting" << std::endl;
        exit(1);
    }

    uhd::transport::udp_simple::sptr udp = uhd::transport::udp_simple::make_connected(addr, port);

    for(int packet = 0; packet < packet_count; packet++)
    {
        Load<uint32_t> load(packet_size);

        Header header(
            packet_size,
            packet,
            has_tsf ? TSF::SAMPLE_COUNT : TSF::NONE,
            has_tsi ? TSI::UTC : TSI::NONE,
            has_tlr,
            has_cid,
            has_sid ? Packet::IF_WITH_STREAM_ID : Packet::IF_WITHOUT_STREAM_ID);

        header.stamp(load);
        header.increment(load);
        header.flip_endian(load);

        udp->send(boost::asio::const_buffer(&load.data.front(), load.bytes));
    }
}
