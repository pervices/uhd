#include <boost/program_options.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/endian/conversion.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <cmath>

enum class Type
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

template <typename T> size_t to_bits()
{
    return sizeof(T) * CHAR_BIT;
}

template <typename T> size_t to_bytes(const size_t count)
{
    return sizeof(T) * count;
}

class Header
{
public:
    const int packet_size;
    const int packet;
    const TSF tsf;
    const TSI tsi;
    const bool has_tlr;
    const bool has_cid;
    const Type type;
    const bool has_hdr;
    const bool has_tsf;
    const bool has_tsi;
    const bool has_sid;

    Header(const int packet_size, const int packet, const TSF tsf, const TSI tsi, const bool has_tlr, const bool has_cid, const Type type)
    :
    packet_size {packet_size},
    packet {packet},
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
        type == Type::IF_WITH_STREAM_ID ||
        type == Type::EXTENSION_WITH_STREAM_ID
    }
    {}

private:
    int get_data_start() const
    {
        return
            1 * has_hdr +
            1 * has_sid +
            2 * has_cid +
            1 * has_tsi +
            2 * has_tsf;
    }

    int get_data_length() const
    {
        return packet_size - get_data_start() - 1 * has_tlr;
    }

    int get_data_end() const
    {
        return get_data_start() + get_data_length();
    }

public:
    void stamp(std::vector<uint32_t>& data, const uint64_t timestamp) const
    {
        int index = 0;
        data[index++] =
            ((static_cast<int>(packet_size) & 0xFFFF) <<  0) |
            ((static_cast<int>(packet     ) & 0x000F) << 16) |
            ((static_cast<int>(tsf        ) & 0x0003) << 20) |
            ((static_cast<int>(tsi        ) & 0x0003) << 22) |
            ((static_cast<int>(has_tlr    ) & 0x0001) << 26) |
            ((static_cast<int>(has_cid    ) & 0x0001) << 27) |
            ((static_cast<int>(type       ) & 0x000F) << 28);

        if(has_sid)
            data[index++] = 0xBabeCafe;

        if(has_cid)
        {
            data[index++] = 0xDeadBeef;
            data[index++] = 0xBeefDead;
        }

        if(has_tsi)
            data[index++] = 0xDeadFace;

        if(has_tsf)
        {
            const uint32_t mask = 0xFFFFFFFF;
            data[index++] = mask & (timestamp >> to_bits<uint32_t>());
            data[index++] = mask & (timestamp);
        }

        if(has_tlr)
            data[packet_size - 1] = 0xDeadCafe;
    }

    void increment(std::vector<uint32_t>& data) const
    {
        const int a = get_data_start();
        const int b = get_data_end();
        for(int i = a; i < b; i++)
            data[i] = i;
    }

    void sin(std::vector<uint32_t>& data, const float ampl, const float freq) const
    {
        const constexpr float pi = std::acos(-1.0);
        const int a = get_data_start();
        const int b = get_data_end();
        const int n = get_data_length();
        for(int i = a; i < b; i++)
        {
            // Generate.
            const double I = ampl * (std::sin(freq * pi * i / (double) n) + 1.0);
            const double Q = ampl * (std::cos(freq * pi * i / (double) n) + 1.0);

            // Interleave.
            const uint16_t II = I * UINT16_MAX;
            const uint16_t QQ = Q * UINT16_MAX;

            // Set.
            data[i] = II << to_bits<uint16_t>() | QQ;
        }
    }

    void flip_endian(std::vector<uint32_t>& data) const
    {
        for(int i = 0; i < packet_size; i++)
            boost::endian::native_to_big_inplace(data[i]);
    }
};

struct Needle
{
    const std::string addr;
    const std::string port;
    const int packet_count;
    const int packet_size;
    const bool has_tlr;
    const bool has_cid;
    const bool has_tsf;
    const bool has_tsi;
    const bool has_sid;
    const double freq;
    const double ampl;
};

void stream(Needle n)
{
    uhd::transport::udp_simple::sptr udp = uhd::transport::udp_simple::make_connected(n.addr, n.port);
    std::vector<uint32_t> data(n.packet_size, 0);
    const int bytes = to_bytes<uint32_t>(data.size());

    // Will stream forever if packet_count is equal to negative one (-1).
    for(int packet = 0; n.packet_count == -1 ? true : packet < n.packet_count; packet++)
    {
        Header header(
            n.packet_size,
            packet,
            n.has_tsf ? TSF::SAMPLE_COUNT : TSF::NONE,
            n.has_tsi ? TSI::UTC : TSI::NONE,
            n.has_tlr,
            n.has_cid,
            n.has_sid ? Type::IF_WITH_STREAM_ID : Type::IF_WITHOUT_STREAM_ID);

        header.stamp(data, (1 + packet) * n.packet_size);
        header.sin(data, n.ampl, n.freq);
        header.flip_endian(data);

        udp->send(boost::asio::const_buffer(&data.front(), bytes));
    }
}

int main(int argc, char* argv[])
{
    // Command line arguments.
    std::vector<std::string> ports;
    std::vector<std::string> addrs;
    int packet_count;
    int packet_size;
    bool has_tlr;
    bool has_cid;
    bool has_tsf;
    bool has_tsi;
    bool has_sid;
    double freq;
    double ampl;

    // Parse command line arguments.
    namespace po = boost::program_options;
    po::options_description desc("Command line arguments");
    desc.add_options()
        ("help"        , "This help screen")
        ("ports"       , po::value<std::vector<std::string>>(&ports)->multitoken(), "List of IP ports (42809 42810)"              )
        ("addrs"       , po::value<std::vector<std::string>>(&addrs)->multitoken(), "List of IP addresses (10.10.10.2 10.10.10.3)")
        ("packet_count", po::value<int   >(&packet_count)->default_value(8         ), "Number of packets to send (-1 for infinite)" )
        ("packet_size" , po::value<int   >(&packet_size )->default_value(2233      ), "Number of samples per packet"                )
        ("has_tlr"     , po::value<bool  >(&has_tlr     )->default_value(0         ), "Use a Trailer?"                              )
        ("has_cid"     , po::value<bool  >(&has_cid     )->default_value(0         ), "Use a Class ID?"                             )
        ("has_tsf"     , po::value<bool  >(&has_tsf     )->default_value(1         ), "Use a Fractional Time Stamp?"                )
        ("has_tsi"     , po::value<bool  >(&has_tsi     )->default_value(0         ), "Use an Integer Time Stamp?"                  )
        ("has_sid"     , po::value<bool  >(&has_sid     )->default_value(0         ), "Use a Stream ID"                             )
        ("freq"        , po::value<double>(&freq        )->default_value(100.0     ), "Sinewave frequency per packet (not total frequency")
        ("ampl"        , po::value<double>(&ampl        )->default_value(0.1       ), "Sinewave amplitude")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
        std::cout << boost::format("%s") % desc << std::endl;
        std::cout << "Exiting..." << std::endl;
        exit(1);
    }
    if(ports.size() != addrs.size())
    {
        std::cout << "Error: Number of ports must equal number of addrs";
        std::exit(1);
    }

    // Stream one thread per channel (eg. one thread per IP Address and Port pair).
    std::vector<boost::thread> threads {addrs.size()};
    for(size_t i = 0; i < threads.size(); i++)
    {
        const std::string addr = addrs[i];
        const std::string port = ports[i];
        Needle needle = { addr, port, packet_count, packet_size, has_tlr, has_cid, has_tsf, has_tsi, has_sid, freq, ampl };
        threads[i] = boost::thread(stream, needle);
    }

    // Wait for threads to complete.
    for(auto& thread: threads) thread.join();
}
