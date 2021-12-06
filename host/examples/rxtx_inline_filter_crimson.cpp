#include <complex>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include "../lib/usrp/crimson_tng/conv.hpp"
#include "../lib/usrp/crimson_tng/xlate.hpp"

#ifndef SKIP_DSP
//#define SKIP_DSP 1
#endif

#ifndef DEBUG_TX
//#define DEBUG_TX 1
#endif

#ifndef DEBUG_RX
//#define DEBUG_RX 1
#endif

#ifndef LINK_WITH_OCTAVE
// XXX: @CF: 20171212: There is a brutal bug with the version of Octave we have at the lab
// https://savannah.gnu.org/bugs/index.php?52650
//#define LINK_WITH_OCTAVE 1
#endif

#ifdef LINK_WITH_OCTAVE
// use octave's fir1() function to design fir filters. Remember to link to liboctave.
#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>
#include <octave/toplev.h>
#endif

namespace po = boost::program_options;

#ifndef MTU_SAMPLES_MAX
#define MTU_SAMPLES_MAX (1 << 11) // 2kS (should fit in jumbo UDP packet)
#endif

#ifndef DEBUG_OCTAVE
#define DEBUG_OCTAVE 1
#endif

/**
 * Choose an appropriate length 'N' based on desired filter properties. The
 * length of the filter will be rounded up to the nearest even number.
 *
 * Normalized frequencies can also be used, in which case f_s becomes 1
 * and the frequencies used to compute df are 0 <= f < 0.5 (Nyquist rate)
 *
 * @param a_db desired attenuation [ dB ]
 * @param f_s  sample rate of signal [ Hz ]
 * @param df  (delta f) transition width [ Hz ]
 * @return FIR filter length, N
 */
static size_t choose_fir_filter_length( const double & a_db, const double & f_s, const double & df ) {
#ifdef LINK_WITH_OCTAVE
	// https://www.allaboutcircuits.com/technical-articles/design-of-fir-filters-design-octave-matlab/

	size_t N;
	N = std::round( a_db * f_s / 22 / df );
	N = 0 == N % 2 ? N : N + 1;

	return N;
#else
    (void) a_db;
    (void) f_s;
    (void) df;
	//return 56;
	return 18;
#endif
}

template <class T>
struct fir {
	std::vector<T> taps;

	fir() {}

	/**
	 * Construct an FIR filter of order n (length n+1)
	 *
	 * @param n Order of FIR filter
	 * @param w stop / pass transition frequency (normalized)
	 * @param type Either "low" or "high"
	 */
	fir( const std::size_t n, T w, const std::string type = "low" )
	: fir( n, std::vector<T>{ w }, type )
	{
	}

	/**
	 * Construct an FIR filter of order n (length n+1)
	 *
	 * @param n Order of FIR filter
	 * @param w stop / pass transition frequency / frequencies (normalized)
	 *          a single value can be used for "low" or "high". For "bandpass"
	 *          a vector of length 2 is required.
	 * @param type One of "all", "low", "high", or "bandpass"
	 */
	fir(const std::size_t n, const std::vector<T> w, const std::string type = "low" )
	:
		taps( std::vector<T>( n + 1, 0 ) )
	{
//#if LINK_WITH_OCTAVE
// The blow below is for informational purposes only
#if 0
		// https://www.gnu.org/software/octave/doc/v4.0.3/Standalone-Programs.html#Standalone-Programs

		octave_value_list in;
		octave_value_list out;
		std::stringstream ss;
		Matrix fir_matrix;

		// quick & dirty all-pass
		if ( "all" == type ) {
			taps[ 0 ] = 1;
			DO() << "taps: " << taps << std::endl;
			return;
		}

		ss
			<< "fir1( ";

		ss << n << ", ";
		DO() << "in( 0 ) => " << n << std::endl;
		in( 0 ) = octave_value( n );
		if ( 1 == w.size() ) {
			ss << w[ 0 ] << ", ";
			DO() << "in( 1 ) => " << w[ 0 ] << std::endl;
			in( 1 ) = octave_value( w[ 0 ] );
		} else {
			Matrix w_matrix( 1, w.size() );
			for( size_t i = 0; i < w.size(); i++ ) {
				w_matrix( i ) = w[ i ];
			}
			ss << w << ", ";
			DO() << "in( 1 ) => " << w << std::endl;
			in( 1 ) = octave_value( w_matrix );
		}
		ss << type;
		DO() << "in( 2 ) => " << type << std::endl;
		in( 2 ) = octave_value( type );

		ss << " )";

		std::string fir1_cmd = ss.str();

		DO() << "Evaluating command '" << fir1_cmd << "'" << std::endl;

		out = feval( "fir1", in, 1 );
		if ( error_state ) {
			throw new std::runtime_error( "error executing Octave command '" + fir1_cmd + "'" );
		}

		DO() << "Getting output 0" << std::endl;

		fir_matrix = out( 0 ).matrix_value();

		//fir_matrix.print_info( DO(), "fir_matrix info:" );

		if ( 1 != fir_matrix.rows() ) {
			throw std::runtime_error( "Unexpected number of rows " + std::to_string( fir_matrix.rows() ) );
		}

		if ( n + 1 != (size_t) fir_matrix.cols() ) {
			throw std::runtime_error( "Unexpected number of cols " + std::to_string( fir_matrix.rows() ) );
		}

		for( size_t i = 0; i < (size_t) fir_matrix.numel(); i++ ) {
			taps[ i ] = fir_matrix( i );
		}

		DO() << "taps: " << taps << std::endl;
#endif
	}

	~fir() {}
};

class branch {
public:

	bool should_break;

	std::thread tx_thread;
	std::thread async_msg_thread;

	size_t channel;

	std::mutex q_mutex;
	std::deque<std::complex<int16_t>> q;

	uhd::usrp::multi_usrp::sptr usrp;
	uhd::tx_streamer::sptr tx;
	uhd::tx_metadata_t meta;

	std::vector<std::complex<int16_t>> tx_buf;
    std::vector<std::complex<int16_t>> fifo_in;
	std::vector<std::complex<double>> filter_in;
	std::vector<std::complex<double>> filter_out;

    std::vector<void *> tx_bufp;

    fir<double> filter;

	branch()
	: should_break( false ), channel( -1 ), usrp( nullptr ), tx( nullptr )
	{}

	branch( const branch & o )
	{
		*this = o;
	}

	branch & operator=( const branch & o ) {
		this->should_break = o.should_break;
		this->channel = o.channel;
		//this->q = std::deque<std::complex<int16_t>>();
		this->usrp = o.usrp;
		this->tx = o.tx;
		this->meta = o.meta;
		this->tx_buf = std::vector<std::complex<int16_t>>( o.tx_buf.capacity(), 0 );
		this->fifo_in = std::vector<std::complex<int16_t>>( o.fifo_in.capacity(), 0 );
		this->filter_in = std::vector<std::complex<double>>( o.filter_in.capacity(), 0 );
		this->filter_out = std::vector<std::complex<double>>( o.filter_out.capacity(), 0 );
		this->tx_bufp = std::vector<void *>{ & tx_buf.front() };
		this->filter = o.filter;
		return *this;
	}

	branch( const size_t & channel, const size_t & q_size, uhd::usrp::multi_usrp::sptr & usrp )
	:
		should_break( false ),
		channel( channel ),
		q( q_size, 0 ),
		usrp( usrp ),
		tx_buf( MTU_SAMPLES_MAX, 0 ),
		fifo_in( MTU_SAMPLES_MAX, 0 ),
		filter_in( MTU_SAMPLES_MAX, 0 ),
		filter_out( MTU_SAMPLES_MAX, 0 ),
		tx_bufp( 1, & tx_buf.front() )
	{
		uhd::stream_args_t args( "sc16", "sc16" );
		args.channels = std::vector<size_t>{ channel };
		tx = usrp->get_tx_stream( args );

		switch( channel ) {
		// fir1( 56, 0.15, "all" )
		case 0:
			filter.taps = std::vector<double>{ 1 };
			break;
		// c++: fir1( 56, 0.15, "low" )
		// matlab: fprintf( stdout, '%.6f, ', fftshift( fir1( 18, 0.15, "low" ) ) ); fprintf( stdout, '\n' );
		case 1:
			//filter.taps = std::vector<double>{ 0.143463000, 0.126915000, 0.102108000, 0.072637200, 0.042548300, 0.015606800, -0.005327740, -0.018707300, -0.024409900, -0.023566700, -0.018163900, -0.010531300, -0.002838180, 0.003293820, 0.006994560, 0.008149110, 0.007235390, 0.005064040, 0.002501450, 0.000247257, -0.001289310, -0.002004910, -0.002031080, -0.001618620, -0.001023590, -0.000430342, 0.000072604, 0.000469784, 0.000469784, 0.000072604, -0.000430342, -0.001023590, -0.001618620, -0.002031080, -0.002004910, -0.001289310, 0.000247257, 0.002501450, 0.005064040, 0.007235390, 0.008149110, 0.006994560, 0.003293820, -0.002838180, -0.010531300, -0.018163900, -0.023566700, -0.024409900, -0.018707300, -0.005327740, 0.015606800, 0.042548300, 0.072637200, 0.102108000, 0.126915000, 0.143463000, 0.149273000 };
			filter.taps = std::vector<double>{ 0.146527, 0.120017, 0.084537, 0.049414, 0.022062, 0.005633, -0.001210, -0.002553, -0.002607, -0.002607, -0.002553, -0.001210, 0.005633, 0.022062, 0.049414, 0.084537, 0.120017, 0.146527, 0.156358 };
			break;
		// fir1( 56, [ 0.15, 0.3 ], "bandpass" )
		case 2:
			//filter.taps = std::vector<double>{ 0.112512000, 0.023114400, -0.068809500, -0.115922000, -0.101222000, -0.044155000, 0.016601000, 0.049494500, 0.047065000, 0.024149500, 0.002337260, -0.005843240, -0.002176240, 0.003476940, 0.003351660, -0.002671590, -0.009066450, -0.010620200, -0.006629940, -0.000491570, 0.003772930, 0.004509870, 0.002796620, 0.000737295, -0.000317636, -0.000296497, 0.000150497, 0.000365213, 0.000365213, 0.000150497, -0.000296497, -0.000317636, 0.000737295, 0.002796620, 0.004509870, 0.003772930, -0.000491570, -0.006629940, -0.010620200,-0.009066450, -0.002671590, 0.003351660, 0.003476940, -0.002176240, -0.005843240, 0.002337260, 0.024149500, 0.047065000, 0.049494500, 0.016601000, -0.044155000, -0.101222000, -0.115922000, -0.068809500, 0.023114400, 0.112512000, 0.149384000 };
			filter.taps = std::vector<double>{ 0.168553, 0.032060, -0.083560, -0.115670, -0.076984, -0.023376, 0.005530, 0.009909, 0.007372, 0.007372, 0.009909, 0.005530, -0.023376, -0.076984, -0.115670, -0.083560, 0.032060, 0.168553, 0.229510 };
			break;
		// fir1( 56, 0.3, "high" )
		case 3:
			//filter.taps = std::vector<double>{ -0.256454000, -0.150062000, -0.032876500, 0.043927800, 0.059220800, 0.028785000, -0.011361900, -0.031053500, -0.022912800, -0.000722600, 0.015802300,0.016397300, 0.005023640, -0.006786340, -0.010358700, -0.005458300, 0.001882420, 0.005614030, 0.004164160, 0.000246999, -0.002503820, -0.002529430, -0.000781247, 0.000876448, 0.001342160, 0.000728066, -0.000223823, -0.000836550, -0.000836550, -0.000223823, 0.000728066, 0.001342160, 0.000876448, -0.000781247, -0.002529430, -0.002503820, 0.000246999, 0.004164160, 0.005614030, 0.001882420, -0.005458300, -0.010358700, -0.006786340, 0.005023640, 0.016397300, 0.015802300, -0.000722600, -0.022912800, -0.031053500, -0.011361900, 0.028785000, 0.059220800, 0.043927800, -0.032876500, -0.150062000, -0.256454000, 0.701673000 };
			filter.taps = std::vector<double>{ -0.249416, -0.135125, -0.025919, 0.028456, 0.029240, 0.009893, -0.002457, -0.004036, -0.002330, -0.002330, -0.004036, -0.002457, 0.009893, 0.029240, 0.028456, -0.025919, -0.135125, -0.249416, 0.699860 };
			break;
		default:
			throw std::runtime_error( "no filter designed for channel " + std::to_string( channel ) );
			break;
		}
	}

	void set_meta( const uhd::tx_metadata_t & meta ) {
		this->meta = meta;
	}

	void start() {
		should_break = false;
		async_msg_thread = std::thread( branch::async_msg_func, this );
		tx_thread = std::thread( branch::tx_func, this );
	}

	void stop() {
		should_break = true;
		async_msg_thread.join();
		tx_thread.join();
	}

	/**
	 * Push samples onto the sample FIFO.
	 *
	 * @param samples Vector of samples
	 */
	void pushq( const std::vector<std::complex<int16_t>> & data ) {

		std::lock_guard<std::mutex> lock( q_mutex );

		size_t n = data.size();
		q.insert( q.end(), data.begin(), data.begin() + n );

		//std::cout << __func__ << "(): " << __LINE__ << ": " << "Pushed " << samples.size() << " samples" << std::endl << std::flush;
	}

	/**
	 * Pop samples from the sample FIFO.
	 *
	 * @param samples Vector of samples
	 */
	void popq() {

		std::lock_guard<std::mutex> lock( q_mutex );

		size_t n = std::min( fifo_in.capacity(), q.size() );

		fifo_in.clear();
		fifo_in.insert( fifo_in.begin(), q.begin(), q.begin() + n );
		q.erase( q.begin(), q.begin() + n );

		//std::cout << __func__ << "(): " << __LINE__ << ": " << "Popped " << nsamp << " samples" << std::endl << std::flush;
	}

	static void async_msg_func( branch *b ) {

		uhd::async_metadata_t as;

		for( ; ! b->should_break;  ) {
			if( b->tx->recv_async_msg( as ) ) {
				switch( as.event_code ) {
				case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW:
					std::cout << "U" << std::flush;
					break;
				default:
					break;
				}
			}
		}

		std::cout << __func__ << "(): " << __LINE__ << ": " << "Out of Async Msg Loop" << std::endl << std::flush;
	}

	static void tx_func( branch *b ) {

	    size_t n;

		for( ; ! b->should_break ; ) {

			b->popq();
			n = b->fifo_in.size();

			if ( n > 0 ) {
#ifdef SKIP_DSP
				b->tx_buf.clear();
				b->tx_buf.insert( b->tx_buf.begin(), b->fifo_in.begin(), b->fifo_in.end() );
#else
				b->filter_in.resize( n );
				b->filter_out.resize( n );

				to_double( b->fifo_in, b->filter_in );

				// N.B. this is a relatively slow way to perform digital filtering.
				// Optimized frequency domain methods should be used in practise.
				conv( b->filter_in, b->filter.taps, b->filter_out, "trunc" );

				to_int16( b->filter_out, b->tx_buf );
#endif

				b->tx->send( b->tx_bufp, n, b->meta );
				// ensure that TX sob happens only once
				b->meta.has_time_spec = false;
				b->meta.start_of_burst = false;

#ifdef DEBUG_TX
				std::cout
					<< __func__ << "(): " << __LINE__ << ": "
					<< std::setprecision( 6 ) << b->usrp->get_time_now().get_real_secs() << ": "
					<< "Channel[ " << (char)(b->channel + 'A') << " ] "
					<< "sent " << n << " samples"
					<< std::endl << std::flush;
#endif
			}
		}

		std::cout << __func__ << "(): " << __LINE__ << ": " << "Out of TX Loop" << std::endl << std::flush;
	}
};

/**
 * Parse channels from "0,1,2,3" to { 0, 1, 2, 3 }
 * @param max max channel number
 * @param channel_list string of the format "0,1,2,3"
 * @return vector of the format { 0, 1, 2, 3 }
 */
static std::vector<size_t> parse_channels( const size_t & max, const std::string & channel_list ) {

    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;

    boost::split( channel_strings, channel_list, boost::is_any_of("\"',") );

    for( size_t ch = 0; ch < channel_strings.size(); ch++ ){

        size_t chan = boost::lexical_cast<size_t>( channel_strings[ ch ] );

        if( chan >= max ) {
            throw std::runtime_error( "Invalid channel(s) specified." );
        }

        channel_nums.push_back( chan );
    }

    return channel_nums;
}

/// flag set by signal handler
static bool should_break;
/// simple signal handler to break the infinite loop in main()
static void sighndlr( int x ) {
    (void) x;
	should_break = true;
}

/// Initialize liboctave (it's used for FIR filter design)
static void octave_init() {
#ifdef LINK_WITH_OCTAVE
    // initialize Octave
	string_vector oct_argv( 2 );
	oct_argv( 0 ) = "embedded";
	oct_argv( 1 ) = "-q";
	octave_main( 2, oct_argv.c_str_vec(), 1 );

	DO() << "Loading 'signal' package" << std::endl;

	feval( "pkg", ovl( "load", "signal" ), 0 );
	if ( error_state ) {
		throw new std::runtime_error( "error executing Octave command 'pkg load signal'. You may need to run this manually." );
	}
#endif
}

/// Finalize liboctave (it's used for FIR filter design)
static void octave_fini( int status ) {
#ifdef LINK_WITH_OCTAVE
	// N.B. AFAIK, there is no way to do this without liboctave calling exit( status ) for us

    DO() << "cleaning up Octave environment" << std::endl;
    clean_up_and_exit( status );
#endif
#ifndef LINK_WITH_OCTAVE
    (void) status;
#endif
}

/**
 * This example program turns Crimson into an inline filter. It's kind of the opposite of a loopback test.
 *
 * We take an input on RX A from a signal generator generating a linear chirp at 9 MHz (250 kHz deviation).
 * The center frequency is set to 9MHz on Crimson, and the sample rate is 500 kSps. We use a low sample
 * rate so that the host can keep up. The user may decide to speed-up filtering e.g. in the frequency
 * domain.
 *
 * The filter-length was limited to 18 taps to reduce complexity using conv(). Higher-order filters would
 * result in better selectivity, but would require frequency-domain filtering.
 */
int UHD_SAFE_MAIN( int argc, char *argv[] ) {

    uhd::set_thread_priority_safe();

    double samp_rate;
    double freq;
    double atten_db;
    double delta_f;
    double rx_sob;
    double tx_sob;
    std::string channel_str;

    po::options_description desc( "Allowed options" );
    desc.add_options()
        ( "help", "this help message" )
        ( "channels", po::value<std::string>( &channel_str )->default_value( "0,1,2,3" ), "comma-separated list of channels to use (no spaces)" )
		( "freq", po::value<double>( &freq )->default_value( 9e6 ), "center frequency" )
        ( "rate", po::value<double>( &samp_rate )->default_value( 500e3 ), "sample rate" )
		( "atten", po::value<double>( &atten_db )->default_value( 20 ), "stop-band attenuation" )
		( "trans", po::value<double>( &delta_f )->default_value( 25e3 ), "filter transition width (Hz)" )
		( "rx-sob", po::value<double>( &rx_sob )->default_value( 4 ), "delay until rx begins" )
		( "tx-sob", po::value<double>( &tx_sob )->default_value( 6 ), "delay until tx begins" )
    ;
    po::variables_map vm;
    po::store( po::parse_command_line( argc, argv, desc ), vm );
    po::notify( vm );

    if ( rx_sob < 0 ) {
    	throw std::invalid_argument( "rx_sob must be >= 0, not " + std::to_string( rx_sob ) );
    }
    if ( tx_sob < 0 ) {
    	throw std::invalid_argument( "tx_sob must be >= 0, not " + std::to_string( tx_sob ) );
    }

    if ( tx_sob < rx_sob ) {
    	throw std::invalid_argument( "tx_sob must be >= rx_sob, not " + std::to_string( tx_sob ) );
    }

    //print the help message
    if ( vm.count( "help" ) ) {
        std::cout << "UHD RX Timed Samples" << std::endl;
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    //
    // Done processing options, now initialize structures, memory, hardware
    //

    const size_t N = choose_fir_filter_length( atten_db, samp_rate, delta_f );

    std::cout << "Selected FIR filter length " << N << std::endl;

    octave_init();

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make( uhd::device_addr_t( "" ) );

    uhd::stream_args_t rx_args( "sc16", "sc16" );
    rx_args.channels = std::vector<size_t>{ 0 };

    uhd::stream_args_t tx_args( "sc16", "sc16" );
    std::vector<size_t> channels = parse_channels( usrp->get_tx_num_channels(), channel_str );

    const size_t nchannels = channels.size();

    usrp->set_rx_rate( samp_rate, 0 );
    usrp->set_rx_freq( uhd::tune_request_t( freq ), 0 );
    usrp->set_rx_gain( 0, 0 );

    for( auto & c: channels ) {
        usrp->set_tx_rate( samp_rate, c );
        usrp->set_tx_freq( uhd::tune_request_t( freq ), c );
        usrp->set_tx_gain( 0, c );
    }

	size_t q_size = std::max( (size_t)MTU_SAMPLES_MAX, (size_t)( ( tx_sob - rx_sob ) * 2 ) );

	std::vector<branch> tree( nchannels );

	std::vector<std::complex<int16_t>> rx_buf( MTU_SAMPLES_MAX, 0 );
    std::vector<void *> rx_bufp{ & rx_buf.front() };

    uhd::rx_metadata_t rx_meta;
    uhd::tx_metadata_t tx_meta;

    uhd::rx_streamer::sptr rx = usrp->get_rx_stream( rx_args );

    for( size_t i = 0; i < nchannels; i++ ) {
    	tree[ i ] = branch( channels[ i ], q_size, usrp );
    }

    //
    // Install signal handler to break us out of infinite signal processing loop
    //

    std::signal( SIGINT, sighndlr );

    //
    // Signal processing below
    //
    // Here, we process individual filters sequentially (slowest). Possible optimizations
    // include
    //
    // a) breaking rx, tx, and per-channel dsp out to individual threads
    // b) performing dsp in separate threads
    // c) using SIMD for dsp routines, performing DSP in the frequency domain, or linking to an external DSP library
    // d) double-buffering to allow rx / processing / tx stages to be pipelined
    //

    double rx_timeout = 0.1;
    uhd::stream_cmd_t stream_cmd( uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS );

    uhd::time_spec_t now = usrp->get_time_now();

    if ( 0 == rx_sob ) {
		stream_cmd.stream_now = true;
		rx->issue_stream_cmd( stream_cmd );
    } else {
    	uhd::stream_cmd_t stream_cmd( uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS );
		stream_cmd.stream_now = false;
		stream_cmd.num_samps = MTU_SAMPLES_MAX;
		stream_cmd.time_spec = now + rx_sob;
		rx_timeout += rx_sob;
		rx->issue_stream_cmd( stream_cmd );

		std::cout
			<< __func__ << "(): " << __LINE__ << ": "
			<< std::setprecision( 6 ) << now.get_real_secs() << ": "
			<< "Wait " << rx_sob << " to receive samples"
			<< std::endl;
    }

    if ( 0 != tx_sob ) {

		tx_meta.has_time_spec = true;
		tx_meta.start_of_burst = true;
		tx_meta.time_spec = now + tx_sob;

		std::cout
			<< __func__ << "(): " << __LINE__ << ": "
			<< std::setprecision( 6 ) << now.get_real_secs() << ": "
			<< "Wait " << tx_sob << " to send samples"
			<< std::endl << std::flush;
    }

    // fire up TX threads
    std::vector<std::thread> tx_thread;
    for( size_t i = 0; i < nchannels; i++ ) {
    	tree[ i ].set_meta( tx_meta );
    	tree[ i ].start();
    }

    for( ; ! should_break; ) {

    	size_t nsamp = rx->recv( rx_bufp, MTU_SAMPLES_MAX, rx_meta, rx_timeout );
    	rx_timeout = 0.1;

#ifdef DEBUG_RX
		std::cout
			<< __func__ << "(): " << __LINE__ << ": "
			<< std::setprecision( 6 ) << ( now + rx_meta.time_spec ).get_real_secs() << ": "
			<< "Received " << nsamp << " samples"
			<< std::endl;
#endif

		rx_buf.resize( nsamp );
		for( size_t i = 0; i < nchannels; i++ ) {
			tree[ i ].pushq( rx_buf );
		}
    }

    std::cout << __func__ << "(): " << __LINE__ << ": " << "Out of RX Loop" << std::endl << std::flush;

    //
    // Clean up and exit
    //

    for( size_t i = 0; i < nchannels; i++ ) {
    	tree[ i ].stop();
    }

    stream_cmd = uhd::stream_cmd_t( uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS );
    rx->issue_stream_cmd( stream_cmd );

    octave_fini( EXIT_SUCCESS );

    // liboctave exits for us (for better or worse), so should not expect to get here
    std::cout << "exiting.." << std::endl;
    return EXIT_SUCCESS;
}
