#include <map>
#include <string>

static std::map<const std::string,int> to = { { "A", 0 }, { "B", 1 }, { "C", 2 }, { "D", 3 } };
static std::map<int,const std::string> fro = { { 0, "A" }, { 1, "B" }, { 2, "C" }, { 3, "D" } };

//
// Copyright 2010-2012,2014 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <complex>
#include <cstdlib>
#include <string>
#include <vector>

#include <fstream>
#include <iostream>

#include "boost/algorithm/string.hpp"
#include "boost/lexical_cast.hpp"

#include <uhd/exception.hpp>

using namespace std;

//
// <SNIP src="http://archive.oreilly.com/network/2003/05/06/examples/calculatorexample.html">
//

#include "boost/spirit.hpp"
#include "boost/spirit/home/classic/phoenix/binders.hpp"
#include "boost/lambda/lambda.hpp"
#include "boost/lambda/bind.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <istream>
#include <map>
#include <ostream>
#include <string>

// Semantic actions can be functors. The operator() function is called
// with two iterators that specify the range of input that matches the production.
//struct do_help {
//  template<typename Iter>
//  void operator()(Iter, Iter) const
//  {
//    std::cout << "help - to be implemented\n";
//  }
//};
//
//struct do_quit {
//  template<typename Iter>
//  void operator()(Iter, Iter) const
//  {
//    std::exit(EXIT_SUCCESS);
//  }
//};

// Symbol table for storing variables.
typedef std::map<std::string, double> symtab_t;

struct calculator : boost::spirit::grammar<calculator>
{
  // The parser object is copied a lot, so instead of keeping its own table
  // of variables, it keeps track of a reference to a common table.
  calculator(symtab_t& variables) : variables(variables) {}

  // A production can have an associated closure, to store information
  // for that production.
  struct value_closure : boost::spirit::closure<value_closure, double>
  {
    member1 value;
  };

  struct assignment_closure :
    boost::spirit::closure<assignment_closure, std::string, double>
  {
    member1 name;
    member2 value;
  };

  struct string_closure : boost::spirit::closure<string_closure, std::string>
  {
    member1 name;
  };

  // Following is the grammar definition.
  template <typename ScannerT>
  struct definition
  {
    //definition(calculator const& self)
	definition(calculator const& self)
    {
      using namespace boost::spirit;
      using namespace phoenix;

      // The commands are linked to functors or member functions,
      // to demonstrate both styles. In real code, you should choose
      // one style and use it uniformly.
//      command
//        = as_lower_d["help"][do_help()]
//        | as_lower_d["quit"][do_quit()]
//        | as_lower_d["dump"][bind(&calculator::dump)(self)]
//        ;

      // The lexeme_d directive tells the scanner to treat white space as
      // significant. Thus, an identifier cannot have internal white space.
      // The alpha_p and alnum_p parsers are built-in.
      // Notice how the semantic action uses a Phoenix lambda function
      // that constructs a std::string. The arg1 and arg2 placeholders are
      // are bound at runtime to the iterator range that matches this rule.
      identifier
        = lexeme_d
        [
          ( alpha_p | '_')
          >> *( alnum_p | '_')
        ][identifier.name = construct_<std::string>(arg1, arg2)]
        ;

      group
        = '('
          >> expression[group.value = arg1]
          >> ')'
        ;

      // An assignment statement must store the variable name and value.
      // The name and the value are stored in the closure, then the define
      // function is called to store the definition. Notice how a rule can
      // have multiple semantic actions.
      assignment
        = identifier[assignment.name = arg1]
          >> '='
          >> expression[assignment.value = arg1]
               [bind(&calculator::define)(self, assignment.name, assignment.value)]
        ;

      // A statement can end at the end of the line, or with a semicolon.
      statement
        =   ( command
            | assignment
//            | expression[bind(&calculator::do_print)(self, arg1)]
			| expression[bind(&calculator::define)(self, "ans", arg1)]
          )
        >> (end_p | ';')
        ;

      // The longest_d directive is built-in to tell the parser to make
      // the longest match it can. Thus "1.23" matches real_p rather than
      // int_p followed by ".23".
      literal
        = longest_d
        [
          int_p[literal.value = arg1]
          | real_p[literal.value = arg1]
        ]
        ;

      // A variable name must be looked up. This is a straightforward
      // Phoenix binding.
      factor
        = literal[factor.value = arg1]
        | group[factor.value = arg1]
        | identifier[factor.value = bind(&calculator::lookup)(self, arg1)]
        ;

      term
        = factor[term.value = arg1]
          >> *( ('*' >> factor[term.value *= arg1])
              | ('/' >> factor[term.value /= arg1])
            )
        ;

      expression
        = term[expression.value = arg1]
          >> *( ('+' >> term[expression.value += arg1])
              | ('-' >> term[expression.value -= arg1])
            )
        ;
    }

    // The start symbol is returned from start().
    boost::spirit::rule<ScannerT> const&
    start() const { return statement; }

    // Each rule must be declared, optionally with an associated closure.
    boost::spirit::rule<ScannerT> command, statement;
    boost::spirit::rule<ScannerT, assignment_closure::context_t> assignment;
    boost::spirit::rule<ScannerT, string_closure::context_t> identifier;
    boost::spirit::rule<ScannerT, value_closure::context_t> expression, factor,
                                                            group, literal, term;
  };

  // Member functions that are called in semantic actions.
  void define(const std::string& name, double value) const
  {
    variables[name] = value;
  }

  double lookup(const std::string& name) const
  {
    symtab_t::iterator it = variables.find(name);
    if (it == variables.end()) {
      std::cerr << "undefined name: " << name << '\n';
      return 0.0;
    }
    else
      return (*it).second;
  }

  void do_print(double x) const
  {
    std::cout << x << '\n';
  }

  void dump() const
  {
    // Dump the entire symbol table. Notice how this function uses
    // Boost lambda functions instead of Phoenix, just to show you that
    // you can mix the two in a single file.
    using namespace boost::lambda;
    typedef std::pair<const std::string, double> symtab_pair;
    for_each(variables.begin(), variables.end(),
    	[]( symtab_pair &n ){  std::cout << n.first << '=' << n.second << '\n'; } );
  }

  double crunch( std::string str ) {

	  using namespace boost::spirit;
	  using namespace std;

	  double r;

	  parse_info<string::iterator> info;
	  info = boost::spirit::parse( str.begin(), str.end(), *this, space_p );
	  r = variables["ans"];

	  return r;
  }

private:
  symtab_t& variables;
};

//
// </SNIP>
//

/***********************************************************************
 * StreamData Class for buffering data for several channels at once from file
 *
 * Input / Output CSV format is as follows
 *
 ***********************************************************************
 * Line 1: Channels
 * Line 2: TX Center Frequencies
 * Line 3: RX Center Frequencies
 * Line 4: TX Sample Rates
 * Line 5: RX Sample Rates
 * Line 6+2*N: I-samples (signed 16-bit integers)
 * Line 7+2*N: Q-samples (signed 16-bit integers)
 *
 * Set all TX ( RX ) sample rates the same to avoid bottlenecks.
 * Output CSV contains the _actual_ frequencies / sample rates the
 * SDR is capable of.
 ***********************************************************************
 * E.g.
 *
 * A,B,D
 * 915e6,2.45e9,5.8e9
 * 915e6,2.45e9,5.8e9
 * 200e6/30,200e6/30,200e6/30
 * 200e6/30,200e6/30,200e6/30
 * 32767,32767,32767
 * 0,0,0
 * 32728,32767,32767
 * 1617,27,36
 * ...
 *
 **********************************************************************/

class StreamData {

public:

	static const char preferred_delimiter = '\t';

	size_t n_channels;
	size_t n_samples;
	std::vector<std::string> channels;
	std::map<std::string, double> rx_center_freq;
	std::map<std::string, double> tx_center_freq;
	std::map<std::string, double> rx_sample_rate;
	std::map<std::string, double> tx_sample_rate;

	std::map<const std::string, std::map<size_t, std::complex<short>>> samples;

	StreamData()
	: n_channels( 0 ), n_samples( 0 )
    {
	}

	virtual ~StreamData() {
	}

	static void fromFile( StreamData& r, std::string& fn ) {

		std::string line;
		int lineno;
		int sampleno;
		std::vector<std::string> split;

		std::ifstream fs( fn );

		symtab_t variables;
		calculator calc( variables );
		variables["pi"] = 3.141592653589792;

		for( lineno = 0; ; lineno++ ) {
			std::getline( (std::istream&) fs, line );

			if ( line.empty() ) {
				break;
			}

			split = boost::split( split, line, boost::is_any_of( ",\t" ) );

			switch( lineno ) {
			case 0:
				r.n_channels = split.size();
				r.channels = split;
				break;

			case 1:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					r.rx_center_freq[ r.channels[ i ] ] = calc.crunch( split[ i ] );
				}
				break;

			case 2:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					r.tx_center_freq[ r.channels[ i ] ] = calc.crunch( split[ i ] );
				}
				break;

			case 3:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					r.rx_sample_rate[ r.channels[ i ] ] = calc.crunch( split[ i ] );
				}
				break;

			case 4:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					r.tx_sample_rate[ r.channels[ i ] ] = calc.crunch( split[ i ] );
				}
				break;

			default:
				sampleno = lineno - 5;
				sampleno /= 2;

				r.n_samples = sampleno;

				for( unsigned i = 0; i < r.n_channels; i++ ) {

					if ( split[ i ].empty() ) {
						// when reading from a file, it is possible that some of the data series are not the same length as others
						// since it only makes sense to buffer 1 period of a periodic signal, and each channel can have a different period
						continue;
					}

					short shrt;
					try {
						shrt = boost::lexical_cast<short>( split[ i ] );
					} catch( const boost::bad_lexical_cast & e ) {
						continue;
					}
					if ( 0 == ( lineno - 5 ) % 2 ) {
						std::complex<short> c;
						c.real( shrt );
						r.samples[ r.channels[ i ] ][ sampleno ] = c;
					} else {
						r.samples[ r.channels[ i ] ][ sampleno ].imag( shrt );
					}
				}
				break;
			}
		}

		fs.close();

		if ( 0 == r.n_channels ) {
			throw uhd::runtime_error( "input file does not exist, or contains invalid data" );
		}
	}

	static void toFile( StreamData &r, std::string fn ) {

		int lineno;
		unsigned sampleno;

		std::ofstream fs( fn );

		for( lineno = 0; ; lineno++ ) {

			sampleno = lineno - 5;
			sampleno /= 2;

			if ( sampleno > r.n_samples ) {
				break;
			}

			switch( lineno ) {
			case 0:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					fs << r.channels[ i ];
					if ( i < r.n_channels - 1 ) {
						fs << preferred_delimiter;
					}
				}
				fs << std::endl;
				break;

			case 1:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					fs << r.rx_center_freq[ r.channels[ i ] ];
					if ( i < r.n_channels - 1 ) {
						fs << preferred_delimiter;
					}
				}
				fs << std::endl;
				break;

			case 2:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					fs << r.tx_center_freq[ r.channels[ i ] ];
					if ( i < r.n_channels - 1 ) {
						fs << preferred_delimiter;
					}
				}
				fs << std::endl;
				break;

			case 3:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					fs << r.rx_sample_rate[ r.channels[ i ] ];
					if ( i < r.n_channels - 1 ) {
						fs << preferred_delimiter;
					}
				}
				fs << std::endl;
				break;

			case 4:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					fs << r.tx_sample_rate[ r.channels[ i ] ];
					if ( i < r.n_channels - 1 ) {
						fs << preferred_delimiter;
					}
				}
				fs << std::endl;
				break;

			default:

				for( unsigned i = 0; i < r.n_channels; i++ ) {

					if ( sampleno < r.samples[ r.channels[ i ] ].size() ) {
						if ( 0 == (lineno - 5) % 2 ) {
							fs << r.samples[ r.channels[ i ] ][ sampleno ].real();
						} else {
							fs << r.samples[ r.channels[ i ] ][ sampleno ].imag();
						}
					}
					if ( i < r.n_channels - 1 ) {
						// when writing to a file, it is still necessary to write out a delimeter even when there is no data, to preserve CSV format
						fs << preferred_delimiter;
					}
				}

				fs << std::endl;

				break;
			}
		}

		fs.close();
	}

	void fillBuffers( std::vector<size_t> channel_nums, std::vector< std::vector< std::complex< short > > > &v, std::vector<size_t> &idx ) {

		if ( v.size() != idx.size() ) {
			throw uhd::runtime_error( "buffer vector and index vector are not equal sizes" );
		}

		for( unsigned i = 0; i < v.size(); i++ ) {

			const std::string &s = ((std::map<int,const std::string>)fro)[ channel_nums[ i ] ];

			std::map<size_t, std::complex<short>> &m = samples[ s ];
			std::vector<std::complex<short>> &vv = v[ i ];

			for( unsigned j = 0; j < v[ i ].size(); j++, idx[ i ] += 1, idx[ i ] %= m.size() ) {
				vv[ j ] =  m[ idx[ i ] ];
			}
		}
	}

	void sizeBuffers( std::vector< std::vector< std::complex< short > > > &v ) {
		for( unsigned i = 0; i < v.size(); i++ ) {
			v[ i ].resize( n_samples );
		}
	}
};

#include "wavetable.hpp"
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <csignal>

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

/***********************************************************************
 * transmit_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void transmit_worker(
	StreamData &sd,
    uhd::tx_streamer::sptr tx_streamer,
    uhd::tx_metadata_t metadata,
    std::vector<size_t> tx_channel_nums
){
	if ( sd.n_channels != tx_channel_nums.size() ) {\
		throw new uhd::runtime_error( "number of channels not consistent" );
	}

	std::vector<size_t> indeces( tx_channel_nums.size() );

	std::vector< std::vector< std::complex< short > > > buffs( tx_channel_nums.size() );

	sd.sizeBuffers( buffs );

    //send data until the signal handler gets called
    while(not stop_signal_called){

        sd.fillBuffers( tx_channel_nums, buffs, indeces );

        //send the entire contents of the buffer
        tx_streamer->send(buffs, sd.n_samples, metadata);

        metadata.start_of_burst = false;
        metadata.has_time_spec = false;
    }

    //send a mini EOB packet
    metadata.end_of_burst = true;
    tx_streamer->send("", 0, metadata);
}


void recv_to_stream_data(
    uhd::usrp::multi_usrp::sptr usrp,
    float settling_time,
    float maximum_rxtime_s,
    size_t maximum_rxsamples,
    StreamData &sd,
    std::vector<size_t> rx_channel_nums
){
	const std::string cpu_format = "sc16";
	const std::string wire_format = "sc16";

    int num_total_samps = 0;
    //create a receive streamer
    uhd::stream_args_t stream_args(cpu_format,wire_format);
    stream_args.channels = rx_channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    std::string chan_with_highest_sample_rate = "";
    double highest_sample_rate = -1;
    for( const auto &kv: sd.rx_sample_rate ) {
        if ( kv.second > highest_sample_rate ) {
            chan_with_highest_sample_rate = kv.first;
            highest_sample_rate = kv.second;
        }
    }

    sd.n_samples = std::min( (size_t)( maximum_rxtime_s * highest_sample_rate ), (size_t)maximum_rxsamples );

    int num_requested_samples = sd.n_samples;

    std::vector< std::vector< std::complex< short > > > buffs(
		rx_channel_nums.size(), std::vector< std::complex< short > >( sd.n_samples )
    );

    std::vector< std::complex< short > * > buff_ptrs;
    for( size_t i = 0; i < buffs.size(); i++ ) {
    	buff_ptrs.push_back( (std::complex< short > * const) & buffs[ i ].front() );
    }

    bool overflow_message = true;
    float timeout = settling_time + 0.1; //expected settling time + padding for first recv

    //setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)?
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
        uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
    );
    stream_cmd.num_samps = num_requested_samples;
    //stream_cmd.stream_now = false;
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t(settling_time);
    rx_stream->issue_stream_cmd(stream_cmd);

    uhd::rx_metadata_t md;

    while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0)){
        size_t num_rx_samps = rx_stream->recv( buff_ptrs, buffs.size(), md, timeout);
        timeout = 0.1; //small timeout for subsequent recv

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
            if (overflow_message){
                overflow_message = false;
                std::cerr << boost::format(
                    "Got an overflow indication. Please consider the following:\n"
                    "  Your write medium must sustain a rate of %fMB/s.\n"
                    "  Dropped samples will not be written to the file.\n"
                    "  Please modify this example for your purposes.\n"
                    "  This message will not appear again.\n"
                ) % (usrp->get_rx_rate()*sizeof( std::complex<short> )/1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Receiver error %s"
            ) % md.strerror()));
        }

        num_total_samps += num_rx_samps;

    }


    for( size_t i = 0; i < buffs.size(); i++ ) {
    	std::vector< std::complex<short> > &v = buffs[ i ];
    	const std::string chan = fro[ rx_channel_nums[ i ] ];
    	std::map< size_t, std::complex<short> > &m = sd.samples[ chan ];
    	for( size_t j = 0; j < v.size(); j++ ) {
    		m[ j ] = v[ j ];
    	}
    }
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //transmit variables to be set by po
    std::string tx_args, wave_type, tx_ant, tx_subdev, ref, otw, tx_channels;
    double tx_rate, tx_freq;

    //receive variables to be set by po
    std::string rx_args, input_fn, output_fn, type, rx_ant, rx_subdev, rx_channels;
    double rx_rate, rx_freq;
    float settling;
    float rxtime;
    size_t rxsamp;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("tx-args", po::value<std::string>(&tx_args)->default_value(""), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value(""), "uhd receive device address args")
        ("input", po::value<std::string>(&input_fn)->default_value("input.csv"), "name of the input file")
		("output", po::value<std::string>(&output_fn)->default_value("output.csv"), "name of the output file")
        ("settling", po::value<float>(&settling)->default_value(1), "settling time (seconds) before receiving")
        ("rxtime", po::value<float>(&rxtime)->default_value(1), "maximum receive time (seconds)")
        ("rxsamp", po::value<size_t>(&rxsamp)->default_value((1 << 25)/4), "maximum number of received samples")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if ( rxtime < 0 ) {
        std::cerr << boost::format("rxtime %f invalid") % rxtime << std::endl;
        return ~0;
    }

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("Crimson TXRX Loopback to / from File %s") % desc << std::endl;
        return ~0;
    }

    StreamData input_stream_data;
    StreamData output_stream_data;

    input_fn = vm[ "input" ].as<std::string>();
    StreamData::fromFile( input_stream_data, input_fn );

    output_stream_data.n_channels = input_stream_data.n_channels;
    output_stream_data.channels = input_stream_data.channels;

    //create a usrp device
    std::cout << boost::format("Creating the transmit crimson device with: %s...") % tx_args << std::endl;
    uhd::usrp::multi_usrp::sptr tx_usrp = uhd::usrp::multi_usrp::make(tx_args);

    std::cout << boost::format("Creating the receive usrp device with: %s...") % rx_args << std::endl;
    uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);

    //detect which channels to use
    std::vector<size_t> tx_channel_nums;
    for( std::string &n: input_stream_data.channels ) {
        tx_channel_nums.push_back( to[ n ] );
    }

    // XXX: @CF: this will not always be the case. In the CSV file,
    // if rx is not in use when the corresponding tx is in use (and vice versa),
    // I suggest we encode it as a negative centre frequency
    std::vector<size_t> rx_channel_nums;
    for( std::string &n: input_stream_data.channels ) {
        rx_channel_nums.push_back( to[ n ] );
    }

    std::cout << boost::format("Using Device: %s") % tx_usrp->get_pp_string() << std::endl;
    std::cout << boost::format("Using Device: %s") % rx_usrp->get_pp_string() << std::endl;

    //set the transmit sample rate
    for( const auto& kv: input_stream_data.tx_sample_rate ) {
		std::string chan = kv.first;
		tx_rate = kv.second;
		std::cout << boost::format("Setting TX Rate for Channel %s: %f Msps...") % chan % (tx_rate/1e6) << std::endl;
	    tx_usrp->set_tx_rate(tx_rate);

	    output_stream_data.tx_sample_rate[ chan ] = tx_usrp->get_tx_rate();
	    std::cout << boost::format("Actual TX Rate: %f Msps...") % ( output_stream_data.tx_sample_rate[ chan ] / 1e6 ) << std::endl;
    }

    //set the receive sample rate
    for( const auto& kv: input_stream_data.rx_sample_rate ) {
		std::string chan = kv.first;
		rx_rate = kv.second;
		std::cout << boost::format("Setting RX Rate for Channel %s: %f Msps...") % chan % (rx_rate/1e6) << std::endl;
	    rx_usrp->set_rx_rate(rx_rate);

	    output_stream_data.rx_sample_rate[ chan ] = tx_usrp->get_rx_rate();
	    std::cout << boost::format("Actual RX Rate: %f Msps...") % ( output_stream_data.rx_sample_rate[ chan ] / 1e6 ) << std::endl;
    }

    for( const auto& kv: input_stream_data.tx_center_freq ) {

        //set the transmit center frequency
		const std::string& chan = kv.first;
		tx_freq = kv.second;
    	std::cout << boost::format("Setting TX Freq for Channel %s: %f MHz...") % chan % (tx_freq/1e6) << std::endl;
        uhd::tune_request_t tx_tune_request(tx_freq);
        int chani = to[ chan ];
        tx_usrp->set_tx_freq( tx_tune_request, chani );

        output_stream_data.tx_center_freq[ chan ] = tx_usrp->get_tx_freq();
        std::cout << boost::format("Actual TX Freq: %f MHz...") % ( output_stream_data.tx_center_freq[ chan ] / 1e6 ) << std::endl;

    	//set the rf gain
        //std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
    	//tx_usrp->set_tx_gain(tx_gain, tx_channel_nums[ch]);
    	//std::cout << boost::format("Actual TX Gain: %f dB...") % tx_usrp->get_tx_gain(tx_channel_nums[ch]) << std::endl << std::endl;
    }


    for( const auto& kv: input_stream_data.rx_center_freq ) {

        //set the transmit center frequency
		std::string chan = kv.first;
		rx_freq = kv.second;
    	std::cout << boost::format("Setting RX Freq for Channel %s: %f MHz...") % chan % (rx_freq/1e6) << std::endl;
        uhd::tune_request_t rx_tune_request(rx_freq);
        int chani = to[ chan ];
        rx_usrp->set_rx_freq(rx_tune_request, chani );

        output_stream_data.rx_center_freq[ chan ] = rx_usrp->get_rx_freq();
        std::cout << boost::format("Actual RX Freq: %f MHz...") % ( output_stream_data.rx_center_freq[ chan ] / 1e6 ) << std::endl << std::endl;

    	//set the rf gain
        //std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
    	//rx_usrp->set_rx_gain(rx_gain, rx_channel_nums[ch]);
    	//std::cout << boost::format("Actual RX Gain: %f dB...") % rx_usrp->get_rx_gain(rx_channel_nums[ch]) << std::endl << std::endl;
    }

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args( "sc16", "sc16" );
    stream_args.channels = tx_channel_nums;
    uhd::tx_streamer::sptr tx_stream = tx_usrp->get_tx_stream( stream_args );

    //setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(0.1); //give us 0.1 seconds to fill the tx buffers

    //reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    tx_usrp->set_time_now(uhd::time_spec_t(0.0));

    //start transmit worker thread
    boost::thread_group transmit_thread;
    transmit_thread.create_thread( boost::bind( &transmit_worker, input_stream_data, tx_stream, md, tx_channel_nums ) );

    //recv to file
    recv_to_stream_data( rx_usrp, settling, rxtime, rxsamp, output_stream_data, rx_channel_nums );

    //clean up transmit worker
    stop_signal_called = true;
    transmit_thread.join_all();

    StreamData::toFile( output_stream_data, output_fn );

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
