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

#include <map>
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

// 32 [ MB ] / 4 [ B / Sample ] = 8 [ M Sample ]
#define DEFAULT_SAMPLE_LIMIT ( 1 * 1024 * 1024 / sizeof( std::complex< short > ) )

static std::map<const std::string,int> to = { { "A", 0 }, { "B", 1 }, { "C", 2 }, { "D", 3 } };
static std::map<int,const std::string> fro = { { 0, "A" }, { 1, "B" }, { 2, "C" }, { 3, "D" } };

/*
// XXX: @CF: kb #3773

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

*/

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

		/*
		 // XXX: @CF: kb #3773
		symtab_t variables;
		calculator calc( variables );
		variables["pi"] = 3.141592653589792;
		*/

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
					// XXX: @CF: kb #3773
					//r.rx_center_freq[ r.channels[ i ] ] = calc.crunch( split[ i ] );
					r.rx_center_freq[ r.channels[ i ] ] = stod( split[ i ] );
				}
				break;

			case 2:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					// XXX: @CF: kb #3773
					//r.tx_center_freq[ r.channels[ i ] ] = calc.crunch( split[ i ] );
					r.tx_center_freq[ r.channels[ i ] ] = stod( split[ i ] );
				}
				break;

			case 3:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					// XXX: @CF: kb #3773
					//r.rx_sample_rate[ r.channels[ i ] ] = calc.crunch( split[ i ] );
					r.rx_sample_rate[ r.channels[ i ] ] = stod( split[ i ] );
				}
				break;

			case 4:
				for( unsigned i = 0; i < r.n_channels; i++ ) {
					// XXX: @CF: kb #3773
					//r.tx_sample_rate[ r.channels[ i ] ] = calc.crunch( split[ i ] );
					r.tx_sample_rate[ r.channels[ i ] ] = stod( split[ i ] );
				}
				break;

			default:
				sampleno = lineno - 5;
				sampleno /= 2;

				r.n_samples = sampleno + 1;

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

	static void toFile( StreamData &r, std::string &fn ) {

		int lineno;
		int sampleno;

		std::ofstream fs( fn );

		for( lineno = 0; ; lineno++ ) {

			sampleno = lineno - 5;
			sampleno /= 2;

			if ( sampleno > (int)r.n_samples ) {
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
//
//				if ( 0 == sampleno % 100 ) {
//					if ( 0 != sampleno ) {
//						std::cout << '\r';
//						for( int i = 0; i < 80; i++ ) {
//							//std::cout << '\b';
//							std::cout << ' ';
//						}
//						std::cout << '\r';
//						std::cout << std::flush;
//					}
//
//					std::cout << "Writing sample " << sampleno << " / " << r.n_samples << "( " << (int)( 100.0 * (float)sampleno / (float)r.n_samples) << "% )";
//					std::cout << std::flush;
//				}

				for( unsigned i = 0; i < r.n_channels; i++ ) {

					if ( sampleno < (int) r.samples[ r.channels[ i ] ].size() ) {
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

		std::cout << std::endl;
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
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //transmit variables to be set by po
    std::string tx_args, tx_channels;
    double tx_rate, tx_freq, tx_gain;
    bool loop;
    double sob;

    //receive variables to be set by po
    std::string input_fn;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("tx-args", po::value<std::string>(&tx_args)->default_value(""), "uhd transmit device address args")
        ("input", po::value<std::string>(&input_fn)->default_value("input.csv"), "name of the input file")
		("loop", po::value<bool>(&loop)->default_value( false ), "retransmit the signal in a loop")
		("sob", po::value<double>(&sob)->default_value( 3 ), "delay transmission for sob seconds")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("Crimson TXRX Loopback to / from File %s") % desc << std::endl;
        return ~0;
    }

    StreamData input_stream_data;

    input_fn = vm[ "input" ].as<std::string>();
    StreamData::fromFile( input_stream_data, input_fn );

    //create a usrp device
    uhd::usrp::multi_usrp::sptr tx_usrp = uhd::usrp::multi_usrp::make(tx_args);

    //detect which channels to use
    std::vector<size_t> tx_channel_nums;
    for( std::string &n: input_stream_data.channels ) {
        tx_channel_nums.push_back( to[ n ] );
    }

	if ( input_stream_data.n_channels != tx_channel_nums.size() ) {
		throw new uhd::runtime_error( "number of channels not consistent" );
	}

    //set the transmit center frequency
    for( const auto& kv: input_stream_data.tx_center_freq ) {

    	std::string chan = kv.first;
		int chani = to[ chan ];

		tx_freq = kv.second;
		std::cout << boost::format("Setting TX Freq for Channel %s: %f MHz...") % chan % (tx_freq/1e6) << std::endl;
        uhd::tune_request_t tx_tune_request(tx_freq);
        tx_usrp->set_tx_freq( tx_tune_request, chani );

        tx_freq = tx_usrp->get_tx_freq( chani );
        std::cout << boost::format("Actual TX Freq: %f MHz...") % ( tx_freq / 1e6 ) << std::endl;
    }

    //set the transmit sample rate
    for( const auto& kv: input_stream_data.tx_sample_rate ) {
		std::string chan = kv.first;
		int chani = to[ chan ];

		tx_rate = kv.second;
		std::cout << boost::format("Setting TX Rate for Channel %s: %f Msps...") % chan % (tx_rate/1e6) << std::endl;
	    tx_usrp->set_tx_rate( tx_rate, chani );

	    tx_rate = tx_usrp->get_tx_rate();
	    std::cout << boost::format("Actual TX Rate: %f Msps...") % ( tx_rate / 1e6 ) << std::endl;
    }

    //set the transmit gain
    for( const auto& kv: input_stream_data.tx_center_freq ) {

    	std::string chan = kv.first;
		int chani = to[ chan ];

        tx_gain = 20.0;
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
    	tx_usrp->set_tx_gain( tx_gain, chani );
    	tx_gain = tx_usrp->get_tx_gain( chani );
    	std::cout << boost::format("Actual TX Gain: %f dB...") % tx_gain << std::endl << std::endl;
    }

    //create a transmit streamer
    uhd::stream_args_t stream_args( "sc16", "sc16" );
    stream_args.channels = tx_channel_nums;
    std::cout << "Getting TX Streamer.." << std::endl;
    uhd::tx_streamer::sptr tx_stream = tx_usrp->get_tx_stream( stream_args );
    std::cout << "Got TX Streamer.." << std::endl;

	std::signal(SIGINT, &sig_int_handler);
	std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

	std::vector<size_t> indeces( tx_channel_nums.size() );

	std::vector< std::vector< std::complex< short > > > buffs( tx_channel_nums.size() );

	input_stream_data.sizeBuffers( buffs );

    std::vector< std::complex< short > * > buff_ptrs;
    for( unsigned i = 0; i < buffs.size(); i++ ) {
    	buff_ptrs.push_back( (std::complex<short int> * const &) & buffs[ i ].front() );
    }

    input_stream_data.fillBuffers( tx_channel_nums, buffs, indeces );

    //setup the metadata flags
    uhd::tx_metadata_t md;

    //send data until the signal handler gets called
    for( ;; ) {

        md.start_of_burst = true;
        md.end_of_burst   = false;
        if ( sob > 0 ) {
    		md.has_time_spec = true;

    		uhd::time_spec_t now = tx_usrp->get_time_now();
    		uhd::time_spec_t then = now + sob;

    		std::cout << "Now: " << std::setprecision(10) << now.get_real_secs() << std::endl;
    		std::cout << "SoB: " << std::setprecision(10) << then.get_real_secs() << std::endl;

    		md.time_spec = then;
        } else {
        	sob = 0;
        	md.has_time_spec = false;
        }

        std::cout << "Sending " << input_stream_data.n_samples << " samples, " << sob << " s from now" << std::endl;

        //send the entire contents of the buffer
        tx_stream->send( buff_ptrs, input_stream_data.n_samples, md );

//        if ( 0 == sob ) {
//            std::cout << "Sending " << input_stream_data.n_samples << " samples, " << sob << " s from now" << std::endl;
//			md.start_of_burst = false;
//			md.has_time_spec = false;
//        }

        if ( ! loop ) {
        	break;
        }

        if ( stop_signal_called ) {
        	break;
        }
    }

    std::cout << "sending EOB" << std::endl;

    //send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send("", 0, md);

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
