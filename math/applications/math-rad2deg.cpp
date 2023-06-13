// Copyright (c) 2011 The University of Sydney

#include <cmath>
#include <iostream>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "convert angles in degrees to radians" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    math-rad2deg <angle> [--precision=<n>]" << std::endl;
    std::cerr << "    cat values.csv | math-rad2deg --fields=,,,alpha,beta" << std::endl;
    std::cerr << "    cat values.bin | math-rad2deg --fields=,b,a --binary=t,2d,ui" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

struct Line
{
    enum { size = 1024 };
    boost::array< double, size > values;
};

static const long double ratio = ( long double )( 180 ) / M_PI;

namespace comma { namespace visiting {

template <> struct traits< Line >
{
    template < typename K, typename V > static void visit( const K&, Line& p, V& v ) { v.apply( "values", p.values ); }
    template < typename K, typename V > static void visit( const K&, const Line& p, V& v ) { v.apply( "values", p.values ); }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        std::vector< std::string > unnamed = options.unnamed( "--flush", "--delimiter,-d,--binary,-b,--fields,-f,--precision" );
        comma::csv::options csv( options );
        if( unnamed.empty() )
        {
            csv.full_xpath = false;
            if( csv.fields == "" ) { std::cerr << "math-rad2deg: please specify --fields (todo: handle default)" << std::endl; return 1; }
            std::vector< std::string > v = comma::split( csv.fields, ',' );
            if( v.size() > Line::size ) { std::cerr << "math-rad2deg: expected not more than 1024 --fields (todo: handle arbitrary number of fields)" << std::endl; return 1; }
            unsigned int count = 0; // quick and dirty
            for( std::size_t i = 0; i < v.size(); ++i )
            {
                if( !v[i].empty() ) { v[i] = "values[" + boost::lexical_cast< std::string >( count++ ) + "]"; }
            }
            csv.fields = comma::join( v, ',' );
            comma::csv::input_stream< Line > istream( std::cin, csv );
            comma::csv::output_stream< Line > ostream( std::cout, csv );
            while( std::cin.good() && !std::cin.eof() )
            {
                const Line* input = istream.read();
                if( !input ) { break; }
                Line output = *input; // quick and dirty
                for( unsigned int i = 0; i < count; output.values[i] = ratio * output.values[i], ++i );
                ostream.write( output, istream );
            }
        }
        else
        {
            std::cout.precision( csv.precision );
            std::cout << ( ratio * boost::lexical_cast< double >( unnamed[0] ) ) << std::endl;
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "math-rad2deg: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "math-rad2deg: unknown exception" << std::endl; }
    return 1;
}
