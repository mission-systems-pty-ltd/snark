// Copyright (c) 2025 Vsevolod Vlaskine

#include <cstring>
#include <iostream>
#include <vector>
#include <boost/optional.hpp>
#include <fftw3.h>
#include <comma/visiting/traits.h>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "fft/basic.h"

static void usage( const comma::command_line_options& options )
{
    bool basic = options.exists( "--basic" ); // quick and dirty for reasonable backward compatibility
    bool verbose = options.exists( "--verbose,-v" );
    std::cerr << "basic usage (if --basic)" << std::endl;
    std::cerr << snark::math::applications::fft::basic::usage( basic ) << std::endl;
    std::cerr << std::endl;
    if( !basic )
    {
        std::cerr << R"(
options
    in progress... use --basic for now
)" << std::endl;
    }
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage( options ); }
        if( options.exists( "--basic" ) ) { return snark::math::applications::fft::basic::run( options ); }
        COMMA_THROW_BRIEF( comma::exception, "please specify --basic; extended functionality: in progress..." );
    }
    catch( std::exception& ex ) { comma::say() << ": " << ex.what() << std::endl; }
    catch( ... ) { comma::say() << ": " << "unknown exception" << std::endl; }
    return 1;
}
