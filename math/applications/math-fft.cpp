// Copyright (c) 2025 Vsevolod Vlaskine

#include <cstring>
#include <iostream>
#include <vector>
#include <boost/optional.hpp>
#include <fftw3.h>
#include <comma/visiting/traits.h>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "fft/basic.h"

void usage( bool verbose )
{
    std::cerr << snark::math::applications::fft::basic::usage( verbose ) << std::endl;
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        return snark::math::applications::fft::basic::run( options );
    }
    catch( std::exception& ex ) { comma::say() << ": " << ex.what() << std::endl; }
    catch( ... ) { comma::say() << ": " << "unknown exception" << std::endl; }
    return 1;
}
