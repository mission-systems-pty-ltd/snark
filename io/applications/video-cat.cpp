// Copyright (c) 2024 Vsevolod Vlaskine

#include <comma/application/command_line_options.h>
#include "../video.h"

void usage( bool )
{
    std::cerr << R"(
usage: video-cat <path> <options>

options
    --height=<rows>
    --width=<bytes>

todo...

)";
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        return 0;
    }
    catch( const std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}