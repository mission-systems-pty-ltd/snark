// Copyright (c) 2019 The University of Sydney

#include "../packet.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/csv/format.h>
#include <comma/csv/names.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <boost/optional.hpp>

static void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --output-fields --output-format"
        " --columns"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

const comma::uint16 default_columns_per_frame = 1024;

void usage( bool verbose )
{
    std::cerr << "\nalign Ouster data records for viewing as image";
    std::cerr << "\n";
    std::cerr << "\nusage: cat <ouster-csv> | ouster-align";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:        display this help message and exit";
    std::cerr << "\n    --verbose,-v:     more output";
    std::cerr << "\n    --output-fields:  list output fields and exit";
    std::cerr << "\n    --output-format:  list output format and exit";
    std::cerr << "\n    --columns=<num>:  columns per frame; default " << default_columns_per_frame;
    std::cerr << "\n";
    std::cerr << "\ndescription:";
    std::cerr << "\n";
    std::cerr << "\n    Each of the 64 channels is offset from each other in a regular way.";
    std::cerr << "\n    We can realign the data so that points physically aligned vertically";
    std::cerr << "\n    are also aligned in the data. That makes it easier to view.";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\n    The beam offsets are something like:";
        std::cerr << "\n    a b c d e f g";
        std::cerr << "\n                a b c d e f g";
        std::cerr << "\n                            a b c d e f g";
        std::cerr << "\n                                        a b c d e f g";
        std::cerr << "\n    a b c d e f g";
        std::cerr << "\n    ...";
        std::cerr << "\n    where \"a\" (etc) appears in the data in the same place.";
        std::cerr << "\n";
        std::cerr << "\n    We need to line the data up so that a pixel which is physically aligned is";
        std::cerr << "\n    also aligned in the data. We do this by taking a few data points from the";
        std::cerr << "\n    start or the end and putting them in the right place, so each channel,";
        std::cerr << "\n    after correction for the offsets (which ouster-to-csv provides), starts at";
        std::cerr << "\n    0 and runs to 2Π.";
    }
    else
    {
        std::cerr << "\n    see ouster-align --help --verbose for much more detail";
    }
    std::cerr << "\n";
    std::cerr << "\nexample:";
    std::cerr << "\n    cat *.bin | ouster-to-csv lidar | ouster-align \\";
    std::cerr << "\n        | csv-shuffle --fields $( ouster-align --output-fields ) \\";
    std::cerr << "\n                      --binary $( ouster-align --output-format ) \\";
    std::cerr << "\n                      --output signal \\";
    std::cerr << "\n        | cv-cat --input=\"rows=64;cols=1024;no-header;type=CV_16UC1\" \\";
    std::cerr << "\n                \"scale=60;resize=1.0,2.0;view;null\"";
    std::cerr << "\n";
    std::cerr << "\nsee also:";
    std::cerr << "\n    ouster-to-csv";
    std::cerr << "\n" << std::endl;
}

std::string output_fields() { return comma::join( comma::csv::names< snark::ouster::lidar::output_lidar_t >( false ), ',' ); }
std::string output_format() { return comma::csv::format::value< snark::ouster::lidar::output_lidar_t >(); }

struct channel_data
{
    std::vector< snark::ouster::lidar::output_lidar_t > front;
    std::vector< snark::ouster::lidar::output_lidar_t > middle;
    std::vector< snark::ouster::lidar::output_lidar_t > back;
};

static std::array< channel_data, snark::ouster::lidar::v1::pixels_per_column > channels;
static comma::uint32 records_per_frame;
static comma::uint32 records_added = 0;

void output_data( comma::csv::binary_output_stream< snark::ouster::lidar::output_lidar_t >& os )
{
    // only output complete frames
    if( records_added == records_per_frame )
    {
        for( channel_data& ch : channels )
        {
            for( const snark::ouster::lidar::output_lidar_t& out : ch.front ) { os.write( out ); }
            for( const snark::ouster::lidar::output_lidar_t& out : ch.middle ) { os.write( out ); }
            for( const snark::ouster::lidar::output_lidar_t& out : ch.back ) { os.write( out ); }
        }
    }
    for( channel_data& ch : channels )
    {
        ch.front.clear();
        ch.middle.clear();
        ch.back.clear();
    }
    records_added = 0;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );

        if( options.exists( "--output-fields" ) ) { std::cout << output_fields() << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << output_format() << std::endl; return 0; }

        records_per_frame = options.value< comma::uint32 >( "--columns", default_columns_per_frame ) * snark::ouster::lidar::v1::pixels_per_column;

        // input and output format are the same
        comma::csv::options csv( options, output_fields(), false );
        csv.format( output_format() );
        comma::csv::binary_input_stream< snark::ouster::lidar::output_lidar_t > is( std::cin, csv );

        comma::csv::options output_csv( csv );
        comma::csv::binary_output_stream< snark::ouster::lidar::output_lidar_t > os( std::cout, output_csv );

        comma::uint32 block_id = 0;

        while( is.ready() || ( std::cin.good() && !std::cin.eof() ))
        {
            const snark::ouster::lidar::output_lidar_t* record = is.read();
            if( !record ) { break; }
            if( record->azimuth_block.block_id != block_id )
            {
                output_data( os );
                block_id = record->azimuth_block.block_id;
            }
            double bearing = record->data_block.bearing;
            channel_data& channel = channels[ record->data_block.channel ];
            if( bearing < 0 )              { channel.front.push_back( *record ); }
            else if( bearing >= 2 * M_PI ) { channel.back.push_back( *record ); }
            else                           { channel.middle.push_back( *record ); }
            records_added++;
        }
        output_data( os );
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "ouster-align: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "ouster-align: unknown exception" << std::endl; }
    return 1;
}
