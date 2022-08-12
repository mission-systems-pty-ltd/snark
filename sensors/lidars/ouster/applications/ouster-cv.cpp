// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020 Mission Systems Pty Ltd

#include "../packet.h"
#include "../traits.h"
#include "../../../../imaging/cv_mat/serialization.h"
#include <comma/application/command_line_options.h>
#include <comma/csv/format.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>

static void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --columns --output --output-images --output-fields --output-format"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

const comma::uint16 default_columns_per_frame = 1024;
const std::string default_fields( comma::join( comma::csv::names< snark::ouster::lidar::output_lidar_t >( false ), ',' ));
const std::string default_output_images( "signal,reflectivity,ambient" );

void usage( bool verbose )
{
    std::cerr << "\noutput Ouster image data in cv format";
    std::cerr << "\n";
    std::cerr << "\nusage: cat <aligned ouster> | ouster-cv";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:          display this help message and exit";
    std::cerr << "\n    --verbose,-v:       more output";
    std::cerr << "\n    --columns=<num>:    columns per frame; default " << default_columns_per_frame;
    std::cerr << "\n    --output=<images>:  images to output; default " << default_output_images;
    std::cerr << "\n    --output-fields:    print output fields and exit";
    std::cerr << "\n    --output-format:    print output format and exit";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\ncsv options:";
        std::cerr << "\n";
        std::cerr << comma::csv::options::usage( default_fields );
    }
    std::cerr << "\n    if multiple images are output, the image data is vertically stacked";
    std::cerr << "\n";
    std::cerr << "\nexample:";
    std::cerr << "\n    cat *.bin | ouster-to-csv lidar | ouster-align \\";
    std::cerr << "\n        | ouster-cv --output signal,ambient \\";
    std::cerr << "\n        | cv-cat \"scale=60;resize=1.0,2.0;view;null\"";
    std::cerr << "\n";
    std::cerr << "\nsee also:";
    std::cerr << "\n    ouster-to-csv  ouster-align";
    std::cerr << "\n" << std::endl;
}

struct buffer_t
{
    std::vector< comma::uint16 > signal;
    std::vector< comma::uint16 > reflectivity;
    std::vector< comma::uint16 > ambient;
};

static std::vector< std::string > output_images;
static comma::uint32 records_per_frame;
static buffer_t buffer;
static boost::scoped_ptr< comma::csv::binary_output_stream< snark::cv_mat::serialization::header > > header_os;
static snark::cv_mat::serialization::header cv_mat_header;

void output_data( const boost::posix_time::ptime& timestamp )
{
    // only output complete frames
    if( buffer.signal.size() == records_per_frame )
    {
        cv_mat_header.timestamp = timestamp;
        header_os->write( cv_mat_header );

        // each image field is by necessity the same size and type
        size_t buffer_size = buffer.signal.size() * sizeof( comma::uint16 );

        for( const std::string& image : output_images )
        {
            if( image == "signal" ) { std::cout.write( (const char*)&buffer.signal[0], buffer_size ); }
            if( image == "reflectivity" ) { std::cout.write( (const char*)&buffer.reflectivity[0], buffer_size ); }
            if( image == "ambient" ) { std::cout.write( (const char*)&buffer.ambient[0], buffer_size ); }
        }
        std::cout.flush();
    }
    buffer.signal.clear();
    buffer.reflectivity.clear();
    buffer.ambient.clear();
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" )) bash_completion( ac, av );

        std::string output_images_str = options.value< std::string >( "--output", default_output_images );
        if( options.exists( "--output-images" )) { std::cout << output_images_str << std::endl; return 0; }

        output_images = comma::split( output_images_str, ',' );
        comma::uint32 columns = options.value< comma::uint32 >( "--columns", default_columns_per_frame );
        records_per_frame = columns * snark::ouster::lidar::v1::pixels_per_column;

        comma::csv::options csv( options, default_fields, false );
        csv.format( comma::csv::format::value< snark::ouster::lidar::output_lidar_t >() );

        // get the format for the output images, so we can use it in the OpenCV header
        comma::csv::format output_format( comma::csv::format::value< snark::ouster::lidar::output_lidar_t >( output_images_str, false ));
        const std::vector< comma::csv::format::element >& elements( output_format.elements() );
        if( find_if_not( std::begin( elements )
                       , std::end( elements )
                       , [elements]( const comma::csv::format::element& m ) ->
                             bool { return m.type == elements[0].type; })
                != std::end( output_format.elements() ))
        {
            COMMA_THROW( comma::exception, "output images must all have the same format - given images have format: " << output_format.string() );
        }
        std::string image_format = comma::csv::format::value< snark::ouster::lidar::output_lidar_t >( output_images[0], false );

        comma::csv::binary_input_stream< snark::ouster::lidar::output_lidar_t > is( std::cin, csv );

        comma::csv::options header_csv;
        header_csv.fields = "t,rows,cols,type";
        header_csv.format( "t,3ui" );
        header_os.reset( new comma::csv::binary_output_stream< snark::cv_mat::serialization::header >( std::cout, header_csv ));

        cv_mat_header.type = snark::cv_mat::type_from_string( image_format );
        cv_mat_header.rows = snark::ouster::lidar::v1::pixels_per_column * output_images.size();
        cv_mat_header.cols = columns;

        if( options.exists( "--output-fields" )) { std::cout << header_csv.fields << ",data" << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << header_csv.format().string() << "," << cv_mat_header.rows * cv_mat_header.cols << image_format << std::endl; return 0; }

        comma::uint32 block_id = 0;

        const snark::ouster::lidar::output_lidar_t* record = nullptr;
        while( is.ready() || ( std::cin.good() && !std::cin.eof() ))
        {
            record = is.read();
            if( !record ) { break; }
            if( record->azimuth_block.block_id != block_id )
            {
                output_data( record->azimuth_block.t );
                block_id = record->azimuth_block.block_id;
            }
            buffer.signal.push_back( record->data_block.signal );
            buffer.reflectivity.push_back( record->data_block.reflectivity );
            buffer.ambient.push_back( record->data_block.ambient );
        }
        if( record ) { output_data( record->azimuth_block.t ); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "ouster-cv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "ouster-cv: unknown exception" << std::endl; }
    return 1;
}
