// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020 Mission Systems Pty Ltd

#include "../config.h"
#include "../packet.h"
#include "../traits.h"
#include "../types.h"
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

static const std::string default_config( "config.json:ouster" );

static void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --config --output-fields --output-format --output-frame"
        " lidar imu"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

void usage( bool verbose )
{
    std::cerr << "\nConvert raw Ouster OS1 lidar data";
    std::cerr << "\n";
    std::cerr << "\nUsage: cat <raw-data> | ouster-to-csv <lidar|imu> [<options>]";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:             display this help message and exit";
    std::cerr << "\n    --verbose,-v:          more output";
    std::cerr << "\n    --config=<file:path>:  default: " << default_config;
    std::cerr << "\n    --output-fields:       list output fields and exit";
    std::cerr << "\n    --output-format:       list output format and exit";
    std::cerr << "\n    --output-frame:        output frame offset as x,y,z,r,p,y";
    std::cerr << "\n";
    std::cerr << "\n    For any particular device the config can be generated by";
    std::cerr << "\n    ouster-cat config --device <address>";
    std::cerr << "\n";
    std::cerr << "\nUnits:";
    std::cerr << "\n    Raw Ouster data is converted to regular SI units. In particular,";
    std::cerr << "\n    timestamps are ISO, range is in metres, and angular acceleration is rad/s.";
    std::cerr << "\n";
    std::cerr << "\nSupported devices:";
    std::cerr << "\n    This driver has been tested on the OS1-64 and OS1-16. Note that the 16 beam";
    std::cerr << "\n    sensor still returns 64 channels of data, however three out of four have";
    std::cerr << "\n    zero signal. This driver filters out those channels.";
    std::cerr << "\n";
    std::cerr << "\nCoordinate frames:";
    std::cerr << "\n    The Software User Guide in §4.1 Sensor Coordinate Frame describes the sensor";
    std::cerr << "\n    frame as x forward, y left and z up; with the connector attached to -ve x.";
    std::cerr << "\n    This driver follows that convention.";
    std::cerr << "\n";
    std::cerr << "\n    The Software User Guide in §4.4 describes transforms to map from lidar frame";
    std::cerr << "\n    to sensor frame. These include a rotation to the x forward convention";
    std::cerr << "\n    described above and a translation from internal sensor position to the";
    std::cerr << "\n    centre of the base plate.";
    std::cerr << "\n";
    std::cerr << "\n    The exact geometry of these transforms is contained in the sensor";
    std::cerr << "\n    configuration as generated by ouster-cat config. This configuration";
    std::cerr << "\n    is read by this driver and the intrinsic transforms are applied to the";
    std::cerr << "\n    output cartesian coordinates.";
    std::cerr << "\n";
    std::cerr << "\n    To see the intrinsic transforms in x,y,z,roll,pitch,yaw format run";
    std::cerr << "\n    ouster-to-csv <lidar|imu> --output-frame --config <config_file>";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\nExamples:";
        std::cerr << "\n    --- save fields and format";
        std::cerr << "\n    fields=$( ouster-to-csv lidar --output-fields )";
        std::cerr << "\n    format=$( ouster-to-csv lidar --output-format )";
        std::cerr << "\n";
        std::cerr << "\n    --- view live data ---";
        std::cerr << "\n    ouster-cat lidar | ouster-to-csv lidar \\";
        std::cerr << "\n        | view-points --fields $fields --binary $format --z-is-up";
        std::cerr << "\n";
        std::cerr << "\n    --- point cloud ---";
        std::cerr << "\n    cat *.bin | ouster-to-csv lidar \\";
        std::cerr << "\n        | csv-play --binary $format \\";
        std::cerr << "\n        | view-points --fields $fields --binary $format --z-is-up";
        std::cerr << "\n";
        std::cerr << "\n    --- image ---";
        std::cerr << "\n    data_field=ambient   # or signal or reflectivity";
        std::cerr << "\n    cat *.bin | ouster-to-csv lidar \\";
        std::cerr << "\n        | csv-select --fields=block --binary=$format --greater=0 --sorted \\";
        std::cerr << "\n        | csv-eval --fields=$fields --binary=$format \"bearing=bearing%(2*pi)\" \\";
        std::cerr << "\n        | csv-sort --fields=$fields --binary=$format --order=elevation,bearing \\";
        std::cerr << "\n        | csv-shuffle --fields $fields --binary $format --output $data_field \\";
        std::cerr << "\n        | cv-cat --input=\"rows=64;cols=1024;no-header;type=CV_16UC1\" \\";
        std::cerr << "\n                 \"flip;brightness=60;resize=1.0,2.0;view;null\"";
        std::cerr << "\n";
        std::cerr << "\n    --- live imu ---";
        std::cerr << "\n    fields=$( ouster-to-csv imu --output-fields )";
        std::cerr << "\n    format=$( ouster-to-csv imu --output-format )";
        std::cerr << "\n    ouster-cat imu | ouster-to-csv imu";
        std::cerr << "\n";
        std::cerr << "\n    --- display imu data ---";
        std::cerr << "\n    ouster-cat imu | ouster-to-csv imu \\";
        std::cerr << "\n        | csv-shuffle --fields $fields --binary $format --output \\";
        std::cerr << "\n              acceleration/t,acceleration/data/x,acceleration/data/y,acceleration/data/z \\";
        std::cerr << "\n        | csv-plot --binary ul,3d \"-;fields=x,y;color=red\" \\";
        std::cerr << "\n                   \"-;fields=x,,y;color=green\" \"-;fields=x,,,y;color=blue\"";
    }
    else
    {
        std::cerr << "\n";
        std::cerr << "For examples of use try: ouster-to-csv --help --verbose";
    }
    std::cerr << "\n";
    std::cerr << "\nSee also:";
    std::cerr << "\n    ouster-cat";
    std::cerr << "\n" << std::endl;
}

struct intrinsics_t
{
    snark::ouster::transform_t imu_transform;
    snark::ouster::transform_t lidar_transform;

    intrinsics_t() {}

    intrinsics_t( snark::ouster::OS1::config_t& config )
        : imu_transform( config.imu_intrinsics.imu_to_sensor_transform )
        , lidar_transform( config.lidar_intrinsics.lidar_to_sensor_transform )
    {}
};

static snark::ouster::OS1::beam_angle_lut_t beam_angle_lut;
static intrinsics_t intrinsics;

template< typename I, typename O >
struct app
{
    static std::string output_fields();
    static std::string output_format() { return comma::csv::format::value< O >(); }
    static std::string output_frame( const intrinsics_t& intrinsics );

    static void process( const I& data_block, comma::csv::binary_output_stream< O >& os );

    static int run( const comma::command_line_options& options )
    {
        if( options.exists( "--output-fields" )) { std::cout << output_fields() << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << output_format() << std::endl; return 0; }

        std::vector< std::string > config_components = comma::split( options.value< std::string >( "--config", default_config ), ':' );
        std::string config_filename = config_components[0];
        std::string config_path = ( config_components.size() > 1 ? config_components[1] : "" );
        snark::ouster::OS1::config_t config = comma::read_json< snark::ouster::OS1::config_t >( config_filename, config_path, false );

        intrinsics = intrinsics_t( config );

        if( options.exists( "--output-frame" ))
        {
            std::cout << output_frame( intrinsics ) << std::endl;
            return 0;
        }

        beam_angle_lut = snark::ouster::OS1::get_beam_angle_lut( config.beam_intrinsics );

        output();
        return 0;
    }

    static void output()
    {
        char buf[ sizeof( I )];

        comma::csv::options output_csv;
        output_csv.format( comma::csv::format::value< O >() );
        comma::csv::binary_output_stream< O > os( std::cout, output_csv );

        std::cin.tie( nullptr );
        while( std::cin.good() && !std::cin.eof() )
        {
            std::cin.read( buf, sizeof( I ));
            if( std::cin.gcount() < ( long )( sizeof( I ))) { break; }
            const I* data_block = reinterpret_cast< I* >( buf );
            if( data_block ) { process( *data_block, os ); }
        }
    }
};

template <>
std::string app< snark::ouster::OS1::azimuth_block_t, snark::ouster::output_lidar_t >::output_fields()
{
    return comma::join( comma::csv::names< snark::ouster::output_lidar_t >( false ), ',' );
}

template<>
std::string app< snark::ouster::OS1::azimuth_block_t, snark::ouster::output_lidar_t >::output_frame( const intrinsics_t& intrinsics )
{
    return comma::join( intrinsics.lidar_transform.frame(), ',' );
}

template<>
void app< snark::ouster::OS1::azimuth_block_t, snark::ouster::output_lidar_t >::process( const snark::ouster::OS1::azimuth_block_t& azimuth_block, comma::csv::binary_output_stream< snark::ouster::output_lidar_t >& os )
{
    static comma::uint32 block_id = 0;
    static comma::uint32 last_encoder_count = 0;

    if( azimuth_block.packet_status() == snark::ouster::OS1::packet_status_good )
    {
        if( azimuth_block.encoder_count() < last_encoder_count ) { block_id++; }
        last_encoder_count = azimuth_block.encoder_count();
        const snark::ouster::output_azimuth_block_t output_azimuth_block( azimuth_block, block_id );
        const double azimuth_encoder_angle( M_PI * 2 * azimuth_block.encoder_count() / snark::ouster::OS1::encoder_ticks_per_rev );
        for( comma::uint16 channel = 0; channel < azimuth_block.data_blocks.size(); ++channel )
        {
            snark::ouster::output_data_block_t data_block( azimuth_encoder_angle
                                                         , azimuth_block.data_blocks[channel]
                                                         , channel
                                                         , beam_angle_lut
                                                         , intrinsics.lidar_transform );
            // 16 beam devices include 64 channels in the data packet, but most of them are invalid.
            // The valid ones have a non-zero signal.
            if( data_block.signal > 0 )
            {
                os.write( snark::ouster::output_lidar_t( output_azimuth_block, data_block ));
            }
        }
        os.flush();
    }
}

template <>
std::string app< snark::ouster::OS1::imu_block_t, snark::ouster::output_imu_t >::output_fields()
{
    return comma::join( comma::csv::names< snark::ouster::output_imu_t >( true ), ',' );
}

template<>
std::string app< snark::ouster::OS1::imu_block_t, snark::ouster::output_imu_t >::output_frame( const intrinsics_t& intrinsics )
{
    return comma::join( intrinsics.imu_transform.frame(), ',' );
}

template<>
void app< snark::ouster::OS1::imu_block_t, snark::ouster::output_imu_t >::process( const snark::ouster::OS1::imu_block_t& data_block, comma::csv::binary_output_stream< snark::ouster::output_imu_t >& os )
{
    os.write( snark::ouster::output_imu_t( data_block ));
    os.flush();
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );

        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--output-fields,--output-format,--output-frame,--verbose,-v", "-.*" );
        if( unnamed.size() == 1 )
        {
            if( unnamed[0] == "lidar" ) { return app< snark::ouster::OS1::azimuth_block_t, snark::ouster::output_lidar_t >::run( options ); }
            else if( unnamed[0] == "imu" ) { return app< snark::ouster::OS1::imu_block_t, snark::ouster::output_imu_t >::run( options ); }
        }
        std::cerr << "ouster-to-csv: require one of lidar or imu" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "ouster-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "ouster-to-csv: unknown exception" << std::endl; }
    return 1;
}
