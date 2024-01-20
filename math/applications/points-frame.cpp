// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2023-2024 Vsevolod Vlaskine
// All rights reserved

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif

#include <map>
#include <set>
#include <tuple>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/none.h>
#include <comma/csv/ascii.h>
#include <comma/csv/binary.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include "../../visiting/eigen.h"
#include "../frames/tree.h"
#include "frame.h"

namespace snark { namespace applications { namespace frames { namespace config { static std::string usage( bool verbose ); } } } }

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "convert timestamped Cartesian points from a reference frame to a target coordinate frame[s]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-frame <options> > points.converted.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --from[=<frame>] : convert points from frames to the reference frame," << std::endl;
    std::cerr << "    --to[=<frame>] : convert points from the reference frame to frame" << std::endl;
    std::cerr << "        <frame> ::= <frame>[+<frames>]" << std::endl;
    std::cerr << "        <frame> : name of file with timestamped nav data" << std::endl;
    std::cerr << "                  or <x>,<y>,<z>[,<roll>,<pitch>,<yaw>]" << std::endl;
    std::cerr << "                  nav data in file: <t>,<x>,<y>,<z>,<roll>,<pitch>,<yaw>" << std::endl;
    std::cerr << "                      default fields: t,x,y,z,roll,pitch,yaw" << std::endl;
    std::cerr << "                      examples:t,x,y,z,roll,pitch,yaw" << std::endl;
    std::cerr << "                          ascii" << std::endl;
    std::cerr << "                              cat txyz.csv | points-frame --from 'nav.bin;binary=t,6d" << std::endl;
    std::cerr << "                          binary, default nav data fields" << std::endl;
    std::cerr << "                              cat txyz.bin | points-frame --binary=t,3d --from 'nav.bin;binary=t,6d" << std::endl;
    std::cerr << "                          binary, custom nav data fields" << std::endl;
    std::cerr << "                              cat txyz.bin | points-frame --binary=t,3d --from 'nav.bin;binary=t,2ui,6d;fields=t,,,x,y,z,roll,pitch,yaw" << std::endl;
    std::cerr << "                          see more examples in examples section below" << std::endl;
    std::cerr << "      ATTENTION: if 'frame' field present on stdin, specify --from or --to without value, default behaviour: --from" << std::endl;
    std::cerr << "    --discard-out-of-order,--discard : if present, discard out of order points silently" << std::endl;
    std::cerr << "    --frame[=<frame>]; default: 0,0,0,0,0,0; if 'frame' field present, <frame> will be used as default for frame fields" << std::endl;
    std::cerr << "    --max-gap <seconds> : max valid time gap between two successive nav solutions;" << std::endl;
    std::cerr << "                          if exceeded, input points between those two timestamps" << std::endl;
    std::cerr << "                          will be discarded, thus use --discard, too; default: infinity" << std::endl;
    std::cerr << "    --no-interpolate: don't interpolate, use nearest point instead" << std::endl;
    std::cerr << "    --output-frame : output each frame for each point" << std::endl;
    std::cerr << "                     can be individually specified for a frame, e.g.:" << std::endl;
    std::cerr << "                     --from \"novatel.csv;output-frame\"" << std::endl;
    std::cerr << "    --pose,--position=[<x>,<y>,<z>,<roll>,<pitch>,<yaw>]: default position, if 'frame' fields are present" << std::endl;
    std::cerr << "        example scenario: you have a gps unit trajectory and gps unit offset from the centre of the vehicle" << std::endl;
    std::cerr << "                          now, you want to find the trajectory of the centre of the vehicle" << std::endl;
    std::cerr << "                          in this case, the gps unit trajectory is the frame, which is variable," << std::endl;
    std::cerr << "                          while the point to georeference (the vehicle centre) is fixed" << std::endl;
    std::cerr << "                          then you could run:" << std::endl;
    std::cerr << "                              cat gps.unit.trajectory.csv | points-frame --fields frame --pose <gps unit offset relative to vehicle centre>" << std::endl;
    std::cerr << "                          this is equivalent to a much longer version:" << std::endl;
    std::cerr << "                              cat gps.unit.trajectory.csv \\" << std::endl;
    std::cerr << "                                  | csv-paste - value=<gps unit offset relative to vehicle centre> \\" << std::endl;
    std::cerr << "                                  | points-frame --fields frame,position \\" << std::endl;
    std::cerr << "                                  | csv-shuffle --fields ,,,,,,,,,,,,x,y,z,roll,pitch,yaw --output-fields x,y,z,roll,pitch,yaw \\" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    fields" << std::endl;
    std::cerr << "        t: timestamp" << std::endl;
    std::cerr << "        x,y,z: coordinates" << std::endl;
    std::cerr << "        roll,pitch,yaw: rotation" << std::endl;
    std::cerr << "        frame/coordinates/x,frame/coordinates/y,frame/coordinates/z,frame/orientation/roll,frame/orientation/pitch,frame/orientation/yaw: frame" << std::endl;
    std::cerr << "            if frame fields are present, get frame from them, apply to" << std::endl;
    std::cerr << "            coordinates (x,y,z) and output input line with converted" << std::endl;
    std::cerr << "            coordinates and rotation appended as x,y,z,roll,pitch,yaw (also see --emplace below)" << std::endl;
    std::cerr << "            options" << std::endl;
    std::cerr << "                --emplace,--in-place: if present, output transform result in place instead of appending" << std::endl;
    std::cerr << "                --from: if present with no argument, conversion is from reference frame, default" << std::endl;
    std::cerr << "                --to: if present with no argument, conversion is to reference frame" << std::endl;
    std::cerr << "        frames: same usage as frame, but allows multiple frames which will be applied in order of their indices" << std::endl;
    std::cerr << "            if frame field is present, it is the same as frames[0] for backward compatibility" << std::endl;
    std::cerr << "            if --frame option is the same as --from/--to=frames[0]=... for backward compatibility" << std::endl;
    std::cerr << "            e.g: --fields=x,y,z,frame[0],,,frame[2]/x,frame[2]/y,frame[2]/z,,,,frame[1]/yaw" << std::endl;
    std::cerr << "            e.g: --fields=x,y,z,frame[0],,,frame[2]/x,frame[2]/y,frame[2]/z,,,,frame[1]/yaw" << std::endl;
    std::cerr << "            options" << std::endl;
    std::cerr << "                --emplace,--in-place: if present, output transform result in place instead of appending" << std::endl;
    std::cerr << "                --from: if present with no argument, conversion is from reference frame, default" << std::endl;
    std::cerr << "                --from=frames[<i>]: default value for frames[<i>] to convert from" << std::endl;
    std::cerr << "                --to: if present with no argument, conversion is to reference frame" << std::endl;
    std::cerr << "                --to=frames[<i>]: default value for frames[<i>] to convert to" << std::endl;
    std::cerr << "        default: t,x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    IMPORTANT: <frame> is the transformation from reference frame to frame" << std::endl;
    std::cerr << "               If still confused, try simple coordinate transformations," << std::endl;
    std::cerr << "               just like examples below." << std::endl;
    std::cerr << std::endl;
    std::cerr << "config" << std::endl;
    std::cerr << snark::applications::frames::config::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << "examples (try them):" << std::endl;
    if( verbose )
    {
        std::cerr << "    echo 20101010T101010,1,0,0 | points-frame --from \"100,100,0\"" << std::endl;
        std::cerr << "    20101010T101010,101,100,0" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    echo 20101010T101010,1,0,0 | points-frame --from \"100,100,0,0,0,$(math-deg2rad 90)\"" << std::endl;
        std::cerr << "    20101010T101010,100,101,0" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    echo 20101010T101010,1,0,0 | points-frame --from \"0,-3,2,$(math-deg2rad 90),0,0\"" << std::endl;
        std::cerr << "    20101010T101010,1,-3,2" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    echo 20101010T101010,1,0,0 | points-frame --from \"3,2,0,0,$( math-deg2rad 90 ),0\"" << std::endl;
        std::cerr << "    20101010T101010,3,2,-1" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    echo 20101010T101010,1,0,0 | points-frame --from \"-2,1,3,0,0,$(math-deg2rad 90)\"" << std::endl;
        std::cerr << "    20101010T101010,-2,2,3" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    echo 1,2,3 | points-frame --from \"3,2,1\" --fields=x,y,z" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    using second stream (e.g. navigation)" << std::endl;
        std::cerr << "        basics" << std::endl;
        std::cerr << "            ( echo 20100101T000000,-2,4,3; echo 20100101T000001,3,2,1 ) > frames.csv" << std::endl;
        std::cerr << "            echo 20100101T000000,1,2,3 | points-frame --fields=t,x,y,z --from 'frames.csv;fields=t,x,y,z'" << std::endl;
        std::cerr << "        input: some 6DoF data in local frame nav.csv: timestamped nav 6DoF reference frames" << std::endl;
        std::cerr << "            cat log.csv | points-frame --from nav.csv > log.world.csv" << std::endl;
        std::cerr << "        live stream" << std::endl;
        std::cerr << "            socat tcp:some-address:12345 | points-frame --from tcp:nav-address:6789 > log.world.csv" << std::endl;
        std::cerr << "    frame is passed on stdin" << std::endl;
        std::cerr << "        translation and rotation" << std::endl;
        std::cerr << "            echo 1,2,3,10,20,30,0,0,0 | points-frame --fields x,y,z,frame  --from" << std::endl;
        std::cerr << "            1,2,3,10,20,30,0,0,0,11,22,33,0,-0,0" << std::endl;
        std::cerr << "        translation only" << std::endl;
        std::cerr << "            echo 1,2,3,10,20,30 | points-frame --fields x,y,z,frame/x,frame/y,frame/z --from" << std::endl;
        std::cerr << "            1,2,3,10,20,30,11,22,33,0,-0,0" << std::endl;
        std::cerr << "        rotation only" << std::endl;
        std::cerr << "            echo 1,2,3,0,$( math-deg2rad 90 ),0 | points-frame --fields x,y,z,frame/roll,frame/pitch,frame/yaw --from" << std::endl;
        std::cerr << "            1,2,3,0,1.57079632679,0,3,2,-1,0,1.57079632679,0" << std::endl;
        std::cerr << "        multiple frames" << std::endl;
        std::cerr << "            basics" << std::endl;
        std::cerr << "                echo 1,2,3,0,$( math-deg2rad 90 ),0,1000 \\" << std::endl;
        std::cerr << "                    | points-frame --fields x,y,z,frames[0]/roll,frames[0]/pitch,frames[0]/yaw,frames[1]/x \\" << std::endl;
        std::cerr << "                                   --from \\" << std::endl;
        std::cerr << "                                   --emplace" << std::endl;
        std::cerr << "                1003,2,-1,0,1.57079632679,0,1000" << std::endl;
        std::cerr << "            using defaults/fixed frames" << std::endl;
        std::cerr << "                echo 1,2,3,0,$( math-deg2rad 90 ),0,1000 \\" << std::endl;
        std::cerr << "                    | points-frame --fields x,y,z,frames[0]/roll,frames[0]/pitch,frames[0]/yaw,frames[2]/x \\" << std::endl;
        std::cerr << "                                   --from=frames[1]=5,6,7,0,0,0 \\" << std::endl;
        std::cerr << "                                   --to=frames[2]=1,2,3,0,0,0 \\" << std::endl;
        std::cerr << "                                   --emplace" << std::endl;
        // todo: epicycles
        // csv-paste line-number | csv-eval --flush --fields a 'a*=pi/180/10' | csv-shuffle --fields a --output-fields a,a,a,a | csv-eval --fields a,b,c 'a*=4.53;b*=0.01;c*=0.053;d*=0.042' --flush | csv-paste value=100,0,0,0,0,0 - | points-frame --fields x,y,z,roll,pitch,yaw,frames[0]/yaw,frames[1]/yaw,frames[2]/yaw --from frames[0]=100,0,0,0,0,0 --from frames[1]=83,0,0,0,0,0 --from frames[2]=57,0,0,0,0,0 --emplace -v | view-points '-;size=5000000' --orthographic
        // csv-paste line-number | csv-eval --flush --fields a 'a*=pi/180/10' | csv-shuffle --fields a --output-fields a,a,a,a | csv-eval --fields a,b,c 'a*=4.53;b*=0.01;c*=0.053;d*=0.042' --flush | csv-paste value=100,0,0,0,0,0 - | points-frame --fields x,y,z,roll,pitch,yaw,frames[0]/yaw,frames[1]/yaw,frames[2]/yaw,frames[1]/pitch --from frames[0]=100,0,0,0,0,0 --from frames[1]=83,0,0,0,0,0 --from frames[2]=57,0,0,0,0,0 --emplace -v | view-points '-;size=5000000' --orthographic
        // csv-paste line-number | csv-eval --flush --fields a 'a*=pi/180/10' | csv-shuffle --fields a --output-fields a,a,a,a | csv-eval --fields a,b,c 'a*=0.00053;b*=-0.001;c*=-0.033134;d*=-0.0012' --flush | csv-paste value=50,0,0,0,0,0 - | points-frame --fields x,y,z,roll,pitch,yaw,frames[0]/yaw,frames[1]/yaw,frames[2]/yaw,frames[2]/pitch --from frames[0]=50,0,0,0,0,0 --from frames[1]=153,0,0,0,0,0 --from frames[2]=87,0,0,0,0,0 --emplace -v | view-points '-;size=5000000' --camera-config camera.json  --window-geometry 2000,0,2000,1500
    }
    else
    {
        std::cerr << "    to see examples run points-frame --help --verbose" << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

namespace snark { namespace applications {

static bool timestamp_required{false}; // quick and dirty
static boost::optional< boost::posix_time::time_duration > max_gap = comma::silent_none< boost::posix_time::time_duration >();
static uint64_t discarded_counter{0};
static boost::posix_time::time_duration discarded_time_diff_max;

std::vector< boost::shared_ptr< snark::applications::frame > > parse_frames( const std::vector< std::string >& values
                                                                           , const comma::csv::options& options
                                                                           , bool discard_out_of_order
                                                                           , bool outputframe
                                                                           , std::vector< bool > to
                                                                           , bool interpolate
                                                                           , bool rotation_present )
{
    std::vector< boost::shared_ptr< snark::applications::frame > > frames;
    for( std::size_t i = 0; i < values.size(); ++i )
    {
        std::vector< std::string > s = comma::split( values[i], '+' );
        for( std::size_t j = 0; j < s.size(); ++j )
        {
            std::string stripped = comma::strip( s[j], ' ' );
            std::vector< std::string > t = comma::split( stripped, ',' );
            boost::optional< snark::applications::position > position;
            try
            {
                switch( t.size() )
                {
                    case 3: position = snark::applications::position( comma::csv::ascii< Eigen::Vector3d >().get( t ) ); break;
                    case 6: position = comma::csv::ascii< snark::applications::position >().get( t ); break;
                    default: break;
                }
            }
            catch( ... ) {}
            if( position )
            {
                frames.push_back( boost::shared_ptr< snark::applications::frame >( new snark::applications::frame( *position, to[i], interpolate, rotation_present ) ) );
            }
            else
            {
                comma::csv::options csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( stripped );
                csv.full_xpath = false;
                if( csv.fields == "" )
                {
                    csv.fields = "t,x,y,z,roll,pitch,yaw";
                    if( options.binary() ) { csv.format( "t,6d" ); }
                }
                outputframe = outputframe || comma::name_value::map( stripped, "filename" ).exists( "output-frame" );
                timestamp_required = true;
                frames.push_back( boost::shared_ptr< snark::applications::frame >( new snark::applications::frame( csv, discard_out_of_order, max_gap, outputframe, to[i], interpolate, rotation_present ) ) );
            }
        }
    }
    return frames;
}

static void run( const std::vector< boost::shared_ptr< snark::applications::frame > >& frames, const comma::csv::options& csv )
{
    comma::signal_flag is_shutdown;
    comma::csv::input_stream< snark::applications::frame::point_type > istream( std::cin, csv );
    comma::csv::output_stream< snark::applications::frame::point_type > ostream( std::cout, csv );
    // ---------------------------------------------------
    // outputting frame: quick and dirty, uglier than britney spears! brush up later
    unsigned int output_frame_count = 0;
    for( std::size_t i = 0; i < frames.size(); ++i ) { if( frames[i]->outputframe ) { ++output_frame_count; } }
    boost::scoped_ptr< comma::csv::binary< snark::applications::frame::point_type > > binary_point;
    boost::scoped_ptr< comma::csv::ascii< snark::applications::frame::point_type > > ascii_point;
    boost::scoped_ptr< comma::csv::binary< snark::applications::position > > binary_frame;
    boost::scoped_ptr< comma::csv::ascii< snark::applications::position > > ascii_frame;
    if( output_frame_count > 0 )
    {
        if( csv.binary() )
        {
            binary_point.reset( new comma::csv::binary< snark::applications::frame::point_type >( csv ) );
            binary_frame.reset( new comma::csv::binary< snark::applications::position >() );
        }
        else
        {
            ascii_point.reset( new comma::csv::ascii< snark::applications::frame::point_type >( csv ) );
            ascii_frame.reset( new comma::csv::ascii< snark::applications::position >() );
        }
    }
    // ---------------------------------------------------
    std::string output_buf;
    if( csv.binary() ) { output_buf.resize( csv.format().size() ); }
    while( !is_shutdown )
    {
        const snark::applications::frame::point_type* p = istream.read();
        if( p == NULL ) { return; }
        snark::applications::frame::point_type converted = *p;
        const snark::applications::frame::point_type* c = NULL;
        bool discarded = false;
        for( std::size_t i = 0; i < frames.size(); ++i )
        {
            c = frames[i]->converted( converted );
            discarded = frames[i]->discarded();
            if( discarded ) 
            {
                if(discarded_time_diff_max < frames[i]->discarded_time_diff())
                    discarded_time_diff_max = frames[i]->discarded_time_diff();
                discarded_counter++;
                break; 
            }
            if( c == NULL ) { return; }
            converted = *c;
        }
        if( discarded ) { continue; }
        if( output_frame_count > 0 ) // quick and dirty, and probably slow; brush up later
        {
            // ---------------------------------------------------
            // outputting frame: quick and dirty, uglier than britney spears! brush up later
            if( csv.binary() )
            {
                static const std::size_t point_size = csv.format().size();
                static const std::size_t position_size = comma::csv::format( comma::csv::format::value< snark::applications::position >() ).size();
                static const std::size_t size = point_size + output_frame_count * position_size;
                std::vector< char > buf( size );
                ::memset( &buf[0], 0, size );
                ::memcpy( &buf[0], istream.binary().last(), point_size );
                binary_point->put( converted, &buf[0] );
                unsigned int count = 0;
                for( std::size_t i = 0; i < frames.size(); ++i )
                {
                    if( !frames[i]->outputframe ) { continue; }
                    binary_frame->put( frames[i]->last(), &buf[0] + point_size + count * position_size );
                    ++count;
                }
                std::cout.write( &buf[0], size );
                if( csv.flush ) { std::cout.flush(); }
            }
            else
            {
                std::string s = comma::join( istream.ascii().last(), csv.delimiter );
                ascii_point->put( converted, s );
                for( std::size_t i = 0; i < frames.size(); ++i )
                {
                    if( !frames[i]->outputframe ) { continue; }
                    std::vector< std::string > f;
                    ascii_frame->put( frames[i]->last(), f );
                    s += ( csv.delimiter + comma::join( f, csv.delimiter ) );
                }
                std::cout << s << std::endl;
            }
            // ---------------------------------------------------
        }
        else
        {
            if( csv.binary() )
            {
                ::memcpy( &output_buf[0], istream.binary().last(), csv.format().size() );
                ostream.write( converted, output_buf );
            }
            else
            {
                ostream.write( converted, istream.ascii().last() );
            }
        }
    }
}

struct position_and_frames
{
    snark::applications::position position;
    std::vector< snark::applications::position > frames;
};

} } // namespace snark { namespace applications {

namespace comma { namespace visiting {

template <> struct traits< snark::applications::position_and_frames >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::applications::position_and_frames& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "frames", p.frames );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::applications::position_and_frames& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "frames", p.frames );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace applications {

struct transform // todo: quick and dirty for now; brush this all up
{
    Eigen::Translation3d translation;
    Eigen::Matrix3d rotation;
    Eigen::Affine3d affine;
    bool from{false};
    bool precomputed{true}; // todo

    transform() = default;
    transform( const snark::applications::position& frame, bool from, bool precomputed = true )
        : translation( frame.coordinates )
        , rotation( snark::rotation_matrix::rotation( frame.orientation ) )
        , affine( from ? ( translation * rotation ) : ( rotation.transpose() * translation.inverse() ) )
        , from( from )
        , precomputed( precomputed )
    {
    }

    snark::applications::position operator*( const snark::applications::position& rhs ) const // todo: simplify
    {
        Eigen::Matrix3d m = snark::rotation_matrix::rotation( rhs.orientation );
        if( from ) { m = rotation * m; } else { m = rotation.transpose() * m; }
        return snark::applications::position( affine * rhs.coordinates, snark::rotation_matrix::roll_pitch_yaw( m ) );
    }
};

namespace frames {

namespace config {

std::string usage( bool verbose )
{
    std::string s =
R"(    options
        --config=[<path>]; json config file
        --config-frame-field-name,--frame-field; default=frame; todo:document
        --config-expand,--expand-config; todo: expand offsets in config following paths in it
)";
    return s + ( verbose ?
R"(    data model
        todo
    examples
        todo: camera pose in world frame (frame[1])
            some-input.csv | points-frames   --config config.json:frames \
                                             --from frame[0]=platform/mount/joint/camera \
                                             --fields x,y,z,roll,pitch,yaw,frames[1] \
                                             --emplace
        todo: camera pose in mount frame
            echo 0,0,0,0,0,0 | points-frames --config config.json:frames \
                                             --from frame[0]=platform/mount/joint/camera \
                                             --to frame[1]=platform/mount
                                             --fields x,y,z,roll,pitch,yaw \
                                             --emplace
        todo: camera in mount frame, shortcut assuming frame indices are in order of appearance, i.e. camera frame index is 0, mount frame index is 1
            echo 0,0,0,0,0,0 | points-frames --config config.json:frames \
                                             --from platform/mount/joint/camera \
                                             --to platform/mount
                                             --fields x,y,z,roll,pitch,yaw \
                                             --emplace
        todo: camera in world frame (frames[2]) where joint is rotating around yaw axis
            some-input.csv | points-frames   --config config.json:frames \
                                             --from frame[0]=platform/mount/joint/camera \
                                             --fields x,y,z,roll,pitch,yaw,platform/mount/joint/position/yaw,frames[2] \
                                             --emplace
        todo: more examples...)"
                   :
R"(    data model, examples
        run --help --verbose for more...)" );
}

static bool handle_info_options( const comma::command_line_options& options )
{
    if( options.exists( "--config-expand,--expand-config" ) )
    {
        std::string config = options.value< std::string >( "--config", "-" );
        auto c = comma::split( config, ':', true );
        auto f = snark::frames::tree::frame_format::from_string( options.value( "--config-frame-format", std::string() ) );
        auto t = c.size() == 1 ? snark::frames::tree::make( config, f ) : snark::frames::tree::make( c[0], c[1], f ); // todo: handle xpath
        comma::name_value::impl::write_json( std::cout, t(), true, true );
        return true;
    }
    return false;
}

static snark::frames::tree tree( const comma::command_line_options& options )
{
    std::string config = options.value< std::string >( "--config", "" );
    if( config.empty() ) { return snark::frames::tree(); }
    auto c = comma::split( config, ':', true );
    auto f = snark::frames::tree::frame_format::from_string( options.value( "--config-frame-format", std::string() ) );
    return c.size() == 1 ? snark::frames::tree::make( config, f ) : snark::frames::tree::make( c[0], c[1], f ); // todo: handle xpath
}

} // namespace config {

static std::pair< unsigned int, std::string::size_type > index( const std::string& f )
{
    auto n = comma::split( f, '=' )[0]; // quick and dirty for now
    comma::xpath p( n );
    return p.elements.size() > 0 && p.elements[0].name == "frames" && p.elements[0].index ? std::make_pair( *p.elements[0].index, p.elements[0].to_string().size() ) : std::make_pair( std::size_t( 0 ), std::string::npos );
}

static std::pair< std::string, std::set< unsigned int > > parse_fields( const std::string& fields, const snark::frames::tree& t, const std::map< std::string, unsigned int >& aliases )
{
    ( void )t; ( void )aliases;
    auto v = comma::split( fields, ',' );
    std::set< unsigned int > indices;
    for( auto& f: v ) // quick and dirty: keeping it backward compatible
    {
        if( f.empty() ) { continue; }
        if( f == "x" || f == "y" || f == "z" || f == "roll" || f == "pitch" || f == "yaw" ) { f = "position/" + f; }
        else if( f == "frame" ) { f = "frames[0]"; }
        else if( f.substr( 0, 6 ) == "frame/" ) { f = "frames[0]/" + f.substr( 6 ); }
        else if( !aliases.empty() )
        {
            comma::xpath p( f );
            if( p.elements.back().index ) { continue; }
            auto n = p.elements.back().name;
            if( !( n == "x" || n == "y" || n == "z" || n == "roll" || n == "pitch" || n == "yaw" ) ) { n = std::string(); }
            else { p = p.head(); n = "/" + n; }
            auto it = aliases.find( p.to_string() );
            if( it == aliases.end() ) { continue; }
            f = "frames[" + std::to_string( it->second ) + "]" + n;
        }
    }


    // todo! handle frame indices vs aliases
    

    for( auto& f: v ) // super-quick and dirty for now
    {
        unsigned int i;
        std::string::size_type p;
        std::tie( i, p ) = frames::index( f ); // auto [ i, p ] = frame_index( f ); // made backward compatible: avoid compiler warnings for now
        if( p == std::string::npos ) { continue; }
        if( p == f.size() )
        {
            indices.insert( i );
        }
        else
        {
            COMMA_ASSERT_BRIEF( f[p] == '/', "invalid csv fields: '" << fields << "'" );
            auto r = f.substr( p + 1 );
            if( r == "x" || r == "y" || r == "z" || r == "roll" || r == "pitch" || r == "yaw" ) { indices.insert( i ); }
        }
    }
    return std::make_pair( comma::join( v, ',' ), indices );
}

static std::pair< std::map< unsigned int, snark::applications::position >, bool > from_options( const comma::command_line_options& options, const std::string& option, const snark::frames::tree& t, std::map< std::string, unsigned int >& aliases )
{
    const auto& values = options.values< std::string >( option );
    std::pair< std::map< unsigned int, snark::applications::position >, bool > m;
    m.second = false;
    for( const auto& v: values )
    {
        unsigned int index;
        std::string::size_type pos;
        std::tie( index, pos ) = frames::index( v ); // auto [ index, pos ] = frame_index( v ); // made backward compatible: avoid compiler warnings for now
        if( pos == std::string::npos || v[pos] != '=' ) { m.second = true; }
        else { m.first[index] = comma::csv::ascii< snark::applications::position >().get( v.substr( pos + 1 ) ); }
    }
    if( m.first.empty() && options.exists( option ) ) { m.second = true; }
    return m;
}

static bool run( const comma::command_line_options& options )
{
    if( frames::config::handle_info_options( options ) ) { return true; }
    comma::csv::options csv( options );
    std::string frame_field = options.value< std::string >( "--config-frame-field-name,--frame-field", "frame" );
    ( void )frame_field;
    auto v = comma::split( csv.fields, ',' );
    for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "x" || v[i] == "y" || v[i] == "z" || v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "position/" + v[i]; } }
    std::set< unsigned int > indices;
    auto tree = frames::config::tree( options );
    std::map< std::string, unsigned int > aliases; // todo

    // todo! fix! --from and --to may be interleaved!

    const auto& froms = frames::from_options( options, "--from", tree, aliases );
    const auto& tos = frames::from_options( options, "--to", tree, aliases );
    std::tie( csv.fields, indices ) = frames::parse_fields( csv.fields, tree, aliases );
    if( indices.empty() && froms.first.empty() && tos.first.empty() ) { return false; }
    COMMA_ASSERT_BRIEF( !froms.second || !tos.second, "--from and --to are mutually exclusive as default direction of frame conversions" );
    bool emplace = options.exists( "--emplace,--in-place" );
    snark::applications::position_and_frames sample;
    sample.position = comma::csv::ascii< snark::applications::position >().get( options.value< std::string >( "--pose,--position", "0,0,0,0,0,0" ) );
    bool from = froms.second || !tos.second;
    // todo: add checks of --from, --to consistency, same frame index repeating, etc
    unsigned int max_index = indices.empty() ? 0 : *indices.rbegin();
    if( !froms.first.empty() && froms.first.rbegin()->first > max_index ) { max_index = froms.first.rbegin()->first; }
    if( !tos.first.empty() && tos.first.rbegin()->first > max_index ) { max_index = tos.first.rbegin()->first; }
    sample.frames = std::vector< snark::applications::position >( max_index + 1 );
    sample.frames[0] = comma::csv::ascii< snark::applications::position >().get( options.value< std::string >( "--frame", "0,0,0,0,0,0" ) ); // for backward compatibility
    std::vector< snark::applications::transform > transforms( sample.frames.size() );
    for( auto& t: transforms ) { t.from = from; }
    for( const auto& i: froms.first ) { transforms[i.first].from = true; sample.frames[i.first] = i.second; }
    for( const auto& i: tos.first ) { transforms[i.first].from = false; sample.frames[i.first] = i.second; }
    for( unsigned int i = 0; i < sample.frames.size(); ++i ) { transforms[i] = snark::applications::transform( sample.frames[i], transforms[i].from ); }
    for( unsigned int i : indices ) { transforms[i].precomputed = false; }
    comma::csv::input_stream< snark::applications::position_and_frames > is( std::cin, csv, sample );
    comma::csv::options output_csv;
    output_csv.flush = csv.flush;
    if( csv.binary() ) { output_csv.format( comma::csv::format::value< snark::applications::position >() ); }
    comma::csv::output_stream< snark::applications::position > os( std::cout, output_csv );
    comma::csv::tied< snark::applications::position_and_frames, snark::applications::position > tied( is, os );
    comma::csv::passed< snark::applications::position_and_frames > passed( is, std::cout, csv.flush );
    while( is.ready() || std::cin.good() )
    {
        const snark::applications::position_and_frames* p = is.read();
        if( !p ) { break; }
        snark::applications::position r = p->position;
        for( unsigned int i = 0; i < p->frames.size(); ++i )
        {
            r = ( transforms[i].precomputed ? transforms[i] : snark::applications::transform( p->frames[i], transforms[i].from ) ) * r;
        }
        if( emplace ) { passed.write( snark::applications::position_and_frames{r, p->frames} ); } else { tied.append( r ); }
    }
    return true;
}

} // namespace frames {

static int run( const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    csv.full_xpath = false;
    if( csv.fields == "" ) { csv.fields="t,x,y,z"; }
    std::vector< std::string > v = comma::split( csv.fields, ',' );
    bool rotation_present = false;
    for( unsigned int i = 0; i < v.size() && !rotation_present; ++i ) { rotation_present = v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw"; }
    if( options.exists( "--pose,--position" ) ) { std::cerr << "points-frame: --pose given, but no frame fields on stdin: not supported, yet" << std::endl; return 1; }
    bool discard_out_of_order = options.exists( "--discard-out-of-order,--discard" );
    bool from = options.exists( "--from" );
    bool to = options.exists( "--to" );
    if( !to && !from ) { COMMA_THROW( comma::exception, "please specify either --to or --from" ); }
    bool interpolate = !options.exists( "--no-interpolate" );
    std::vector< std::string > names = options.names();
    std::vector< bool > to_vector;
    for( unsigned int i = 0u; i < names.size(); i++ )
    {
        if( names[i] == "--from" ) { to_vector.push_back( false ); }
        else if( names[i] == "--to" ) { to_vector.push_back( true ); }
    }
    if( options.exists( "--max-gap" ) )
    {
        double d = options.value< double >( "--max-gap" );
        if( !comma::math::less( 0, d ) ) { std::cerr << "points-frame: expected --max-gap in seconds, got " << d << std::endl; usage(); }
        max_gap = boost::posix_time::seconds( int( d ) ) + boost::posix_time::microseconds( int( 1000000.0 * ( d - int( d ) ) ) );
    }
    std::vector< boost::shared_ptr< snark::applications::frame > > frames = parse_frames( options.values< std::string >( "--from,--to" )
                                                                                        , csv
                                                                                        , discard_out_of_order
                                                                                        , options.exists( "--output-frame" )
                                                                                        , to_vector
                                                                                        , interpolate
                                                                                        , rotation_present );
    //if( timestamp_required ) { if( csv.fields != "" && !comma::csv::namesValid( comma::split( csv.fields, ',' ), comma::split( "t,x,y,z", ',' ) ) ) { COMMA_THROW( comma::exception, "expected mandatory fields t,x,y,z; got " << csv.fields ); } }
    //else { if( csv.fields != "" && !comma::csv::namesValid( comma::split( csv.fields, ',' ), comma::split( "x,y,z", ',' ) ) ) { COMMA_THROW( comma::exception, "expected mandatory fields x,y,z; got " << csv.fields ); } }
    if( timestamp_required ) { if( csv.fields != "" && !comma::csv::fields_exist( csv.fields, "t" ) ) { COMMA_THROW( comma::exception, "expected mandatory field t; got " << csv.fields ); } }
    snark::applications::run( frames, csv );
    if( discarded_counter ) { std::cerr << "points-frame: discarded " << discarded_counter << " points; max time diff: " << discarded_time_diff_max.total_microseconds() <<" microseconds" << std::endl; }
    return 0;
}

} } // namespace snark { namespace applications {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( !snark::applications::frames::run( options ) ) { return snark::applications::run( options ); }
    }
    catch( std::exception& ex ) { std::cerr << "points-frame: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-frame: unknown exception" << std::endl; }
}
