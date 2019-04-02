// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2018 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// snark is a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author vsevolod vlaskine

#include <memory>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include "../../../../timing/time.h"
#include "../../../../visiting/traits.h"
#include "../calculator.h"
#include "../packet.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat robosense*.bin | robosense-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "only rs-lidar-16 currently supported" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --angles,--angle,-a=<filename>; default: as in spec" << std::endl;
    //std::cerr << "    --rotation-per-second=<rps>: specify rotation speed; default: 20" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output options:" << std::endl;
    std::cerr << "    --binary,-b[=<format>]: output in binary equivalent of csv" << std::endl;
    std::cerr << "    --fields <fields>: e.g. t,x,y,z,scan" << std::endl;
    std::cerr << "    --output-fields: todo: print output fields and exit" << std::endl;
    std::cerr << "    --output-invalid-points: output invalid points" << std::endl;
    std::cerr << "    --scans-discard-invalid,--discard-invalid-scans: todo: don't output scans with missing packets" << std::endl;
    std::cerr << "    --scans-missing-packets-threshold,--missing-packets-threshold=<n>: number of consecutive missing packets for new/invalid scan" << std::endl;
    std::cerr << "        if the data is missing more than <n> consecutive packets (calculated based on timestamp), then the next packet after the gap will be marked as the beginning of a new scan" << std::endl;
    std::cerr << "        use --discard-invalid-scans option together with this option to discard all the scans that have missing consecutive packets greater than <n>" << std::endl;
    std::cerr << "        by default (when --missing-packets-threshold is not specified) it will mark new/invalid scan if more than 25 millisecond of data is missing (half a circle at 20Hz)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct output
{
    boost::posix_time::ptime t;
    comma::uint32 id;
    comma::uint32 scan;
    double range;
    double bearing;
    double elevation;
    comma::uint32 reflectivity;
    Eigen::Vector3d coordinates;
    
    output(): id( 0 ), scan( 0 ), range( 0 ), bearing( 0 ), elevation( 0 ), reflectivity( 0 ), coordinates( Eigen::Vector3d::Zero() ) {}
};

namespace comma { namespace visiting {
    
template <> struct traits< output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "id", p.id );
        v.apply( "scan", p.scan );
        v.apply( "range", p.range );
        v.apply( "bearing", p.bearing );
        v.apply( "elevation", p.elevation );
        v.apply( "coordinates", p.coordinates );
    }
};

} } // namespace comma { namespace visiting {

typedef std::pair< boost::posix_time::ptime, snark::robosense::msop::packet* > pair_t;

static pair_t read( std::istream& is, char* buffer )
{
    comma::uint64 microseconds;
    pair_t p( boost::posix_time::not_a_date_time, NULL );
    is.read( reinterpret_cast< char* >( &microseconds ), sizeof( comma::uint64 ) );
    if( is.bad() || is.eof() ) { return p; }
    is.read( buffer, snark::robosense::msop::packet::size );
    if( is.bad() || is.eof() ) { return p; }
    comma::uint64 seconds = microseconds / 1000000; //to avoid time overflow on 32bit systems with boost::posix_time::microseconds( m_microseconds ), apparently due to a bug in boost
    microseconds = microseconds % 1000000;
    static boost::posix_time::ptime epoch( snark::timing::epoch );
    p.first = epoch + boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( microseconds );
    p.second = reinterpret_cast< snark::robosense::msop::packet* >( buffer );
    return p;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output >( false ), ',' ) << std::endl; return 0; }
        bool output_invalid_points = options.exists( "--output-invalid-points" );
        comma::csv::options csv( options );
        csv.full_xpath = false;
        comma::csv::output_stream< output > ostream( std::cout, csv );
        uint32_t scan = 0;
        std::vector< char > buffer( snark::robosense::msop::packet::size );
        snark::robosense::calculator calculator;
        std::string angles = options.value< std::string >( "--angles", "" );
        if( !angles.empty() ) { calculator = snark::robosense::calculator( angles ); }
        while( std::cin.good() && !std::cin.eof() )
        {
            auto p = read( std::cin, &buffer[0] );
            if( !p.second ) { break; }
            for( snark::robosense::msop::packet::const_iterator it( p.second ); !it.done() && std::cout.good(); ++it )
            {
                static double min_range = 0.2; // as in https://github.com/RoboSense-LiDAR/ros_rslidar/blob/develop-curves-function/rslidar_pointcloud/src/rawdata.cc
                static double max_range = 200.0; // as in https://github.com/RoboSense-LiDAR/ros_rslidar/blob/develop-curves-function/rslidar_pointcloud/src/rawdata.cc
                bool valid = it->range > min_range && it->range < max_range;
                if( !valid && !output_invalid_points ) { continue; }
                output o;
                o.t = p.first + boost::posix_time::microseconds( it->delay * 1000000 );
                o.id = it->id;
                o.scan = scan;
                o.range = it->range;
                o.bearing = it->azimuth;
                o.elevation = calculator.elevation()[ o.id ];
                o.reflectivity = it->reflectivity;
                o.coordinates = calculator.point( o.id, o.range, o.bearing );
                ostream.write( o );
            }
            // todo: update scan
            // todo: timestamp by laser id (see https://github.com/RoboSense-LiDAR/ros_rslidar/blob/develop-curves-function/rslidar_pointcloud/src/rawdata.cc)
            // todo: discard invalid scans
        }        
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "robosense-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "robosense-to-csv: unknown exception" << std::endl; }
    return 1;
}
