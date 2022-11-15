// Copyright (c) 2018-2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <array>
#include <cmath>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "calculator.h"

namespace snark { namespace robosense {

struct channel_num { std::array< double, 41 > data; };

} } // namespace snark { namespace robosense {

namespace comma { namespace visiting {

template <> struct traits< snark::robosense::channel_num >
{
    template < typename K, typename V > static void visit( const K& k,       snark::robosense::channel_num& t, V& v ) { v.apply( "data", t.data ); }
    template < typename K, typename V > static void visit( const K& k, const snark::robosense::channel_num& t, V& v ) { v.apply( "data", t.data ); }
};
    
} } // namespace comma { namespace visiting {

namespace snark { namespace robosense {
    
calculator::scan::scan( unsigned int max_number_of_missing_packets, bool accumulate )
    : max_number_of_missing_packets_( max_number_of_missing_packets )
    , max_time_gap_( boost::posix_time::microseconds( int( msop::data::block::firing_interval() * msop::data::number_of_blocks * 1000000 ) ) * max_number_of_missing_packets )
    , estimated_max_angle_gap_( 0 )
    , accumulate_( accumulate )
    , current_( 0 )
{
}

void calculator::scan::update( const boost::posix_time::ptime& timestamp, const msop::data& data )
{
    comma::uint32 angle = data.blocks[0].azimuth();
    comma::uint32 end_angle = data.blocks[ msop::data::number_of_blocks - 1 ].azimuth();
    if( end_angle < angle ) { end_angle += 36000; }
    estimated_max_angle_gap_ = ( ( end_angle - angle ) * msop::data::number_of_blocks / ( msop::data::number_of_blocks - 1 ) ) * max_number_of_missing_packets_; // todo: quick and dirty, watch performance
    if( scans_[current_].size == 0 )
    {
        scans_[current_].begin = angle;
        scans_[current_].end = angle;
        scans_[current_].size = 1;
    }
    else if( ( !last_timestamp_.is_not_a_date_time() && !timestamp.is_not_a_date_time() && timestamp - last_timestamp_ > max_time_gap_ )
        || angle - scans_[current_].end > estimated_max_angle_gap_ )
    {
        current_ = 1 - current_;
        scans_[current_].id = scans_[ 1 - current_ ].id + 1;
        scans_[current_].begin = angle;
        scans_[current_].end = angle;
        scans_[current_].size = 1;
    }
    else
    {
        scans_[current_].end = angle;
        ++scans_[current_].size;
    }
    if( !timestamp.is_not_a_date_time() ) { last_timestamp_ = timestamp; }
}

bool calculator::scan::is_complete( const data& d ) const
{
    if( !estimated_max_angle_gap_ ) { COMMA_THROW( comma::exception, "uninitialized" ); }
    return 36000 - ( d.end - d.begin ) < estimated_max_angle_gap_;
}

// static std::array< std::array< double, 41 >, robosens::msop::data::number_of_lasers > default_channel_num_ =
// {{
//     {{ 454,454,454,454,454,454,454,454,455,454,456,455,457,457,456,456,456,456,457,457,458,459,459,460,461,462,462,463,463,463,463,463,463,463,463,463,463,463,463,463,463 }},
//     {{ 459,459,459,459,459,459,459,459,459,459,460,459,461,460,460,460,460,460,461,462,463,463,464,465,465,466,467,467,467,467,467,467,467,467,467,467,467,467,467,467,467 }},
//     {{ 450,450,450,450,450,450,450,450,451,451,451,451,452,452,453,453,454,454,455,456,456,457,458,459,459,460,461,462,462,462,462,462,462,462,462,462,462,462,462,462,462 }},
//     {{ 451,451,451,451,451,451,451,451,452,451,452,452,452,453,453,453,454,454,455,456,457,458,459,460,460,461,462,463,463,463,463,463,463,463,463,463,463,463,463,463,463 }},
//     {{ 452,452,452,452,452,452,452,452,452,452,453,452,454,454,455,455,456,453,452,452,453,454,454,455,456,457,458,457,457,457,457,457,457,457,457,457,457,457,457,457,457 }},
//     {{ 451,451,451,451,451,451,451,451,451,451,452,451,453,452,453,453,454,452,451,451,452,452,453,454,455,456,456,456,456,456,456,456,456,456,456,456,456,456,456,456,456 }},
//     {{ 462,462,462,462,462,462,462,462,463,463,464,463,465,465,465,466,466,464,463,463,464,465,465,466,467,468,469,468,468,468,468,468,468,468,468,468,468,468,468,468,468 }},
//     {{ 461,461,461,461,461,461,461,461,462,461,462,462,464,463,464,464,465,463,462,462,463,464,465,466,467,467,468,467,467,467,467,467,467,467,467,467,467,467,467,467,467 }},
//     {{ 452,452,452,452,452,452,452,452,452,452,453,453,453,454,454,453,450,449,449,450,451,452,452,453,453,451,451,451,451,451,451,451,451,451,451,451,451,451,451,451,451 }},
//     {{ 467,467,467,467,467,467,467,467,468,467,468,468,469,469,470,468,466,465,465,466,467,468,468,468,469,467,467,468,468,468,468,468,468,468,468,468,468,468,468,468,468 }},
//     {{ 454,454,454,454,454,454,454,454,455,455,455,456,456,457,458,458,459,459,459,459,459,460,460,461,462,463,464,464,464,464,464,464,464,464,464,464,464,464,464,464,464 }},
//     {{ 461,461,461,461,461,461,461,461,461,461,462,462,463,463,464,464,465,466,466,466,466,467,467,468,469,469,470,471,471,471,471,471,471,471,471,471,471,471,471,471,471 }},
//     {{ 450,450,450,450,450,450,450,450,451,450,451,450,452,451,452,453,454,454,455,456,457,458,458,459,460,459,458,458,458,458,458,458,458,458,458,458,458,458,458,458,458 }},
//     {{ 455,455,455,455,455,455,455,455,455,455,456,455,457,457,457,458,459,459,460,462,463,464,463,464,465,465,464,463,463,463,463,463,463,463,463,463,463,463,463,463,463 }},
//     {{ 459,459,459,459,459,459,459,459,460,460,461,460,462,461,462,463,464,464,465,467,468,469,468,470,471,471,469,469,469,469,469,469,469,469,469,469,469,469,469,469,469 }},
//     {{ 450,450,450,450,450,450,450,450,451,451,452,452,453,453,454,454,455,455,456,458,459,460,460,461,462,461,460,459,459,459,459,459,459,459,459,459,459,459,459,459,459 }}
// }};
                                                      
void calculator::init_lasers_()
{
    for( unsigned int j = 0; j < robosense::msop::data::number_of_lasers; ++j ) { lasers_[j] = laser_( j, elevation_ ); }
}

calculator::calculator( const angles_t& azimuth, const angles_t& elevation, double range_resolution, double zero_angle_offset )
    : azimuth_( azimuth )
    , elevation_( elevation )
    , range_resolution_( range_resolution )
    , zero_angle_offset_( zero_angle_offset )
{
    init_lasers_();
}

double calculator::range( unsigned int r, unsigned int laser, unsigned int temperature ) const { return range_resolution_ * ( channel_num_ ? r - ( *channel_num_ )[laser][temperature] : r ); }

::Eigen::Vector3d calculator::to_cartesian( unsigned int laser, double range, double angle ) const
{
    return ::Eigen::Vector3d( range * lasers_[laser].cos * std::sin( angle )
                            , range * lasers_[laser].cos * std::cos( angle )
                            , range * lasers_[laser].sin );
}

double calculator::intensity( unsigned int, unsigned char intensity, double ) const { return intensity; }

calculator::point calculator::make_point( comma::uint32 scan, const boost::posix_time::ptime& t, const robosense::msop::const_iterator& it, unsigned int temperature )
{
    calculator::point p;
    p.scan = scan;
    p.t = t + boost::posix_time::microseconds( int( it->delay * 1000000 ) );
    p.id = it->id;
    p.range = range( it->range, it->id, temperature );
    p.bearing = it->azimuth + azimuth_[ it->id ];
    // p.bearing = it->azimuth + azimuth_[ it->id ] + zero_angle_offset_; // todo? try subtracting it instead of adding?
    // if( p.bearing < 0 ) { p.bearing += M_PI * 2; } else if( p.bearing > M_PI * 2 ) { p.bearing -= M_PI * 2; } // todo? can it be (because of zero_angle_offset)? should we subtract 2pi? should we check for negative offset, too? should we add offset instead of subtracting it?
    p.elevation = elevation_[ it->id ];
    p.reflectivity = it->reflectivity;
    p.coordinates = to_cartesian( it->id, p.range, p.bearing );
    return p;
}

template < typename Lidar >
static calculator::angles_t _load( const std::string& filename, const std::string& what )
{
    calculator::angles_t angles;
    std::ifstream ifs( filename );
    if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
    unsigned int i = 0;
    for( ; !ifs.eof() && ifs.good() && i < Lidar::msop::data::number_of_lasers; )
    {
        std::string line;
        std::getline( ifs, line );
        line = comma::strip( line );
        if( line.empty() ) { continue; }
        angles[i] = boost::lexical_cast< double >( line ) * M_PI / 180;
        ++i;
    }
    if( i < Lidar::msop::data::number_of_lasers ) { COMMA_THROW( comma::exception, "expected " << Lidar::msop::data::number_of_lasers << " " << what << " angle values in '" << filename << "'; got only " << i ); }
    return angles;
}

template < typename Lidar >
calculator calculator::make( const std::string& azimuth_filename, const std::string& elevation_filename, const std::string& channel_num_filename, const boost::optional< double >& range_resolution )
{
    auto azimuth = azimuth_filename.empty() ? Lidar::difop::data::corrected_horizontal_angles_default() : _load< Lidar >( azimuth_filename, "azimuth" );
    auto elevation = elevation_filename.empty() ? Lidar::difop::data::corrected_vertical_angles_default() : _load< Lidar >( elevation_filename, "elevation" );
    calculator c( azimuth, elevation, range_resolution ? *range_resolution : Lidar::range_resolution_default() );
    if( !channel_num_filename.empty() )
    {
        std::ifstream ifs( channel_num_filename );
        if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << channel_num_filename << "'" ); }
        comma::csv::input_stream< robosense::channel_num > is( ifs );
        c.channel_num_ = channel_num_t_();
        for( unsigned int i = 0; !ifs.eof() && ifs.good() && i < Lidar::msop::data::number_of_lasers; ++i )
        {
            const auto* p = is.read();
            if( !p ) { COMMA_THROW( comma::exception, "expected " << Lidar::msop::data::number_of_lasers << " channel num arrays in '" << channel_num_filename << "'; got only " << i ); }
            ( *c.channel_num_ )[i] = p->data;
        }
    }
    return c;
}

template calculator calculator::make< lidar_16 >( const std::string& azimuth, const std::string& elevation, const std::string& num_channels, const boost::optional< double >& range_resolution );
template calculator calculator::make< helios_16p >( const std::string& azimuth, const std::string& elevation, const std::string& num_channels, const boost::optional< double >& range_resolution );

} } // namespace snark { namespace robosense {
