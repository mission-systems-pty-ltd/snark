// Copyright (c) 2018-2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <memory.h>
#include <cmath>
#include <comma/base/exception.h>
#include <boost/tuple/tuple.hpp>
#include "packet.h"

namespace snark { namespace robosense {

std::pair< double, double > msop::data::azimuths( unsigned int block ) const // quick and dirty
{
    double t = blocks[ block ].azimuth_as_radians();
    double r;
    double d;
    if( block + 1 == number_of_blocks ) // quick and dirty; watch precision
    {
        double s = blocks[ block - 1 ].azimuth_as_radians();
        if( t < s ) { t += M_PI * 2; }
        d = ( t - s ) / 2;
    }
    else
    {
        double s = blocks[ block + 1 ].azimuth_as_radians();
        if( s < t ) { s += M_PI * 2; }
        d = ( s - t ) / 2;
    }
    r = t + d;
    if( r > M_PI * 2 ) { r -= M_PI * 2; }
    return std::make_pair( t, r );
}

static double azimuth_step_( const std::pair< double, double >& azimuths )
{ 
    return ( ( azimuths.first > azimuths.second ? azimuths.second + M_PI * 2 : azimuths.second ) - azimuths.first ) * ( msop::data::laser_return::firing_interval() / msop::data::block::firing_interval() );
}

msop::const_iterator::const_iterator() : data_( nullptr ), done_( true ) {}

msop::const_iterator::const_iterator( const msop::data& d )
    : data_( &d )
    , block_( 0 )
    , subblock_( 0 )
    , done_( false )
{
    update_value_( data_->azimuths( block_ ) );
}

void msop::const_iterator::update_value_()
{
    const msop::data::laser_return& r = data_->blocks[block_].channels[subblock_][value_.id];
    value_.range = r.range();
    value_.reflectivity = r.reflectivity(); // todo: apply curves
    value_.delay = msop::data::laser_return::firing_interval() * value_.id + ( subblock_ == 0 ? 0.0 : msop::data::block::firing_interval() / 2 ); // quick and dirty; could use 
    value_.azimuth += value_.azimuth_step;
}

void msop::const_iterator::update_value_( double azimuth )
{
    update_value_();
    value_.azimuth = azimuth;
}

void msop::const_iterator::update_value_( const std::pair< double, double >& azimuths )
{
    update_value_( azimuths.first );
    value_.azimuth_step = robosense::azimuth_step_( azimuths );
}

void msop::const_iterator::operator++()
{
    ++value_.id;
    if( value_.id < msop::data::number_of_lasers ) { update_value_(); return; }
    ++subblock_;
    value_.id = 0;
    if( subblock_ < msop::data::number_of_subblocks ) { update_value_( data_->azimuths( block_ ).second ); return; }
    subblock_ = 0;
    ++block_;
    if( block_ == msop::data::number_of_blocks ) { done_ = true; return; }
    update_value_( data_->azimuths( block_ ) );
}

bool msop::const_iterator::value_type::valid() const
{
    static unsigned int min_range = 2; // as in https://github.com/RoboSense-LiDAR/ros_rslidar/blob/develop-curves-function/rslidar_pointcloud/src/rawdata.cc
    static unsigned int max_range = 20000; // as in https://github.com/RoboSense-LiDAR/ros_rslidar/blob/develop-curves-function/rslidar_pointcloud/src/rawdata.cc
    return range > min_range && range < max_range;
}

template < unsigned int Size >
static std::array< char, Size > make_zeroes() { std::array< char, Size > a; ::memset( &a[0], 0, a.size() ); return a; }

const std::map< std::string, models::values > models::names =   {
                                                                    { "lidar-16", lidar_16 },
                                                                    { "lidar-32", lidar_32 },
                                                                    { "bpearl", bpearl },
                                                                    { "ruby", ruby },
                                                                    { "ruby-lite", ruby_lite },
                                                                    { "helios-5515", helios_5515 },
                                                                    { "helios-16p", helios_16p },
                                                                    { "helios-1615", helios_1615 }
                                                                    
                                                                };

std::string models::to_string( models::values value )
{
    switch( value )
    {
        case lidar_16: return "lidar-16";
        case lidar_32: return "lidar-32";
        case bpearl: return "bpearl";
        case ruby: return "ruby";
        case ruby_lite: return "ruby-lite";
        case helios_5515: return "helios-5515";
        case helios_16p: return "helios-16xx";
    }
    COMMA_THROW( comma::exception, "expected robosense lidar model code; got: \"" << value << "\"" ); // never here really
}

models::values models::from_string( const std::string& name )
{
    auto it = names.find( name );
    if( it == names.end() ) { COMMA_THROW( comma::exception, "expected robosense lidar model name; got: \"" << name << "\"" ); }
    return it->second;
}

models::values msop::detect_model( const char* header_bytes )
{
    if( reinterpret_cast< const lidar_16::msop::header* >( header_bytes )->sentinel() == lidar_16::msop::header::sentinel_value() ) { return model_value( *reinterpret_cast< const lidar_16::msop::header* >( header_bytes ) ); }
    if( reinterpret_cast< const helios_16p::msop::header* >( header_bytes )->sentinel() == helios_16p::msop::header::sentinel_value() ) { return model_value( *reinterpret_cast< const helios_16p::msop::header* >( header_bytes ) ); }
    COMMA_THROW( comma::exception, "could not detect model, since the header sentinel does not match sentinels of supported models (lidar-16 and helios-16p)" );
}

bool lidar_16::difop::data::corrected_angles::empty() const
{
    const char* p = values[0].data(); // as in ros driver
    for( unsigned int i = 0; i < 4; ++i ) { if( p[i] != 0x00 && p[i] != 0xff ) { return false; } } // as in ros driver
    return true;
    
    //enum { size_in_bytes = msop::data::number_of_lasers * difop::packet::data_t::corrected_vertical_angle::size };
    //static std::array< char, size_in_bytes > zeroes = make_zeroes< size_in_bytes >();
    //return ::memcmp( corrected_vertical_angles.data(), &zeroes[0], size_in_bytes ) == 0;
}

double lidar_16::difop::data::top_board_firmware_version_t::range_resolution() const
{
    auto d = value.data();
    return ( d[1] == 0x00 && d[2] == 0x00 && d[3] == 0x00 ) || ( d[1] == 0xff && d[2] == 0xff && d[3] == 0xff ) || ( d[1] == 0x55 && d[2] == 0xaa && d[3] == 0x5a ) ? 0.01 : 0.005;
}

const std::array< double, robosense::msop::data::number_of_lasers >& lidar_16::difop::data::default_corrected_vertical_angles()
{
    static std::array< double, robosense::msop::data::number_of_lasers > default_elevation_ = { { -15. * M_PI / 180
                                                                                                , -13. * M_PI / 180
                                                                                                , -11. * M_PI / 180
                                                                                                ,  -9. * M_PI / 180
                                                                                                ,  -7. * M_PI / 180
                                                                                                ,  -5. * M_PI / 180
                                                                                                ,  -3. * M_PI / 180
                                                                                                ,  -1. * M_PI / 180
                                                                                                ,  15. * M_PI / 180
                                                                                                ,  13. * M_PI / 180
                                                                                                ,  11. * M_PI / 180
                                                                                                ,   9. * M_PI / 180
                                                                                                ,   7. * M_PI / 180
                                                                                                ,   5. * M_PI / 180
                                                                                                ,   3. * M_PI / 180
                                                                                                ,   1. * M_PI / 180 } };
    return default_elevation_;
}

double helios_16p::difop::data::corrected_angles::angle::radians() const { return 0.01 * value() * ( sign() == 0 ? 1. : -1. ) * M_PI / 180; } // helios-16p spec B.10

// double helios_16p::difop::data::corrected_angles::angle::radians() const
// { 
//     std::cerr << "==> sign: " << sign() << " angle raw: " << value() << std::endl;
//     return 0.01 * value() * ( sign() == 0 ? 1. : -1. );
// } // helios-16p spec B.10

bool helios_16p::difop::data::corrected_angles::empty() const // for now, copied from lidar_16; helios-16p documentation says nothing about it
{
    const char* p = values[0].data(); // as in ros driver
    for( unsigned int i = 0; i < 4; ++i ) { if( p[i] != 0x00 && p[i] != 0xff ) { return false; } } // as in ros driver
    return true;
    
    //enum { size_in_bytes = msop::data::number_of_lasers * difop::packet::data_t::corrected_vertical_angle::size };
    //static std::array< char, size_in_bytes > zeroes = make_zeroes< size_in_bytes >();
    //return ::memcmp( corrected_vertical_angles.data(), &zeroes[0], size_in_bytes ) == 0;
}

const std::array< double, robosense::msop::data::number_of_lasers >& helios_16p::difop::data::default_corrected_vertical_angles()
{
    static std::array< double, robosense::msop::data::number_of_lasers > default_elevation_ = { {  13. * M_PI / 180
                                                                                                ,  15. * M_PI / 180
                                                                                                ,   9. * M_PI / 180
                                                                                                ,  11. * M_PI / 180
                                                                                                ,   5. * M_PI / 180
                                                                                                ,   7. * M_PI / 180
                                                                                                ,   1. * M_PI / 180
                                                                                                ,   3. * M_PI / 180
                                                                                                ,  -3. * M_PI / 180
                                                                                                ,  -1. * M_PI / 180
                                                                                                ,  -7. * M_PI / 180
                                                                                                ,  -5. * M_PI / 180
                                                                                                , -11. * M_PI / 180
                                                                                                ,  -9. * M_PI / 180
                                                                                                , -15. * M_PI / 180
                                                                                                , -13. * M_PI / 180 } };
    return default_elevation_;
}

} } // namespace snark { namespace robosense {
