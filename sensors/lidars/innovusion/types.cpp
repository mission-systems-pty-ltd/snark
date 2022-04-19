// Copyright (c) 2021 Mission Systems Pty Ltd

#include "types.h"

namespace snark { namespace innovusion {

static boost::posix_time::ptime inno_time_to_ptime( inno_timestamp_us_t start_of_frame, uint16_t offset_100us, int64_t timeframe_offset_us )
{
    int64_t timestamp_us = static_cast< int64_t >( start_of_frame ) + static_cast< int64_t >( offset_100us ) * 100 + timeframe_offset_us;

    return boost::posix_time::from_time_t( timestamp_us / 1000000 ) + boost::posix_time::microseconds( timestamp_us % 1000000 );
}

frame_t::frame_t()
    : idx( 0 )
    , sub_idx( 0 )
    , sub_seq( 0 )
    , ts_us_start( 0 )
    , ts_us_end( 0 )
    , points_number( 0 )
    , conf_level( 0 )
    , timestamp_sync( 0 )
{}

frame_t::frame_t( const inno_frame* frame )
    : idx( frame->idx )
    , sub_idx( frame->sub_idx )
    , sub_seq( frame->sub_seq )
    , ts_us_start( frame->ts_us_start )
    , ts_us_end( frame->ts_us_end )
    , points_number( frame->points_number )
    , conf_level( frame->conf_level )
    , timestamp_sync( frame->timestamp_sync_type )
{}

point_t::point_t()
    : x( 0 )
    , y( 0 )
    , z( 0 )
    , radius( 0 )
    , ts_100us( 0 )
    , value( 0 )
    , flags( 0 )
    , channel( 0 )
    , scan_id( 0 )
    , scan_idx( 0 )
{}

point_t::point_t( const inno_point* point )
    : x( point->x )
    , y( point->y )
    , z( point->z )
    , radius( point->radius )
    , ts_100us( point->ts_100us )
    , value( point->ref )                // reflectance or intensity: 1-255
    , flags( point->flags )
    , channel( point->flags & 0x01 )
    , scan_id( point->scan_id )
    , scan_idx( point->scan_idx )
{}

output_data_t::output_data_t()
    : block( 0 )
    , x( 0 )
    , y( 0 )
    , z( 0 )
    , radius( 0 )
    , value( 0 )
{}

output_data_t::output_data_t( const inno_frame* frame, unsigned int index, int64_t timeframe_offset_us )
    : block( frame->idx )
    , x( frame->points[index].x )
    , y( frame->points[index].y )
    , z( frame->points[index].z )
    , radius( frame->points[index].radius )
    , value( frame->points[index].ref )
{
    t = inno_time_to_ptime( frame->ts_us_start, frame->points[index].ts_100us, timeframe_offset_us );
}

template< typename T >
void add_to_buf( T v, char** buf )
{
    comma::csv::format::traits< T, comma::csv::format::type_to_enum< T >::value >::to_bin( v, *buf );
    *buf += comma::csv::format::traits< T, comma::csv::format::type_to_enum< T >::value >::size;
}

void output_data_t::to_bin( char* buf ) const
{
    add_to_buf< boost::posix_time::ptime >( t, &buf );
    add_to_buf< comma::uint32 >( block, &buf );
    add_to_buf< float >( x, &buf );
    add_to_buf< float >( y, &buf );
    add_to_buf< float >( z, &buf );
    add_to_buf< float >( radius, &buf );
    add_to_buf< comma::uint16 >( value, &buf );
}

output_data_full_t::output_data_full_t()
    : block( 0 )
{}

output_data_full_t::output_data_full_t( const inno_frame* frame_, unsigned int index, int64_t timeframe_offset_us )
    : block( frame_->idx )
    , frame( frame_ )
    , point( &(frame_->points[index]) )
{
    t = inno_time_to_ptime( frame_->ts_us_start, frame_->points[index].ts_100us, timeframe_offset_us );
}

std::string alarm_type_to_string( inno_alarm alarm_type )
{
    switch( alarm_type )
    {
        case INNO_ALARM_NO: return "None";
        case INNO_ALARM_WARNING: return "Warning";
        case INNO_ALARM_ERROR: return "Error";
        case INNO_ALARM_CRITICAL: return "CRITICAL";
        case INNO_ALARM_FATAL: return "FATAL";
        default: return "unknown";
    }
}

std::string alarm_code_to_string( inno_alarm_code alarm_code )
{
    switch( alarm_code )
    {
        case INNO_ALARM_CODE_NO: return "No";
        case INNO_ALARM_CODE_CANNOT_READ: return "Cannot Read";
        case INNO_ALARM_CODE_INVALID_DATA_TYPE: return "Invalid Data Type";
        case INNO_ALARM_CODE_TRIGGER_BUFFER_FULL: return "Trigger Buffer Full";
        case INNO_ALARM_CODE_PULSE_BUFFER_FULL: return "Pulse Buffer Full";
        case INNO_ALARM_CODE_LOW_FRAME_RATE: return "Low Frame Rate";
        case INNO_ALARM_CODE_HIGH_FRAME_RATE: return "High Frame Rate";
        case INNO_ALARM_CODE_SLOW_NETWORK: return "Slow Network";
        case INNO_ALARM_CODE_LOW_FRAME_INTEGRITY: return "Low Frame Integrity";
        case INNO_ALARM_CODE_POINT_DATA_DROP: return "Point Data Drop";
        case INNO_ALARM_CODE_FRAME_DATA_DROP: return "Frame Data Drop";
        case INNO_ALARM_CODE_READ_TIMEOUT: return "Read Timeout";
        case INNO_ALARM_CODE_DROP_DATA_1: return "Drop Data 1";
        case INNO_ALARM_CODE_DROP_DATA_2: return "Drop Data 2";
        case INNO_ALARM_CODE_DROP_DATA_3: return "Drop Data 3";
        case INNO_ALARM_CODE_DROP_DATA_4: return "Drop Data 4";
        case INNO_ALARM_CODE_SENSOR_ERROR: return "Sensor Error";
        case INNO_ALARM_CODE_BAD_CONFIG_YAML: return "Bad Config Yaml";
        case INNO_ALARM_CODE_NO_POINT_MEMORY: return "No Point Memory";
        case INNO_ALARM_CODE_FRAME_CALL_TOO_LONG: return "Frame Call Too Long";
        case INNO_ALARM_CODE_TEMP_TOO_LOW: return "Temp Too Low";
        case INNO_ALARM_CODE_TEMP_TOO_HIGH: return "Temp Too High";
        case INNO_ALARM_CODE_LIB_VERSION_MISMATCH: return "Lib Version Mismatch";
        case INNO_ALARM_CODE_CLOCK_DRIFT: return "Clock Drift";
        case INNO_ALARM_CODE_CORRUPT_DATA: return "Corrupt Data";
        case INNO_ALARM_CODE_OUT_OF_MEMORY: return "Out Of Memory";
        case INNO_ALARM_CODE_DROP_DATA_FILTER: return "Drop Data Filter";
        case INNO_ALARM_CODE_FILTER_TOO_LONG: return "Filter Too Long";
        default: return "unknown";
    }
};

std::string timestamp_sync_to_string( inno_timestamp_sync timestamp_sync )
{
    switch( timestamp_sync )
    {
        case INNO_TIMESTAMP_SYNC_NONE: return "None";
        case INNO_TIMESTAMP_SYNC_RECORDED: return "Recorded";
        case INNO_TIMESTAMP_SYNC_HOST: return "Host";
        case INNO_TIMESTAMP_SYNC_GPS_INIT: return "GPS Init";
        case INNO_TIMESTAMP_SYNC_GPS_LOCKED: return "GPS Locked";
        case INNO_TIMESTAMP_SYNC_GPS_UNLOCKED: return "GPS Unlocked";
        case INNO_TIMESTAMP_SYNC_PTP_INIT: return "PTP Init";
        case INNO_TIMESTAMP_SYNC_PTP_LOCKED: return "PTP Locked";
        case INNO_TIMESTAMP_SYNC_PTP_UNLOCKED: return "PTP Unlocked";
        case INNO_TIMESTAMP_SYNC_FILE_INIT: return "File Init";
        case INNO_TIMESTAMP_SYNC_MAX: return "Max";
        default: return "unknown";
    }
};

} } // namespace snark { namespace innovusion {
