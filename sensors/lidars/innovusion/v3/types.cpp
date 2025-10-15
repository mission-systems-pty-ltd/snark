// Copyright (c) 2025 Mission Systems Pty Ltd

#include "types.h"

namespace snark { namespace innovusion {

static boost::posix_time::ptime inno_time_to_ptime( InnoTimestampUs start_of_frame, uint16_t offset_10us, int64_t timeframe_offset_us )
{
    int64_t timestamp_us = static_cast< int64_t >( start_of_frame ) + static_cast< int64_t >( offset_10us ) * 10 + timeframe_offset_us;

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

frame_t::frame_t( const InnoDataPacket* frame )
    : idx( frame->idx )
    , sub_idx( frame->sub_idx )
    , sub_seq( frame->sub_seq )
    , ts_us_start( frame->common.ts_start_us )
      //, ts_us_end( frame->ts_us_end )
      //, points_number( frame->points_number )
    , conf_level( frame->confidence_level )
    , timestamp_sync( frame->common.timestamp_sync_type )
{}

point_t::point_t()
    : x( 0 )
    , y( 0 )
    , z( 0 )
    , radius( 0 )
    , ts_10us( 0 )
    , value( 0 )
    , flags( 0 )
    , channel( 0 )
    , scan_id( 0 )
    , scan_idx( 0 )
{}

point_t::point_t( const InnoXyzPoint* point )
    : x( point->x )
    , y( point->y )
    , z( point->z )
    , radius( point->radius )
    , ts_10us( point->ts_10us )
    , value( point->refl )               // reflectance or intensity: 1-255
      //, flags( point->flags )
    , channel( point->channel )
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

output_data_t::output_data_t( const InnoDataPacket* frame, unsigned int index, int64_t timeframe_offset_us )
    : block( frame->idx )
    , x( frame->xyz_points[index].x )
    , y( frame->xyz_points[index].y )
    , z( frame->xyz_points[index].z )
    , radius( frame->xyz_points[index].radius )
    , value( frame->xyz_points[index].refl )
{
    t = inno_time_to_ptime( frame->common.ts_start_us, frame->xyz_points[index].ts_10us, timeframe_offset_us );
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

output_data_full_t::output_data_full_t( const InnoDataPacket* frame_, unsigned int index, int64_t timeframe_offset_us )
    : block( frame_->idx )
    , frame( frame_ )
    , point( &(frame_->xyz_points[index]) )
{
    t = inno_time_to_ptime( frame_->common.ts_start_us, frame_->xyz_points[index].ts_10us, timeframe_offset_us );
}

const std::string message_level_to_string( InnoMessageLevel level )
{
    switch( level )
    {
        case INNO_MESSAGE_LEVEL_FATAL:    return "FATAL";
        case INNO_MESSAGE_LEVEL_CRITICAL: return "CRITICAL";
        case INNO_MESSAGE_LEVEL_ERROR:    return "ERROR";
        case INNO_MESSAGE_LEVEL_TEMP:     return "Temp";
        case INNO_MESSAGE_LEVEL_WARNING:  return "Warning";
        case INNO_MESSAGE_LEVEL_DEBUG:    return "Debug";
        case INNO_MESSAGE_LEVEL_INFO:     return "Info";
        case INNO_MESSAGE_LEVEL_TRACE:    return "Trace";
        case INNO_MESSAGE_LEVEL_DETAIL:   return "Detail";
        case INNO_MESSAGE_LEVEL_MAX:      return "Max";
        default: return "unknown";
    }
}

const std::string message_code_to_string( InnoMessageCode code )
{
    switch( code )
    {
        case INNO_MESSAGE_CODE_NONE: return "None";
        case INNO_MESSAGE_CODE_LIB_VERSION_MISMATCH: return "Lib Version Mismatch";
        case INNO_MESSAGE_CODE_READ_TIMEOUT: return "Read Timeout";
        case INNO_MESSAGE_CODE_CANNOT_READ: return "Cannot Read";
        case INNO_MESSAGE_CODE_BAD_CONFIG_YAML: return "Bad Config Yaml";
        case INNO_MESSAGE_CODE_OVERHEAT_PROTECTION: return "Overheat Protection";
        case INNO_MESSAGE_CODE_TO_NON_WORKING_MODE: return "To Non Working Mode";
        case INNO_MESSAGE_CODE_READ_FILE_END: return "Read File End";
        case INNO_MESSAGE_CODE_RAW_RECORDING_FINISHED: return "Raw Recording Finished";
        case INNO_MESSAGE_CODE_NEW_START: return "New Start";
        case INNO_MESSAGE_CODE_ROI_CHANGED: return "ROI Changed";
        case INNO_MESSAGE_CODE_GALVO_MIRROR_CHECK_RESULT: return "Galvo Mirror Check Result";
        case INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT: return "Max Distance Check Result";
        default: return "unknown";
    }
};

const std::string confidence_level_to_string( InnoConfidenceLevel confidence_level )
{
    switch( confidence_level )
    {
        case INNO_NO_CONFIDENCE:   return "no confidence";
        case INNO_RARE_CONFIDENCE: return "rare confidence";
        case INNO_WELL_CONFIDENCE: return "well confidence";
        case INNO_FULL_CONFIDENCE: return "full confidence";
        default: return "unknown";
    }
};

const std::string timestamp_sync_to_string( InnoTimeSyncType timestamp_sync )
{
    switch( timestamp_sync )
    {
        case INNO_TIME_SYNC_TYPE_NONE: return "None";
        case INNO_TIME_SYNC_TYPE_RECORDED: return "Recorded";
        case INNO_TIME_SYNC_TYPE_HOST: return "Host";
        case INNO_TIME_SYNC_TYPE_GPS_INIT: return "GPS Init";
        case INNO_TIME_SYNC_TYPE_GPS_LOCKED: return "GPS Locked";
        case INNO_TIME_SYNC_TYPE_GPS_UNLOCKED: return "GPS Unlocked";
        case INNO_TIME_SYNC_TYPE_PTP_INIT: return "PTP Init";
        case INNO_TIME_SYNC_TYPE_PTP_LOCKED: return "PTP Locked";
        case INNO_TIME_SYNC_TYPE_PTP_UNLOCKED: return "PTP Unlocked";
        case INNO_TIME_SYNC_TYPE_FILE_INIT: return "File Init";
        case INNO_TIME_SYNC_TYPE_MAX: return "Max";
        default: return "unknown";
    }
};

} } // namespace snark { namespace innovusion {
