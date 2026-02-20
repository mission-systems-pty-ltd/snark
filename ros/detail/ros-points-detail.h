#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <thread>
#include <unordered_map>

#include "../file-util.h"

#ifdef SNARK_BUILD_ROS_1
    #include <ros/ros.h>
    #include <rosbag/bag.h>
    #include <rosbag/view.h>
    #include <sensor_msgs/PointCloud2.h>
#endif

#ifdef SNARK_BUILD_ROS_2
    #include "../ros2/rclcpp/time.h"
#endif

#include "../version.h"

// TODO: Use this file in ros-points too.
#if ROS_VERSION_MINIMUM(2,0,0)
    namespace snark_ros_sensor_msgs = sensor_msgs::msg;
#else
    namespace snark_ros_sensor_msgs = sensor_msgs;
#endif

static bool status = 0; // quick and dirty

namespace snark { namespace ros { namespace pointcloud {

enum class time_format_enum { none, offset_seconds, offset_nanoseconds };

static time_format_enum string_to_time_format( const std::string& time_format_str )
{
    if( time_format_str == "none" ) { return time_format_enum::none; }
    if( time_format_str == "offset-seconds" ) { return time_format_enum::offset_seconds; }
    if( time_format_str == "offset-nanoseconds" ) { return time_format_enum::offset_nanoseconds; }
    COMMA_THROW( comma::exception, "unknown time format \"" << time_format_str << "\"" );
}
struct record
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    std::vector< char > data;
    record() : block( 0 ) {}
};

std::size_t size_of_type( comma::csv::format::types_enum t )
{
    switch( t )
    {
        case comma::csv::format::char_t: return sizeof( char );
        case comma::csv::format::int8: return sizeof( char );
        case comma::csv::format::uint8: return sizeof( unsigned char );
        case comma::csv::format::int16: return sizeof( int16_t );
        case comma::csv::format::uint16: return sizeof( uint16_t );
        case comma::csv::format::int32: return sizeof( int32_t );
        case comma::csv::format::uint32: return sizeof( uint32_t );
        case comma::csv::format::int64: return sizeof( int64_t );
        case comma::csv::format::uint64: return sizeof( uint64_t );
        case comma::csv::format::float_t: return sizeof( float );
        case comma::csv::format::double_t: return sizeof( double );
        case comma::csv::format::time: return sizeof( int64_t );
        case comma::csv::format::long_time: return sizeof( int64_t ) + sizeof( int32_t );
        case comma::csv::format::fixed_string: return 0; // will it blast somewhere?
        default: { COMMA_THROW( comma::exception, "invalid type " << unsigned( t )); }
    }
}

// Interface class for accessing the pointcloud data. Allowing configuration and
// then interrogation of the ROS bag pointcloud data and returning data as per
// the configuration. Provides:
//
//     constructor - configures the class for required transformation
//     get( ptr to ros bag record ) - return transformed data from the given record
//     size() - size of an output record
//
// Transformations can be anything but typically are:
//     selection of a subset of fields
//     re-ordering of fields
//     some processing of data (e.g. converting time offsets to absolute time)
class bin_base
{
public:
    virtual ~bin_base() {}
    virtual const char* get( const char* data ) = 0;
    virtual std::size_t size() const = 0;
};

// Direct copy of ros bag data to output
class bin_cat : public bin_base
{
public:
    bin_cat( uint32_t s = 0 ) : size_( s ) { }
    const char* get( const char* data ){ return data; }
    std::size_t size() const { return size_; }

private:
    uint32_t size_;
};

} } } //  namespace pointcloud { namespace snark { namespace ros {

/// utility functions for ros sensor_msgs::msg::PointCloud2
namespace snark { namespace ros { namespace detail {

template < unsigned int Version > struct traits;

#ifdef SNARK_BUILD_ROS_1
    template <> struct traits< 1 >
    {
        typedef sensor_msgs::PointCloud2 PointCloud2;
        typedef sensor_msgs::PointCloud2ConstPtr PointCloud2Ptr;
        typedef sensor_msgs::PointField PointField;
        typedef ::ros::Time Time;
        static const auto INT8 = sensor_msgs::PointField::INT8;
        static const auto UINT8 = sensor_msgs::PointField::UINT8;
        static const auto INT16 = sensor_msgs::PointField::INT16;
        static const auto UINT16 = sensor_msgs::PointField::UINT16;
        static const auto INT32 = sensor_msgs::PointField::INT32;
        static const auto UINT32 = sensor_msgs::PointField::UINT32;
        static const auto FLOAT32 = sensor_msgs::PointField::FLOAT32;
        static const auto FLOAT64 = sensor_msgs::PointField::FLOAT64;
        static void shutdown() { ::ros::shutdown(); }
        // time_to_boost
        static boost::posix_time::ptime time_to_boost( const Time& t )
        {
            return t.toBoost();
        }
        // time from boost
        static Time time_from_boost( const boost::posix_time::ptime& t )
        {
            return ::ros::Time::fromBoost( t );
        }
        static void update_block( PointCloud2Ptr input, unsigned int& block_ )
        {
            block_ = input->header.seq;
        }
    };
#endif
#ifdef SNARK_BUILD_ROS_2
    template <> struct traits< 2 >
    {
        typedef sensor_msgs::msg::PointCloud2 PointCloud2;
        typedef sensor_msgs::msg::PointCloud2::SharedPtr PointCloud2Ptr;
        typedef sensor_msgs::msg::PointField PointField;
        typedef rclcpp::Time Time;
        static const auto INT8 = sensor_msgs::msg::PointField::INT8;
        static const auto UINT8 = sensor_msgs::msg::PointField::UINT8;
        static const auto INT16 = sensor_msgs::msg::PointField::INT16;
        static const auto UINT16 = sensor_msgs::msg::PointField::UINT16;
        static const auto INT32 = sensor_msgs::msg::PointField::INT32;
        static const auto UINT32 = sensor_msgs::msg::PointField::UINT32;
        static const auto FLOAT32 = sensor_msgs::msg::PointField::FLOAT32;
        static const auto FLOAT64 = sensor_msgs::msg::PointField::FLOAT64;
        static void shutdown() { rclcpp::shutdown(); }
        // time_to_boost
        static boost::posix_time::ptime time_to_boost( const Time& t )
        {
            return snark::ros::time::to_boost( t );
        }
        // time_from_boost
        static Time time_from_boost( const boost::posix_time::ptime& t )
        {
            return snark::ros::time::from_boost( t );
        }
        static void update_block( PointCloud2Ptr input, unsigned int& block_ )
        {
            block_++;
        }
    };
#endif
template < unsigned int Version >
struct pointcloud
{
    static const std::vector< comma::csv::format::types_enum >& get_rmap_data_type()
    {
        static std::vector< comma::csv::format::types_enum > rmap_data_type;

        if( rmap_data_type.size() == 0 )
        {
            rmap_data_type.resize( 9 );
            rmap_data_type.at( traits<Version>::INT8 ) = comma::csv::format::char_t;
            rmap_data_type.at( traits<Version>::UINT8 ) = comma::csv::format::uint8;
            rmap_data_type.at( traits<Version>::INT16 ) = comma::csv::format::int16;
            rmap_data_type.at( traits<Version>::UINT16 ) = comma::csv::format::uint16;
            rmap_data_type.at( traits<Version>::INT32 ) = comma::csv::format::int32;
            rmap_data_type.at( traits<Version>::UINT32 ) = comma::csv::format::uint32;
            rmap_data_type.at( traits<Version>::FLOAT32 ) = comma::csv::format::float_t;
            rmap_data_type.at( traits<Version>::FLOAT64 ) = comma::csv::format::double_t;
        }
        return rmap_data_type;
    }

    static std::vector< comma::csv::format::types_enum > padding_types( std::size_t num_bytes )
    {
        std::vector< comma::csv::format::types_enum > result;
        std::vector< comma::csv::format::types_enum > candidates = {
            comma::csv::format::uint64,
            comma::csv::format::uint32,
            comma::csv::format::uint16,
            comma::csv::format::uint8
        };
        for( auto candidate: candidates )
        {
            while( num_bytes >= comma::csv::format::size_of( candidate ))
            {
                result.push_back( candidate );
                num_bytes -= comma::csv::format::size_of( candidate );
            }
        }
        return result;
    }


    /// returns list of field names from the message
    static std::string msg_fields_names( const typename traits<Version>::PointCloud2::_fields_type& msg_fields
                                , const std::vector< std::string >& field_filter = std::vector< std::string >() )
    {
        if( !field_filter.empty() ) { return comma::join( field_filter, ',' ); }

        std::string s;
        std::string delimiter;
        std::size_t expected_offset = 0;
        static unsigned int padding_field_count = 0;
        const auto& rmap = get_rmap_data_type();
        for( const auto& f : msg_fields )
        {
            comma::csv::format::types_enum type = rmap.at( f.datatype );
            if( f.offset > expected_offset )
            {
                for( unsigned int i = 0; i < padding_types( f.offset - expected_offset ).size(); ++i )
                {
                    s += delimiter + "padding";
                    if( padding_field_count > 0 ) { s += boost::lexical_cast< std::string >( padding_field_count ); }
                    padding_field_count++;
                }
            }
            s += delimiter + f.name;
            expected_offset = f.offset + comma::csv::format::size_of( type ) * f.count;
            if( delimiter.empty() ) { delimiter = ","; }
        }
        return s;
    }

    /// returns csv format from the message, optionally filtered by field name
    static comma::csv::format msg_fields_format( const typename traits<Version>::PointCloud2::_fields_type& msg_fields
                                        , const std::vector< std::string >& field_filter = std::vector< std::string >()
                                        , bool unmap_time_fields = false )
    {
        comma::csv::format format;
        std::size_t expected_offset = 0;
        bool add_field;
        const auto& rmap = pointcloud< Version >::get_rmap_data_type();
        for( const auto& f : msg_fields )
        {
            comma::csv::format::types_enum type = rmap.at( f.datatype );
            if( unmap_time_fields && ( f.name == "t" || f.name == "time" )) { type = comma::csv::format::time; }
            if( field_filter.empty() )
            {
                if( f.offset > expected_offset )
                {
                    for( auto t: padding_types( f.offset - expected_offset ))
                    {
                        format += comma::csv::format::to_format( t );
                    }
                }
                expected_offset = f.offset + comma::csv::format::size_of( type ) * f.count;
                add_field = true;
            }
            else
            {
                add_field = ( std::find( field_filter.begin(), field_filter.end(), f.name ) != field_filter.end() );
            }
            if( add_field )
            {
                for( unsigned int i = 0; i < f.count; i++ ) { format += comma::csv::format::to_format( type ); }
            }
        }
        return format;
    }

    // copy specified fields from a point record, given msg point field info and a list of field names
    // also handle different time formats
    class bin_shuffle : public snark::ros::pointcloud::bin_base
    {
        //first: offset, second: size
        typedef typename std::pair< std::size_t, std::size_t > range_t;

        struct field_desc_t
        {
            range_t range;
            unsigned int datatype;
            bool is_time_field;

            field_desc_t() : datatype(0), is_time_field( false ) {}
            field_desc_t( range_t range, unsigned int datatype, bool is_time_field )
                : range( range ), datatype( datatype ), is_time_field( is_time_field ) {}
        };

    public:
        /// prepare
        bin_shuffle( const std::string& field_names
                , const typename traits<Version>::PointCloud2::_fields_type& msg_fields
                , const typename traits<Version>::Time header_time_stamp
                , bool ignore_time_format )
            : header_time_stamp( traits<Version>::time_to_boost( header_time_stamp ) )
            , ignore_time_format( ignore_time_format )
        {
            std::vector< std::string > fields = comma::split( field_names, "," );
            std::unordered_map< std::string, unsigned int > msg_field_name_map;
            std::vector< range_t > elements;
            const auto& rmap = pointcloud< Version >::get_rmap_data_type();
            std::size_t size = 0;
            for( std::size_t i = 0; i < msg_fields.size(); i++ )
            {
                msg_field_name_map[ msg_fields[i].name ] = i;
                if( msg_fields[i].datatype < 1 || msg_fields[i].datatype >= rmap.size() ) { COMMA_THROW( comma::exception, "datatype out of range (1 to 8) " << unsigned( msg_fields[i].datatype )); }
                comma::csv::format::types_enum type = rmap.at( msg_fields[i].datatype );
                //using comma::csv::format::size_of(type) corrupts stack
                //                 elements.push_back(range_t(msg_fields[i].offset, msg_fields[i].count * comma::csv::format::size_of(type)));
                elements.push_back( range_t( msg_fields[i].offset, msg_fields[i].count * snark::ros::pointcloud::size_of_type( type )));
            }
            for( const std::string& field_name : fields )
            {
                unsigned int index;
                try { index = msg_field_name_map.at( field_name ); }
                catch( std::out_of_range& ex ) { COMMA_THROW( comma::exception, "couldn't find " << field_name << " in msg_field_name_map" ); }
                bool is_time_field = ( field_name == "t" || field_name == "time" );
                field_descs.push_back( field_desc_t( elements[index], msg_fields[index].datatype, is_time_field ));
                if( is_time_field && !ignore_time_format ) { size += sizeof( boost::posix_time::ptime ); }
                else { size += elements[index].second; }
            }
            buf.resize( size );
        }

        /// shuffle
        const char* get( const char* data )
        {
            std::size_t offset = 0;
            for( const auto& field_desc : field_descs )
            {
                if( field_desc.is_time_field )
                {
                    // when reading a ros topic we deduce the time format by the data type,
                    // unless we have been explicitly told not to do any processing by --ignore-time-format
                    // that option is useful if you want to output whatever is in the time field and not interpret it
                    if( !ignore_time_format )
                    {
                        // is the time format either offset_seconds (float32) or offset_nanoseconds (uint32)?
                        if( field_desc.datatype == traits<Version>::PointField::FLOAT32 ||
                            field_desc.datatype == traits<Version>::PointField::UINT32 )
                        {
                            boost::posix_time::time_duration time_offset;
                            if( field_desc.datatype == traits<Version>::PointField::FLOAT32 )
                            {
                                float offset_seconds = comma::csv::format::traits< float >::from_bin( data + field_desc.range.first );
                                time_offset = boost::posix_time::microseconds( static_cast< long >( offset_seconds * 1000000 ));
                            }
                            else
                            {
                                comma::uint32 offset_nanoseconds = comma::csv::format::traits< comma::uint32 >::from_bin( data + field_desc.range.first );
                                // we're using the 64 bit boost ptime implementation, which is accurate to microseconds
                                time_offset = boost::posix_time::microseconds( offset_nanoseconds / 1000 );
                            }
                            boost::posix_time::ptime time = header_time_stamp + time_offset;
                            comma::csv::format::traits< boost::posix_time::ptime, comma::csv::format::time >::to_bin( time, &buf[offset] );
                            offset += sizeof( time );
                            continue;   // we've copied the data, go to the next iteration of the loop
                        }
                    }
                }
                std::memcpy( &buf[offset], data + field_desc.range.first, field_desc.range.second );
                offset += field_desc.range.second;
            }
            return buf.data();
        }

        std::size_t size() const { return buf.size(); }

    private:
        std::vector< char > buf;
        std::vector< field_desc_t > field_descs;
        boost::posix_time::ptime header_time_stamp;
        bool ignore_time_format;
    };

    struct header
    {
        boost::posix_time::ptime t;
        uint32_t block{0};
        header() : block( 0 ) {}
        header( const typename traits<Version>::Time& time, uint32_t seq ) : t( traits<Version>::time_to_boost( time ) ), block( seq ) {}
    };

    /// process ros traits<Version>::PointCloud2 message
    class points
    {
    public:
        points( const comma::command_line_options& options )
            : csv( options )
            , output_fields_option( options.exists( "--output-fields" ))
            , output_format_option( options.exists( "--output-format" ))
            , flush( options.exists( "--flush" ))
            , write_header( options.exists( "--header,--output-header" ))
            , discard( !options.exists( "--no-discard" ))
            , ignore_time_format( options.exists( "--ignore-time-format" ))
        {
            fields = comma::split( options.value< std::string >( "--fields", "" ), ',' );
            if( fields.size() == 1 && fields[0].empty() ) { fields.clear(); } // comma::split quirk
        }


        void process( const typename traits<Version>::PointCloud2Ptr input )
        {
            try
            {
                std::string pointcloud_fields = msg_fields_names( input->fields, fields );
                comma::csv::format pointcloud_format = msg_fields_format( input->fields, fields );
                comma::csv::format output_format = msg_fields_format( input->fields, fields, !ignore_time_format );

                if( output_fields_option )
                {
                    if( write_header ) { std::cout << comma::join( comma::csv::names< header >(), ',' ) << ","; }
                    std::cout << pointcloud_fields << std::endl;
                    traits<Version>::shutdown();
                    return;
                }
                if( output_format_option )
                {
                    if( write_header ) { std::cout << comma::csv::format::value< header >() << ","; }
                    // output as collapsed string, to match expectations
                    std::cout << output_format.collapsed_string() << std::endl;
                    traits<Version>::shutdown();
                    return;
                }
                if( format.count() == 0 )
                {
                    format = pointcloud_format;
                    comma::verbose << "setting format to " << format.string() << std::endl;
                }
                unsigned int count = input->width * input->height;
                unsigned int record_size = input->point_step;
                traits<Version>::update_block( input, block_ );
                header h( input->header.stamp, block_ );

                std::unique_ptr< snark::ros::pointcloud::bin_base > bin;
                // if we haven't selected a subset fields or re-arranged the fields;
                // and there are no time fields which might need intepretation:
                // we can do a straight copy from ros data to output
                // in practice this will rarely be the case
                bool has_time_field = comma::csv::fields_exist( pointcloud_fields, "t" ) || comma::csv::fields_exist( pointcloud_fields, "time" );
                if( csv.fields.empty() && !has_time_field )
                {
                    bin.reset( new snark::ros::pointcloud::bin_cat( record_size ));
                }
                else
                {
                    bin.reset( new bin_shuffle( csv.fields.empty() ? pointcloud_fields : csv.fields
                                                                    , input->fields, input->header.stamp, ignore_time_format ));
                }

                bin_writer writer;

                std::unique_ptr< filter_base > filter;
                if( discard ) { filter.reset( new float_filter( format )); }

                for( unsigned int i = 0; i < count; i++ )
                {
                    const char* buf = bin->get( reinterpret_cast< const char* >( &input->data[i * record_size] ));
                    if( !filter || filter->valid( buf, bin->size() ))
                    {
                        if( write_header ) { writer.write_header( h ); }
                        writer.write( buf, bin->size() );
                        if( flush ) { std::cout.flush(); }
                    }
                    if( !std::cout.good() ) { traits<Version>::shutdown(); break; }
                }
            }
            catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; status = 1; traits<Version>::shutdown(); }
            catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; status = 1; traits<Version>::shutdown(); }
        }

    private:
        struct bin_writer
        {
            comma::csv::binary< header > header_csv_bin;
            std::vector< char > header_buf;

            bin_writer() : header_buf( header_csv_bin.format().size() ) { }

            void write_header( const header& h )
            {
                std::cout.write( header_csv_bin.put( h, &header_buf[0] ), header_buf.size() );
            }

            void write( const char* buf, uint32_t size )
            {
                std::cout.write( buf, size );
            }
        };

        struct filter_base
        {
            virtual ~filter_base() { }
            virtual bool valid( const char* buf, uint32_t size ) = 0;
        };

        struct float_filter : public filter_base
        {
            const comma::csv::format& format;
            std::vector< std::size_t > offsets;

            float_filter( const comma::csv::format& format ) : format( format )
            {
                offsets.reserve( format.elements().size() );
                for( const auto& i : format.elements() )
                {
                    if( i.type == comma::csv::format::float_t )
                    {
                        if( i.count != 1 ) { COMMA_THROW( comma::exception, "expected format count 1, got " << i.count ); }
                        if( i.size != sizeof( float )) { COMMA_THROW( comma::exception, "expected format size " << sizeof( float ) << "; got" << i.size ); }
                        offsets.push_back( i.offset );
                    }
                }
            }

            bool valid( const char* buf, uint32_t size )
            {
                for( const std::size_t offset : offsets )
                {
                    if( offset + sizeof( float ) > size ) { COMMA_THROW( comma::exception, "offset out of range. offset: " << offset << " size: " << size ); }
                    float value = *reinterpret_cast< const float* >( buf + offset );
                    if( std::isnan( value ) || std::isinf( value )) { return false; }
                }
                return true;
            }
        };

        std::vector< std::string > fields;
        comma::csv::format format;
        comma::csv::options csv;
        bool output_fields_option;
        bool output_format_option;
        bool flush;
        bool write_header;
        bool discard;
        bool ignore_time_format;
        uint block_{0};
    };
    class to_point_cloud
    {
        struct field_desc
        {
            typename traits<Version>::PointField point_field; // includes name,offset,datatype,count for ROS data
            comma::csv::format::types_enum input_type;
            std::size_t input_offset;
            std::size_t size;               // target size = sizeof( output datatype ) * count

            field_desc( typename traits<Version>::PointField point_field
                    , comma::csv::format::types_enum input_type
                    , std::size_t input_offset
                    , std::size_t size )
                : point_field( point_field )
                , input_type( input_type )
                , input_offset( input_offset )
                , size( size )
            {}
        };

    public:
        to_point_cloud( const comma::command_line_options& options
                    , const comma::csv::options& csv
                    , const comma::csv::format& format )
        {
            std::string output_fields_str = options.value< std::string >( "--output-fields", csv.fields );
            comma::verbose << "outputting " << output_fields_str << std::endl;
            frame_id = options.value< std::string >( "--frame", "" );
            std::string field_name_mappings = options.value< std::string >( "--field-name-map", "" );
            time_format = snark::ros::pointcloud::string_to_time_format( options.value< std::string >( "--time-format", "none" ));

            comma::csv::format expanded_format( format.expanded_string() );
            std::vector< std::string > fields = comma::split( csv.fields, ',' );
            std::vector< std::string > output_fields = comma::split( output_fields_str, ',' );
            const auto& elements = expanded_format.elements();
            if( fields.size() != elements.size() ) { COMMA_THROW( comma::exception, "size of fields and binary mismatch: " << fields.size() << " vs " << elements.size() ); }
            field_descs.reserve( fields.size() );

            if( !field_name_mappings.empty() )
            {
                std::vector< std::string > field_name_mapping_pairs = comma::split( field_name_mappings, ',' );
                for( const auto& field_name_mapping_pair : field_name_mapping_pairs )
                {
                    std::vector< std::string > field_name_pair = comma::split( field_name_mapping_pair, ':' );
                    comma::verbose << "mapping " << field_name_pair[0] << " to " << field_name_pair[1] << std::endl;
                    field_name_map[ field_name_pair[0] ] = field_name_pair[1];
                }
            }

            std::size_t output_offset = 0;
            for( auto& output_field : output_fields )
            {
                auto it = std::find( fields.begin(), fields.end(), output_field );
                if( it != fields.end() )
                {
                    auto i = std::distance( fields.begin(), it );
                    typename traits<Version>::PointField point_field;
                    point_field.name = map_field_name( fields[i] );
                    point_field.offset = output_offset;
                    point_field.datatype = map_data_type( elements[i].type );
                    point_field.count = elements[i].count;
                    std::size_t total_size = sizeof_datatype( point_field.datatype ) * point_field.count;
                    field_descs.push_back( field_desc( point_field, elements[i].type, elements[i].offset, total_size ));
                    output_offset += total_size;
                    comma::verbose << "added " << point_field.name
                                << "(" << comma::csv::format::to_format( elements[i].type )
                                << ") to point fields" << std::endl;
                }
            }
            output_data_size = output_offset;
            comma::verbose << "output_data_size: " << output_data_size << std::endl;
        }

        typename traits<Version>::PointCloud2 create_msg( const std::vector< snark::ros::pointcloud::record >& records )
        {
            unsigned int count = records.size();
            typename traits<Version>::PointCloud2 msg;

            std::vector< typename traits<Version>::PointField > point_fields;
            for( const auto& field_desc : field_descs ) { point_fields.push_back( field_desc.point_field ); }

            boost::posix_time::ptime msg_start_time = records[0].t;

            msg.header.stamp = traits<Version>::time_from_boost( msg_start_time ); //
            msg.header.frame_id = frame_id;
            msg.height = 1;
            msg.width = count;
            msg.point_step = output_data_size;
            msg.row_step = output_data_size * count;
            msg.fields = point_fields;
            msg.data.resize( output_data_size * count );
            msg.is_dense = true;

            std::size_t msg_data_offset = 0;
            for( const auto& record : records )
            {
                size_t field_offset = 0;
                for( const auto& field_desc : field_descs )
                {
                    const char* input_address = &record.data[0] + field_desc.input_offset;
                    if( field_desc.input_type == comma::csv::format::types_enum::time )
                    {
                        if( time_format == snark::ros::pointcloud::time_format_enum::offset_seconds ||
                            time_format == snark::ros::pointcloud::time_format_enum::offset_nanoseconds )
                        {
                            char* field_address = reinterpret_cast< char*>( &msg.data[msg_data_offset] + field_offset );
                            boost::posix_time::ptime point_time( comma::csv::format::traits< boost::posix_time::ptime, comma::csv::format::time >::from_bin( input_address ));
                            boost::posix_time::time_duration time_offset = point_time - msg_start_time;
                            if( time_format == snark::ros::pointcloud::time_format_enum::offset_seconds )
                            {
                                float time_offset_seconds = static_cast< float >( time_offset.total_microseconds() ) / 1000000.0;
                                comma::csv::format::traits< float >::to_bin( time_offset_seconds, field_address );
                            }
                            else // snark::ros::pointcloud::time_format_enum::offset_nanoseconds )
                            {
                                comma::uint32 time_offset_nanoseconds = time_offset.total_microseconds() * 1000.0;
                                comma::csv::format::traits< comma::uint32 >::to_bin( time_offset_nanoseconds, field_address );
                            }
                            field_offset += field_desc.size;
                            continue;
                        }
                    }
                    std::memcpy( &msg.data[msg_data_offset] + field_offset, input_address, field_desc.size );
                    field_offset += field_desc.size;
                }
                msg_data_offset += output_data_size;
            }
            return msg;
        }

    private:
        std::string map_field_name( const std::string& src_name )
        {
            try { return field_name_map.at( src_name ); }
            catch( std::out_of_range& ex ) { return src_name; }
        }

        unsigned int map_data_type( comma::csv::format::types_enum t )
        {
            switch(t)
            {
                case comma::csv::format::char_t:
                case comma::csv::format::int8:     return traits<Version>::PointField::INT8;
                case comma::csv::format::uint8:    return traits<Version>::PointField::UINT8;
                case comma::csv::format::int16:    return traits<Version>::PointField::INT16;
                case comma::csv::format::uint16:   return traits<Version>::PointField::UINT16;
                case comma::csv::format::int32:    return traits<Version>::PointField::INT32;
                case comma::csv::format::uint32:   return traits<Version>::PointField::UINT32;
                case comma::csv::format::float_t:  return traits<Version>::PointField::FLOAT32;
                case comma::csv::format::double_t: return traits<Version>::PointField::FLOAT64;
                case comma::csv::format::int64:
                case comma::csv::format::uint64:   comma::verbose << "warning: ROS PointCloud2 doesn't support data type '"
                                                                << comma::csv::format::to_format( t )
                                                                << "', using FLOAT64 instead" << std::endl;
                                                return traits<Version>::PointField::FLOAT64;
                case comma::csv::format::time:
                    {
                        switch( time_format )
                        {
                            case snark::ros::pointcloud::time_format_enum::none:               comma::verbose << "warning: ROS PointCloud2 doesn't support data type '"
                                                                                    << comma::csv::format::to_format( t )
                                                                                    << "', using FLOAT64 instead" << std::endl;
                                                                    return traits<Version>::PointField::FLOAT64;
                            case snark::ros::pointcloud::time_format_enum::offset_seconds:     return traits<Version>::PointField::FLOAT32;
                            case snark::ros::pointcloud::time_format_enum::offset_nanoseconds: return traits<Version>::PointField::UINT32;
                        }
                    }
                default:
                    { COMMA_THROW( comma::exception, "data type not supported: " << comma::csv::format::to_format(t) ); }
            }
        }

        static std::size_t sizeof_datatype( std::size_t datatype )
        {
            switch( datatype )
            {
                case traits<Version>::PointField::INT8:
                case traits<Version>::PointField::UINT8:   return 1;
                case traits<Version>::PointField::INT16:
                case traits<Version>::PointField::UINT16:  return 2;
                case traits<Version>::PointField::INT32:
                case traits<Version>::PointField::UINT32:
                case traits<Version>::PointField::FLOAT32: return 4;
                case traits<Version>::PointField::FLOAT64: return 8;
                default:
                    { COMMA_THROW( comma::exception, "unknown data type: " << datatype ); }
            }
        }

        std::vector< field_desc > field_descs;
        std::size_t output_data_size;
        std::string frame_id;
        std::unordered_map< std::string, std::string > field_name_map;
        snark::ros::pointcloud::time_format_enum time_format;

    };

    class to_points
    {
    public:
        to_points( const comma::command_line_options& options
                , const comma::csv::options& csv )
            : format( csv.binary() ? csv.format() : comma::csv::format( options.value< std::string >( "--format" )))
            , point_cloud( options, csv, format )
            , data_size( format.size() )
            , ascii( !csv.binary() )
        {}

        void add_record( const snark::ros::pointcloud::record& r, const comma::csv::input_stream<snark::ros::pointcloud::record>& is )
        {
            records.push_back( r );
            records.back().data.resize( data_size );
            if( ascii )
            {
                std::string buf = format.csv_to_bin( is.ascii().last() );
                if( buf.size() != data_size ) { COMMA_THROW( comma::exception, "csv_to_bin size mismatch " << buf.size() << "; " << data_size ); }
                std::memcpy( &records.back().data[0], buf.data(), data_size );
            }
            else
            {
                std::memcpy( &records.back().data[0], is.binary().last(), data_size );
            }
        }

        void send( const std::function< void( typename traits<Version>::PointCloud2 ) >& publisher_fn )
        {
            //create msg
            typename traits<Version>::PointCloud2 msg = point_cloud.create_msg( records );
            publisher_fn( msg );
            records.clear();
        }

        bool empty() { return records.empty(); }

    private:
        std::vector< snark::ros::pointcloud::record > records;
        comma::csv::format format;
        to_point_cloud point_cloud;
        std::size_t data_size;
        bool ascii;
    };

}; // struct pointcloud<Version>

} } } // namespace snark { namespace ros { namespace detail {


namespace comma { namespace visiting {
#ifdef SNARK_BUILD_ROS_1
    template <> struct traits< typename snark::ros::detail::pointcloud< 1 >::header >
    {
        template< typename K, typename V > static void visit( const K& k, const typename snark::ros::detail::pointcloud< 1 >::header& p, V& v )
        {
            v.apply( "t", p.t );
            v.apply( "block", p.block );
        }
    };
#endif
#ifdef SNARK_BUILD_ROS_2
    template <> struct traits< typename snark::ros::detail::pointcloud< 2 >::header >
    {
        template< typename K, typename V > static void visit( const K& k, const typename snark::ros::detail::pointcloud< 2 >::header& p, V& v )
        {
            v.apply( "t", p.t );
            v.apply( "block", p.block );
        }
    };
#endif

} } // namespace comma { namespace visiting {


// =========================
// --to topic


namespace comma { namespace visiting {

template <> struct traits< snark::ros::pointcloud::record >
{
    template< typename K, typename V > static void visit( const K&, const snark::ros::pointcloud::record& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
    }
    template< typename K, typename V > static void visit( const K&, snark::ros::pointcloud::record& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
    }
};

} } // namespace comma { namespace visiting {
