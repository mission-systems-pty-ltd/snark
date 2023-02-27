// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include "detail/file-util.h"
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <thread>
#include <unordered_map>

void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --header-fields --header-format --node-name --output-fields --output-format"
        " --from --bags --fields --flush --header --output-header --max-datagram-size --no-discard --queue-size"
        " --to --all --field-name-map --hang-on --stay --frame --latch --node-name --pass-through --pass --queue-size"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

void usage( bool detail )
{
    std::cerr << "\nconvert ROS PointCloud2 to csv and vice-versa";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " --from=<topic> [<options>]";
    std::cerr << "\n       " << comma::verbose.app_name() << " --to=<topic> [<options>]";
    std::cerr << "\n";
    std::cerr << "\ngeneral options";
    std::cerr << "\n    --help,-h:       show help; --help --verbose: show more help";
    std::cerr << "\n    --verbose,-v:    show detailed messages";
    std::cerr << "\n    --header-fields: write csv field names of header to stdout and exit";
    std::cerr << "\n    --header-format: write csv format of header to stdout and exit";
    std::cerr << "\n    --node-name:     node name for this process, when not specified uses";
    std::cerr << "\n                     ros::init_options::AnonymousName flag";
    std::cerr << "\n    --output-fields: print field names and exit";
    std::cerr << "\n    --output-format: print format and exit";
    std::cerr << "\n";
    std::cerr << "\n    field names and format are extracted from the first message of the";
    std::cerr << "\n    subscribed topic";
    std::cerr << "\n";
    std::cerr << "\nfrom options";
    std::cerr << "\n    --from=<topic>:      topic to read";
    std::cerr << "\n    --bags=[<bags>]:     load from rosbags rather than subscribe";
    std::cerr << "\n    --fields=[<names>]:  only output listed fields";
    std::cerr << "\n    --flush:             call flush on stdout after each write";
    std::cerr << "\n    --header,--output-header: prepend t,block header to output with t,ui format";
    std::cerr << "\n    --max-datagram-size: for UDP transport. See ros::TransportHints";
    std::cerr << "\n    --no-discard:        don't discard points with nan or inf in their values";
    std::cerr << "\n    --queue-size=[<n>]:  ROS Subscriber queue size, default 1";
    std::cerr << "\n    --topic=<topic>:     name of the topic to subscribe to";
    std::cerr << "\n";
    std::cerr << "\nto options";
    std::cerr << "\n    --to=<topic>:             topic to publish to";
    std::cerr << "\n    --all:                    send all records as one ros message";
    std::cerr << "\n    --fields,-f=<fields>:     fields names; default=x,y,z";
    std::cerr << "\n    --field-name-map=[<map>]: rename fields; format: old:new,...";
    std::cerr << "\n    --frame=[<frame>]:        ros message frame as string";
    std::cerr << "\n    --hang-on,--stay:         wait before exiting so that subscribers can";
    std::cerr << "\n                              receive the last message";
    std::cerr << "\n    --latch:                  last message will be saved for future subscribers";
    std::cerr << "\n    --output,-o=[<bag>]:      write to bag rather than publish";
    std::cerr << "\n    --output-fields=[<fields>]: fields to output; default: all input fields";
    std::cerr << "\n    --pass-through,--pass:    pass input data to stdout";
    std::cerr << "\n    --queue-size=[<n>]:       ROS publisher queue size, default=1";
    std::cerr << "\n";
    std::cerr << "\nexamples:";
    std::cerr << "\n    view points from a published topic:";
    std::cerr << "\n    " << comma::verbose.app_name() << " --from some_topic --fields x,y,z --binary 3f --header \\";
    std::cerr << "\n        | view-points --fields t,block,x,y,z --binary t,ui,3f";
    std::cerr << "\n";
    std::cerr << "\n    view points from a set of bags:";
    std::cerr << "\n    " << comma::verbose.app_name() << " --from some_topic --bags \"*.bag\" --fields x,y,z --binary 3f --header \\";
    std::cerr << "\n        | view-points --fields t,block,x,y,z --binary t,ui,3f";
    std::cerr << "\n";
    std::cerr << "\n    csv-random make --type 3d | csv-paste line-number - \\";
    std::cerr << "\n        | csv-blocks group --fields scalar --span 1000 | csv-time-stamp \\";
    std::cerr << "\n        | " << comma::verbose.app_name() << " --to /points -f t,id,x,y,z,block --format t,ui,3d,ui";
    std::cerr << "\n";
    std::cerr << "\n    cat data.bin | " << comma::verbose.app_name() << " --to /points -f t,block,x,y,z -b t,ui,3f";
    std::cerr << "\n" << std::endl;
}

// =========================
// --from topic

static bool status = 0; // quick and dirty

namespace snark { namespace ros {

/// utility functions for ros sensor_msgs::PointCloud2
struct point_cloud
{
private:
    static std::vector< comma::csv::format::types_enum > rmap_data_type;
public:
    static const std::vector< comma::csv::format::types_enum >& get_rmap_data_type()
    {
        if( rmap_data_type.size() == 0 )
        {
            rmap_data_type.resize( 9 );
            rmap_data_type.at( sensor_msgs::PointField::INT8 ) = comma::csv::format::char_t;
            rmap_data_type.at( sensor_msgs::PointField::UINT8 ) = comma::csv::format::uint8;
            rmap_data_type.at( sensor_msgs::PointField::INT16 ) = comma::csv::format::int16;
            rmap_data_type.at( sensor_msgs::PointField::UINT16 ) = comma::csv::format::uint16;
            rmap_data_type.at( sensor_msgs::PointField::INT32 ) = comma::csv::format::int32;
            rmap_data_type.at( sensor_msgs::PointField::UINT32 ) = comma::csv::format::uint32;
            rmap_data_type.at( sensor_msgs::PointField::FLOAT32 ) = comma::csv::format::float_t;
            rmap_data_type.at( sensor_msgs::PointField::FLOAT64 ) = comma::csv::format::double_t;
        }
        return rmap_data_type;
    }
    static std::size_t size_of_type( comma::csv::format::types_enum t )
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
    static std::string msg_fields_names( const sensor_msgs::PointCloud2::_fields_type& msg_fields
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
    static std::string msg_fields_format( const sensor_msgs::PointCloud2::_fields_type& msg_fields
                                        , const std::vector< std::string >& field_filter = std::vector< std::string >() )
    {
        std::string s;
        std::string delimiter;
        std::size_t expected_offset = 0;
        bool add_field;
        const auto& rmap = get_rmap_data_type();
        for( const auto& f : msg_fields )
        {
            comma::csv::format::types_enum type = rmap.at( f.datatype );
            if( field_filter.empty() )
            {
                if( f.offset > expected_offset )
                {
                    for( auto t: padding_types( f.offset - expected_offset ))
                    {
                        s += delimiter + comma::csv::format::to_format( t );
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
                s += delimiter + ( f.count > 1 ? boost::lexical_cast< std::string >( f.count ) : "" )
                    + comma::csv::format::to_format( type );
                if( delimiter.empty() ) { delimiter = ","; }
            }
        }
        return s;
    }

    struct bin_base
    {
        virtual ~bin_base() {}
        virtual const char* get( const char* data ) = 0;
        virtual std::size_t size() const = 0;
    };

    struct bin_cat : public bin_base
    {
        uint32_t size_;
        bin_cat( uint32_t s = 0 ) : size_( s ) { }
        const char* get( const char* data ){ return data; }
        std::size_t size() const { return size_; }
    };

    /// copy specified fields from a point record, given msg point field info and a list of field names
    struct bin_shuffle : public bin_base
    {
        /// prepare
        bin_shuffle( const std::string& field_names, const sensor_msgs::PointCloud2::_fields_type& msg_fields )
        {
            std::vector< std::string > fields = comma::split( field_names, "," );
            std::unordered_map< std::string, unsigned int > msg_field_name_map;
            std::vector< range_t > elements;
            const auto& rmap = get_rmap_data_type();
            std::size_t size = 0;
            for( std::size_t i = 0; i < msg_fields.size(); i++ )
            {
                msg_field_name_map[ msg_fields[i].name ] = i;
                if( msg_fields[i].datatype < 1 || msg_fields[i].datatype >= rmap.size() ) { COMMA_THROW( comma::exception, "datatype out of range (1 to 8) " << unsigned( msg_fields[i].datatype )); }
                comma::csv::format::types_enum type = rmap.at( msg_fields[i].datatype );
                //using comma::csv::format::size_of(type) corrupts stack
//                 elements.push_back(range_t(msg_fields[i].offset, msg_fields[i].count * comma::csv::format::size_of(type)));
                elements.push_back( range_t( msg_fields[i].offset, msg_fields[i].count * point_cloud::size_of_type( type )));
            }
            for( const auto& f : fields )
            {
                unsigned int index;
                try { index = msg_field_name_map.at( f ); }
                catch( std::out_of_range& ex ) { COMMA_THROW( comma::exception, "couldn't find " << f << " in msg_field_name_map" ); }
                ranges.push_back( elements[index] );
                size += elements[index].second;
            }
            buf.resize( size );
        }
        /// shuffle
        const char* get( const char* data )
        {
            std::size_t offset = 0;
            for( const auto& i : ranges )
            {
                std::memcpy( &buf[offset], data+i.first, i.second );
                offset += i.second;
            }
            return buf.data();
        }
        bool empty() const { return ranges.empty(); }
        std::size_t size() const { return buf.size(); }

    private:
        std::vector< char > buf;
        //first: offset, second: size
        typedef typename std::pair< std::size_t, std::size_t > range_t;
        std::vector< range_t > ranges;
    };
};

std::vector< comma::csv::format::types_enum > point_cloud::rmap_data_type;

} } // namespace snark { namespace ros {

struct header
{
    boost::posix_time::ptime t;
    uint32_t block;
    header() : block( 0 ) { }
    header( const ros::Time& time, uint32_t seq ) : t( time.toBoost() ), block( seq ) {}
};

namespace comma { namespace visiting {

template <> struct traits< header >
{
    template< typename K, typename V > static void visit( const K& k, const header& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

/// process ros sensor_msgs::PointCloud2 message
struct points
{
    std::vector< std::string > fields;
    comma::csv::format format;
    comma::csv::options csv;
    bool output_fields;
    bool output_format;
    bool flush;
    bool write_header;
    bool discard;

    points( const comma::command_line_options& options )
        : csv( options )
        , output_fields( options.exists( "--output-fields" ))
        , output_format( options.exists( "--output-format" ))
        , flush( options.exists( "--flush" ))
        , write_header( options.exists( "--header,--output-header" ))
        , discard( !options.exists( "--no-discard" ))
    {
        fields = comma::split( options.value< std::string >( "--fields", "" ), ',' );
        if( fields.size() == 1 && fields[0].empty() ) { fields.clear(); } // comma::split quirk
    }

    void process( const sensor_msgs::PointCloud2ConstPtr input )
    {
        try
        {
            if( output_fields )
            {
                if( write_header ) { std::cout << comma::join( comma::csv::names< header >(), ',' ) << ","; }
                std::cout << snark::ros::point_cloud::msg_fields_names( input->fields, fields ) << std::endl;
                ros::shutdown();
                return;
            }
            if( output_format )
            {
                if( write_header ) { std::cout << comma::csv::format::value< header >() << ","; }
                std::cout << snark::ros::point_cloud::msg_fields_format( input->fields, fields ) << std::endl;
                ros::shutdown();
                return;
            }
            if( format.count() == 0 )
            {
                format = comma::csv::format( snark::ros::point_cloud::msg_fields_format( input->fields, fields ));
                comma::verbose << "setting format to " << format.string() << std::endl;
            }
            unsigned int count = input->width * input->height;
            unsigned int record_size = input->point_step;
            ::header header( input->header.stamp, input->header.seq );

            std::unique_ptr< snark::ros::point_cloud::bin_base > bin;
            if( csv.fields.empty() ) { bin.reset( new snark::ros::point_cloud::bin_cat( record_size )); }
            else { bin.reset( new snark::ros::point_cloud::bin_shuffle( csv.fields, input->fields )); }

            bin_writer writer;

            std::unique_ptr< filter_base > filter;
            if( discard ) { filter.reset( new float_filter( format )); }

            for( unsigned int i = 0; i < count; i++ )
            {
                const char* buf = bin->get( reinterpret_cast< const char* >( &input->data[i * record_size] ));
                if( !filter || filter->valid( buf, bin->size() ))
                {
                    if( write_header ) { writer.write_header( header ); }
                    writer.write( buf, bin->size() );
                    if( flush ) { std::cout.flush(); }
                }
                if( !std::cout.good() ) { ros::shutdown(); break; }
            }
        }
        catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; status = 1; ros::shutdown(); }
        catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; status = 1; ros::shutdown(); }
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
};

// =========================
// --to topic

struct record
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    std::vector< char > data;
    record() : block( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< record >
{
    template< typename K, typename V > static void visit( const K&, const record& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
    }

    template< typename K, typename V > static void visit( const K&, record& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace ros {

class to_point_cloud
{
    struct field_desc
    {
        sensor_msgs::PointField point_field;
        std::size_t input_offset;
        std::size_t size;
        field_desc( sensor_msgs::PointField point_field, std::size_t input_offset, std::size_t size )
            : point_field( point_field )
            , input_offset( input_offset )
            , size( size )
        {}
    };

public:
    to_point_cloud( const std::string& fields_str
                  , const std::string& format_str
                  , const std::string& output_fields_str
                  , const std::string& frame_id
                  , const std::string& field_name_mappings )
        : frame_id( frame_id )
    {
        comma::csv::format format( format_str );
        std::vector< std::string > fields = comma::split( fields_str, ',' );
        std::vector< std::string > output_fields = comma::split( output_fields_str, ',' );
        const auto& elements = format.elements();
        if( fields.size() != elements.size() ) { COMMA_THROW( comma::exception, "size of fields and binary mismatch: " << fields.size() << " vs " << elements.size() ); }
        field_descs.reserve( fields.size() );

        if( !field_name_mappings.empty() )
        {
            std::vector< std::string > field_name_mapping_pairs = comma::split( field_name_mappings, '.' );
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
                sensor_msgs::PointField point_field;
                point_field.name = map_field_name( fields[i] );
                point_field.offset = output_offset;
                point_field.datatype = map_data_type( elements[i].type );
                point_field.count = elements[i].count;
                std::size_t total_size = sizeof_datatype( point_field.datatype ) * point_field.count;
                field_descs.push_back( field_desc( point_field, elements[i].offset, total_size ));
                output_offset += total_size;
                comma::verbose << "added " << point_field.name
                               << "(" << comma::csv::format::to_format( elements[i].type )
                               << ") to point fields" << std::endl;
            }
        }
        output_data_size = output_offset;
        comma::verbose << "output_data_size: " << output_data_size << std::endl;
    }

    sensor_msgs::PointCloud2 create_msg( const std::vector< record >& records )
    {
        unsigned int count = records.size();
        sensor_msgs::PointCloud2 msg;

        std::vector< sensor_msgs::PointField > point_fields;
        for( const auto& field_desc : field_descs ) { point_fields.push_back( field_desc.point_field ); }

        msg.header.stamp = ::ros::Time::fromBoost( records[0].t );
        msg.header.seq = records[0].block;
        msg.header.frame_id = frame_id;
        msg.height = 1;
        msg.width = count;
        msg.point_step = output_data_size;
        msg.row_step = output_data_size * count;
        msg.fields = point_fields;
        msg.data.resize( output_data_size * count );

        std::size_t msg_data_offset = 0;
        for( const auto& record : records )
        {
            size_t field_offset = 0;
            for( const auto& field_desc : field_descs )
            {
                std::memcpy( &msg.data[msg_data_offset] + field_offset
                           , &record.data[0] + field_desc.input_offset
                           , field_desc.size );
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

    static unsigned int map_data_type( comma::csv::format::types_enum t )
    {
        switch(t)
        {
            case comma::csv::format::char_t:
            case comma::csv::format::int8:
                return sensor_msgs::PointField::INT8;
            case comma::csv::format::uint8:
                return sensor_msgs::PointField::UINT8;
            case comma::csv::format::int16:
                return sensor_msgs::PointField::INT16;
            case comma::csv::format::uint16:
                return sensor_msgs::PointField::UINT16;
            case comma::csv::format::int32:
                return sensor_msgs::PointField::INT32;
            case comma::csv::format::uint32:
                return sensor_msgs::PointField::UINT32;
            case comma::csv::format::float_t:
                return sensor_msgs::PointField::FLOAT32;
            case comma::csv::format::double_t:
                return sensor_msgs::PointField::FLOAT64;
            case comma::csv::format::int64:
            case comma::csv::format::uint64:
            case comma::csv::format::time:
                comma::verbose << "warning: ROS PointCloud2 doesn't support data type '" << comma::csv::format::to_format(t) << "', using FLOAT64 instead" << std::endl;
                return sensor_msgs::PointField::FLOAT64;
            default:
                { COMMA_THROW( comma::exception, "data type not supported: " << comma::csv::format::to_format(t) ); }
        }
    }

    static std::size_t sizeof_datatype( std::size_t datatype )
    {
        switch( datatype )
        {
            case sensor_msgs::PointField::INT8:
            case sensor_msgs::PointField::UINT8:
                return 1;
            case sensor_msgs::PointField::INT16:
            case sensor_msgs::PointField::UINT16:
                return 2;
            case sensor_msgs::PointField::INT32:
            case sensor_msgs::PointField::UINT32:
            case sensor_msgs::PointField::FLOAT32:
                return 4;
            case sensor_msgs::PointField::FLOAT64:
                return 8;
            default:
                { COMMA_THROW( comma::exception, "unknown data type: " << datatype ); }
        }
    }

    std::vector< field_desc > field_descs;
    std::size_t output_data_size;
    std::string frame_id;
    std::unordered_map< std::string, std::string > field_name_map;
};

} } // namespace snark { namespace ros {

class to_points
{
public:
    to_points( const comma::csv::options& csv
             , const comma::csv::format& format
             , const std::string& output_fields
             , const std::string& frame_id
             , const std::string& field_name_mapping )
        : format( format )
        , point_cloud( csv.fields, format.expanded_string(), output_fields, frame_id, field_name_mapping )
        , data_size( format.size() )
        , ascii( !csv.binary() )
    {}

    void add_record( const record& r, const comma::csv::input_stream<record>& is )
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

    void send( const std::function< void( sensor_msgs::PointCloud2 ) >& publisher_fn )
    {
        //create msg
        sensor_msgs::PointCloud2 msg = point_cloud.create_msg( records );
        publisher_fn( msg );
        records.clear();
    }

    bool empty() { return records.empty(); }

private:
    std::vector< record > records;
    comma::csv::format format;
    snark::ros::to_point_cloud point_cloud;
    std::size_t data_size;
    bool ascii;
};

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );
        if( options.exists( "--from" ))
        {
            if( options.exists( "--header-fields" )) { std::cout << comma::join( comma::csv::names< header >(), ',' ) << std::endl; return 0; }
            if( options.exists( "--header-format" )) { std::cout << comma::csv::format::value< header >() << std::endl; return 0; }
            std::string bags_option = options.value< std::string >( "--bags", "" );
            std::string topic = options.value< std::string >( "--from" );
            unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
            boost::optional< int > max_datagram_size = options.optional< int >( "--max-datagram-size" );
            boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );
            uint32_t node_options = 0;
            if( !node_name )
            {
                node_name = "ros_points";
                node_options = ros::init_options::AnonymousName;
            }
            int arrrgc = 1;
            ros::init( arrrgc, argv, *node_name, node_options );
            points points( options );
            if( !bags_option.empty() )
            {
                rosbag::Bag bag;
                std::vector< std::string > bag_names;
                for( auto name: comma::split( bags_option, ',' ))
                {
                    std::vector< std::string > expansion = snark::ros::glob( name );
                    bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
                }
                for( auto bag_name: bag_names )
                {
                    comma::verbose << "opening " << bag_name << std::endl;
                    bag.open( bag_name );
                    for( rosbag::MessageInstance const m: rosbag::View( bag, rosbag::TopicQuery( topic )))
                    {
                        sensor_msgs::PointCloud2ConstPtr msg = m.instantiate< sensor_msgs::PointCloud2 >();
                        points.process( msg );
                        if( ros::isShuttingDown() ) { break; }
                    }
                    bag.close();
                    if( ros::isShuttingDown() ) { break; }
                }
            }
            else
            {
                if( !ros::master::check() ) { std::cerr << comma::verbose.app_name() << ": roscore appears not to be running, node will wait for it to start" << std::endl; }
                ros::NodeHandle ros_node;
                ros::TransportHints transport_hints;
                if( max_datagram_size ) { transport_hints = ros::TransportHints().maxDatagramSize( *max_datagram_size ); }
                ros::Subscriber subscriber = ros_node.subscribe( topic, queue_size, &points::process, &points, transport_hints );
                ros::spin();
            }
            return status;
        }
        else if( options.exists( "--to" ))
        {
            comma::csv::options csv(options);
            if( !csv.binary() && !options.exists( "--format" )) { COMMA_THROW( comma::exception, "please specify --binary=<format>, or --format=<format> for ascii"); }
            csv.full_xpath = true;
            std::string topic = options.value< std::string >( "--to" );
            unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
            if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
            comma::csv::format format = ( csv.binary()
                                        ? csv.format()
                                        : comma::csv::format( options.value< std::string >( "--format" )));
            std::string output_fields = options.value< std::string >( "--output-fields", csv.fields );
            comma::verbose << "outputting " << output_fields << std::endl;
            std::string field_name_mapping = options.value< std::string >( "--field-name-map", "" );
            bool has_block = csv.has_field( "block" );
            bool all = options.exists( "--all" );
            std::string frame_id = options.value< std::string >( "--frame", "" );
            boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );
            bool pass_through = options.exists( "--pass-through,--pass" );
            std::string output_option = options.value< std::string >( "--output,-o", "" );
            bool publishing = output_option.empty();

            std::unique_ptr< ros::NodeHandle > ros_node;
            std::unique_ptr< ros::Publisher > publisher;
            std::unique_ptr< rosbag::Bag > bag;
            std::unique_ptr< std::function< void( sensor_msgs::PointCloud2 ) > > publish_fn;

            if( publishing )
            {
                int arrrgc = 1;
                uint32_t node_options = 0;
                if( !node_name )
                {
                    node_name = "ros_points";
                    node_options = ros::init_options::AnonymousName;
                }
                ros::init( arrrgc, argv, *node_name, node_options );
                ros_node.reset( new ros::NodeHandle );
                publisher.reset( new ros::Publisher( ros_node->advertise< sensor_msgs::PointCloud2 >( topic, queue_size, options.exists( "--latch" ))));
                ros::spinOnce();
                publish_fn.reset( new std::function< void( sensor_msgs::PointCloud2 ) >(
                                                                                        [&]( sensor_msgs::PointCloud2 msg )
                                                                                        {
                                                                                            publisher->publish( msg );
                                                                                            ros::spinOnce();
                                                                                        } ));
            }
            else
            {
                bag.reset( new rosbag::Bag );
                bag->open( output_option, rosbag::bagmode::Write );
                publish_fn.reset( new std::function< void( sensor_msgs::PointCloud2 ) >(
                                                                                        [&]( sensor_msgs::PointCloud2 msg )
                                                                                        {
                                                                                            bag->write( topic, msg.header.stamp, msg );
                                                                                        } ));
            }

            comma::csv::input_stream< record > is( std::cin, csv );
            comma::csv::passed< record > passed( is, std::cout, csv.flush );
            unsigned int block = 0;
            to_points points( csv, format, output_fields, frame_id, field_name_mapping );

            while( std::cin.good() )
            {
                //read binary from input
                const record* p = is.read();
                if (( !p || block != p->block ) && !points.empty() )
                {
                    points.send( *publish_fn.get() );
                }
                if( !p ) { break; }
                if( pass_through ) { passed.write(); }
                block = p->block;
                points.add_record( *p, is );
                if( !has_block && !all ) { points.send( *publish_fn.get() ); }
            }

            if( publishing && options.exists( "--hang-on,--stay" ))
            {
                for( int i = 0; i < 3; i++ )
                {
                    ros::spinOnce();
                    std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
                }
            }
            return 0;
        }
        else
        {
            std::cerr << comma::verbose.app_name() << ": requires --from or --to option" << std::endl;
            return 1;
        }
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
