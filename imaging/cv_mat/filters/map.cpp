// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <fstream>
#include <boost/date_time/posix_time/ptime.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>
#include "map.h"

namespace snark { namespace cv_mat { namespace filters {

struct map_input_t
{
    typedef double value_type;
    typedef comma::int32 key_type;
    key_type key;
    value_type value;
};

typedef map_input_t::key_type key_type;
typedef map_input_t::value_type output_value_type;

} } } // namespace snark { namespace cv_mat { namespace filters {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_mat::filters::map_input_t >
{
    template< typename K, typename V > static void visit( const K&, snark::cv_mat::filters::map_input_t& t, V& v )
    {
        v.apply( "key", t.key );
        v.apply( "value", t.value );
    }
    template< typename K, typename V > static void visit( const K&, const snark::cv_mat::filters::map_input_t& t, V& v )
    {
        v.apply( "key", t.key );
        v.apply( "value", t.value );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
map< H >::map( const std::string& map_filter_options, bool permissive ) : permissive_ ( permissive )
{
    comma::csv::options csv_options = comma::name_value::parser( "filename", '&' , '=' ).get< comma::csv::options >( map_filter_options );
    if( csv_options.fields.empty() ) { csv_options.fields = "value"; }
    if( !csv_options.has_field( "value" ) ) { COMMA_THROW( comma::exception, "map filter: fields option is given but \"value\" field is not found" ); }
    bool no_key_field = !csv_options.has_field( "key" );
    std::ifstream ifs( &csv_options.filename[0] );
    if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "map filter: failed to open \"" << csv_options.filename << "\"" ); }
    comma::csv::input_stream< map_input_t > map_stream( ifs , csv_options );
    for( key_type counter = 0; map_stream.ready() || ( ifs.good() && !ifs.eof() ) ; ++counter )
    {
        const map_input_t* map_input = map_stream.read();
        if( !map_input ) { break; }
        map_[ no_key_field ? counter : map_input->key ] = map_input->value;
    }
}

#if ( defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2 ) || ( !defined( CV_VERSION_EPOCH ) && ( ( CV_VERSION_MAJOR == 3 && ( CV_VERSION_MINOR < 3 || ( CV_VERSION_MINOR == 3 && CV_VERSION_REVISION < 1 ) ) ) ) )
template < typename H >
typename map< H >::value_type map< H >::operator()( typename map< H >::value_type m ) // todo: support multiple channels
{
    value_type n( m.first, cv::Mat( m.second.size(), cv::DataType< output_value_type >::type ) );
    try
    {
        switch( m.second.type() ) // quick and dirty; opencv really got their design wrong: type is known in runtime whereas handling types is a compile-time thing
        {
            case cv::DataType< unsigned char >::type : apply_map_< unsigned char >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< unsigned char, 2 > >::type : apply_map_< cv::Vec< unsigned char, 2 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< unsigned char, 3 > >::type : apply_map_< cv::Vec< unsigned char, 3 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< unsigned char, 4 > >::type : apply_map_< cv::Vec< unsigned char, 4 > >( m.second, n.second ); break;
            case cv::DataType< comma::uint16 >::type : apply_map_< comma::uint16 >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::uint16, 2 > >::type : apply_map_< cv::Vec< comma::uint16, 2 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::uint16, 3 > >::type : apply_map_< cv::Vec< comma::uint16, 3 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::uint16, 4 > >::type : apply_map_< cv::Vec< comma::uint16, 4 > >( m.second, n.second ); break;
            case cv::DataType< char >::type : apply_map_< char >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< char, 2 > >::type : apply_map_< cv::Vec< char, 2 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< char, 3 > >::type : apply_map_< cv::Vec< char, 3 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< char, 4 > >::type : apply_map_< cv::Vec< char, 4 > >( m.second, n.second ); break;
            case cv::DataType< comma::int16 >::type : apply_map_< comma::int16 >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::int16, 2 > >::type : apply_map_< cv::Vec< comma::int16, 2 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::int16, 3 > >::type : apply_map_< cv::Vec< comma::int16, 3 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::int16, 4 > >::type : apply_map_< cv::Vec< comma::int16, 4 > >( m.second, n.second ); break;
            case cv::DataType< comma::int32 >::type : apply_map_< comma::int32 >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::int32, 2 > >::type : apply_map_< cv::Vec< comma::int32, 2 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::int32, 3 > >::type : apply_map_< cv::Vec< comma::int32, 3 > >( m.second, n.second ); break;
            case cv::DataType< cv::Vec< comma::int32, 4 > >::type : apply_map_< cv::Vec< comma::int32, 4 > >( m.second, n.second ); break;
            default: std::cerr << "map filter: expected integer cv type, got " << m.second.type() << std::endl; return value_type();
        }
    }
    catch ( std::out_of_range ) { return value_type(); }
    return n;
}
#else
template < typename H >
typename map< H >::value_type map< H >::operator()( tyename map< H >::value_type m ) // todo: support multiple channels
{
    value_type n( m.first, cv::Mat( m.second.size(), cv::traits::Type< output_value_type >::value ) );
    try
    {
        switch( m.second.type() ) // quick and dirty; opencv really got their design wrong: type is known in runtime whereas handling types is a compile-time thing
        {
            case cv::traits::Type< unsigned char >::value: apply_map_< unsigned char >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< unsigned char, 2 > >::value : apply_map_< cv::Vec< unsigned char, 2 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< unsigned char, 3 > >::value : apply_map_< cv::Vec< unsigned char, 3 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< unsigned char, 4 > >::value : apply_map_< cv::Vec< unsigned char, 4 > >( m.second, n.second ); break;
            case cv::traits::Type< comma::uint16 >::value : apply_map_< comma::uint16 >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::uint16, 2 > >::value : apply_map_< cv::Vec< comma::uint16, 2 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::uint16, 3 > >::value : apply_map_< cv::Vec< comma::uint16, 3 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::uint16, 4 > >::value : apply_map_< cv::Vec< comma::uint16, 4 > >( m.second, n.second ); break;
            case cv::traits::Type< char >::value : apply_map_< char >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< char, 2 > >::value : apply_map_< cv::Vec< char, 2 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< char, 3 > >::value : apply_map_< cv::Vec< char, 3 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< char, 4 > >::value : apply_map_< cv::Vec< char, 4 > >( m.second, n.second ); break;
            case cv::traits::Type< comma::int16 >::value : apply_map_< comma::int16 >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::int16, 2 > >::value : apply_map_< cv::Vec< comma::int16, 2 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::int16, 3 > >::value : apply_map_< cv::Vec< comma::int16, 3 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::int16, 4 > >::value : apply_map_< cv::Vec< comma::int16, 4 > >( m.second, n.second ); break;
            case cv::traits::Type< comma::int32 >::value : apply_map_< comma::int32 >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::int32, 2 > >::value : apply_map_< cv::Vec< comma::int32, 2 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::int32, 3 > >::value : apply_map_< cv::Vec< comma::int32, 3 > >( m.second, n.second ); break;
            case cv::traits::Type< cv::Vec< comma::int32, 4 > >::value : apply_map_< cv::Vec< comma::int32, 4 > >( m.second, n.second ); break;
            default: std::cerr << "map filter: expected integer cv type, got " << m.second.type() << std::endl; return value_type();
        }
    }
    catch ( std::out_of_range& ) { return value_type(); }
    return n;
}
#endif

template < typename H >
template < typename input_value_type >
void map< H >::apply_map_( const cv::Mat& input, cv::Mat& output ) // todo: certainly reimplement with tbb::parallel_for
{
    for( int i = 0; i < input.rows; ++i )
    {
        for( int j = 0; j < input.cols; ++j )
        {
            const auto& keys = input.at< input_value_type >(i,j);
            for( int channel = 0; channel < input.channels(); ++channel )
            {
                auto key = get_channel_( keys, channel );
                map_t_::const_iterator it = map_.find( key );
                if( it == map_.end() )
                {
                    if( permissive_ ) { std::cerr << "map filter: expected a pixel value from the map, got: pixel at " << i << "," << j << " with value " << key << std::endl; throw std::out_of_range(""); }
                    set_channel_( output.at< output_value_type >(i,j), channel, output_value_type( key ) ); // todo? implement value clipping to 0 or 1? refactor not-found behaviour!
                }
                else
                {
                    set_channel_( output.at< output_value_type >(i,j), channel, it->second );
                }
            }
        }
    }
}

template class map< boost::posix_time::ptime >;
template class map< std::vector< char > >;

} } } // namespace snark { namespace cv_mat { namespace filters {
