// Copyright (c) 2019 Vsevolod Vlaskine

#include <fstream>
#include <unordered_map>
#include <boost/date_time/posix_time/ptime.hpp>
#include <opencv2/core/version.hpp>
#include <comma/base/exception.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>
#include "text.h"
#include "../utils.h"

namespace comma { namespace visiting {

template <>
struct traits< snark::cv_mat::filters::text_input >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::cv_mat::filters::text_input& p, Visitor& v ) // hyper-quick and dirty
    {
        v.apply( "x", p.origin.x );
        v.apply( "y", p.origin.y );
        v.apply( "r", p.colour[2] / 256 );
        v.apply( "g", p.colour[1] / 256 );
        v.apply( "b", p.colour[0] / 256 );
        v.apply( "text", p.text );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::cv_mat::filters::text_input& p, Visitor& v ) // hyper-quick and dirty
    {
        v.apply( "x", p.origin.x );
        v.apply( "y", p.origin.y );
        int i = p.colour[2];
        v.apply( "r", i );
        p.colour[2] = i * 256;
        i = p.colour[1];
        v.apply( "g", i );
        p.colour[1] = i * 256;
        i = p.colour[0];
        v.apply( "b", i );
        p.colour[0] = i * 256;
        v.apply( "text", p.text );
    }
};
    
} } // namespace comma { namespace visiting {

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
text< H >::text( const text_input& caption
               , const comma::csv::options& csv
               , const std::vector< std::pair< unsigned int, unsigned int > >& ranges
               , float font_scale )
    : caption_( caption )
    , is_( nullptr )
    , istream_( nullptr )
    , ranges_( ranges )
    , font_scale_( font_scale )
    , range_index_( 0 )
    , count_( 0 )
{
    if( csv.filename.empty() ) { return; }
    is_.reset( new comma::io::istream( csv.filename ) );
    istream_.reset( new comma::csv::input_stream< text_input >( **is_, csv, text_input( caption_.origin, cv::Scalar( caption_.colour / 256 ), caption_.text ) ) ); // quick and dirty
}

static std::unordered_map< int, float > colour_scale_factors{ { CV_8S, 0.5 }, { CV_16U, 256.0 }, { CV_16S, 128.0 }, { CV_32S, 8388608.0 }, { CV_32F, 1.0 / 255.0 } };

static float colour_scale_factor_( int depth )
{
    auto found = colour_scale_factors.find( depth );
    return colour_scale_factors.cend() != found ? found->second : 1.0;
}

template < typename H >
typename std::pair< H, cv::Mat > text< H >::operator()( typename std::pair< H, cv::Mat > m )
{
    if( m.second.empty() ) { return m; }
    if( !ranges_.empty() )
    {
        for( ; range_index_ < ranges_.size() && count_ >= ranges_[range_index_].second; ++range_index_ );
        if( range_index_ == ranges_.size() ) { ++count_; return m; }
        if( count_ < ranges_[range_index_].first ) { ++count_; return m; }
    }
    text_input caption = caption_;
    if( istream_ )
    {
        const text_input* p = istream_->read();
        if( p ) { caption = *p; }
    }
    #if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
        if( !caption.text.empty() ) { cv::putText( m.second, caption.text, caption.origin, cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar( caption.colour * colour_scale_factor_( m.second.depth() ) ), 1, CV_AA ); }
    #else
        if( !caption.text.empty() ) { cv::putText( m.second, caption.text, caption.origin, cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar( caption.colour * colour_scale_factor_( m.second.depth() ) ), 1, cv::LINE_AA ); }
    #endif
    ++count_;
    return m;
}

template < typename H >
std::pair< typename text< H >::functor_t, bool > text< H >::make( const std::string& options )
{
    if( options.empty() ) { COMMA_THROW( comma::exception, "text: expected text options, got none" ); }
    const std::vector< std::string >& v = comma::split( options, ',' );
    text_input t( cv::Point( 10, 10 ), cv::Scalar( 0, 0xffff, 0xffff ), v[0] );
    if( v.size() >= 3 ) { t.origin = cv::Point( boost::lexical_cast< unsigned int >( v[1] ), boost::lexical_cast< unsigned int >( v[2] ) ); }
    if( v.size() >= 4 ) { t.colour = color_from_string( v[3] ); }
    comma::csv::options csv;
    std::vector< std::pair< unsigned int, unsigned int > > ranges;
    float font_scale = 1;
    for( unsigned int i = 4; i < v.size(); ++i )
    {
        if( v[i].substr( 0, 9 ) == "filename:" )
        { 
            csv.filename = v[i].substr( 9 );
            csv.fields = "text"; // quick and dirty
            // todo: csv options; the only difficulty: ran out of separators; e.g. use & and +? or something better? or introduce () in filter subscript semantics
        }
        else if( v[i].substr( 0, 7 ) == "ranges:" )
        { 
            std::ifstream ifs( v[i].substr( 7 ) );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "text: failed to open '" << v[i].substr( 7 ) << "'" ); }
            while( ifs.good() && !ifs.eof() )
            {
                std::string g;
                std::getline( ifs, g );
                if( comma::strip( g, " \t" ).empty() ) { continue; }
                ranges.push_back( comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( g ) );
            }
            ifs.close();
        }
        else if( v[i].substr( 0, 6 ) == "scale:" )
        {
            font_scale = boost::lexical_cast< float >( v[i].substr( 7 ) );
        }
    }
    return std::make_pair( text< H >( t, csv, ranges, font_scale ), true );
}

template < typename H >
typename std::string text< H >::usage( unsigned int indent )
{
    std::string offset( indent, ' ' );
    std::ostringstream oss;
    oss << offset << "text=<text>,[<x>,<y>],[<colour>],[filename:<filename>],[ranges:<ranges>]: print text; default x,y: 10,10\n";
    oss << offset << "                                                                          default colour: yellow\n";
    oss << offset << "     <x>,<y>: text origin pixel coordinates; default x,y: 10,10\n";
    oss << offset << "     <colour>: text colour: red, green, blue, black, white, yellow, cyan, magenta; default: yellow\n";
    oss << offset << "     filename:<filename>]: read text from a file line by line, new line for each new frame\n";
    oss << offset << "     ranges:<filename>]: file containing sorted list of non-intersecting desired ranges of frame numbers\n";
    oss << offset << "                         to put text on; <begin>,<end> pairs, where <end> is not included\n";
    oss << offset << "     scale:<factor>]: font scale, default: 1.0, see opencv putText() for more\n";
    return oss.str();
}

template class text< boost::posix_time::ptime >;
template class text< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
