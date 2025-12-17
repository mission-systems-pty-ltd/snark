// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2019 Vsevolod Vlaskine

#include <fstream>
#include <sstream>
#include <boost/filesystem/operations.hpp>
#include <boost/icl/interval.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include "file.h"
#include "../utils.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
file< H >::file( const get_timestamp_functor& get_timestamp
               , const std::string& type
               , bool no_header
               , const boost::optional< int >& quality
               , bool do_index
               , bool numbered
               , bool force_filenames
               , bool exit_if_done
               , const std::vector< std::string >& filenames
               , const std::vector< std::pair< unsigned int, unsigned int > >& ranges
               , const std::string& prefix )
    : get_timestamp_( get_timestamp )
    , type_( type )
    , quality_( quality )
    , do_index_( do_index )
    , numbered_( numbered )
    , prefix_( prefix )
    , force_filenames_( force_filenames )
    , exit_if_done_( exit_if_done )
    , index_( 0 )
    , filenames_( filenames )
    , filename_index_( 0 )
    , ranges_( ranges )
    , range_index_( 0 )
    , count_( 0 )
{
    snark::cv_mat::serialization::options options;
    options.no_header = no_header;
    serialization_ = snark::cv_mat::serialization( options );
}

template < typename H >
typename std::pair< H, cv::Mat > file< H >::operator()( typename std::pair< H, cv::Mat > m )
{
    if( m.second.empty() ) { return m; }
    if( !ranges_.empty() )
    {
        for( ; range_index_ < ranges_.size() && count_ >= ranges_[range_index_].second; ++range_index_ );
        if( range_index_ == ranges_.size() ) { ++count_; return exit_if_done_ ? std::pair< H, cv::Mat >() : m; }
        if( count_ < ranges_[range_index_].first ) { ++count_; return m; }
    }
    boost::posix_time::ptime timestamp = get_timestamp_( m.first );
    index_ = timestamp == previous_timestamp_ ? index_ + 1 : 0;
    previous_timestamp_ = timestamp;
    const std::string& filename = make_filename_( timestamp );
    if( filename.empty() ) { ++count_; return exit_if_done_ ? std::pair< H, cv::Mat >() : m; }
    if( type_ == "bin" )
    {
        std::ofstream ofs( filename );
        if( !ofs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
        serialization_.write( ofs, m );
    }
    else if( type_ == "gz" )
    {
        std::ofstream ofs( filename );
        if( !ofs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
        std::ostringstream oss;
        serialization_.write( oss, m );
        std::istringstream iss( oss.str() ); // todo? watch performance; do zero-copy?
		boost::iostreams::filtering_streambuf< boost::iostreams::input > os;  // todo? watch performance; create output stream on construction
		os.push( boost::iostreams::gzip_compressor( boost::iostreams::gzip_params( boost::iostreams::gzip::best_compression ) ) );
		os.push( iss );
		boost::iostreams::copy( os, ofs );
    }
    else
    {
        check_image_type( m.second, type_ );
        if( !cv::imwrite( filename, m.second, quality_ ? imwrite_params( type_, *quality_ ) : std::vector< int >() ) ) { COMMA_THROW( comma::exception, "failed to write image to '" << filename << "'" ); }
    }
    ++count_;
    return m;
}

template < typename H >
std::string file< H >::make_filename_( const boost::posix_time::ptime& t )
{
    if( numbered_ ) { return prefix_ == "" ? boost::lexical_cast< std::string >( count_ ) + '.' + type_ : prefix_ + '.' + boost::lexical_cast< std::string >( count_ ) + '.' + type_; }
    if( !force_filenames_ ) { return make_filename( t, type_, do_index_ ? boost::optional< unsigned int >( index_ ) : boost::none ); }
    if( filename_index_ >= filenames_.size() ) { return ""; }
    const std::string& filename = filenames_[ filename_index_++ ];
    const auto& dirname = boost::filesystem::path( filename ).parent_path();
    if( dirname.empty() || boost::filesystem::is_directory( dirname ) || boost::filesystem::create_directories( dirname ) ) { return filename; }
    COMMA_THROW( comma::exception, "failed to create directory '" << dirname << "' for file: '" << filename << "'" );
}

template < typename H >
std::pair< typename file< H >::functor_t, bool > file< H >::make( boost::function< boost::posix_time::ptime( const H& ) > get_timestamp, const std::string& options )
{
    COMMA_ASSERT( !options.empty(), "file: expected file type like jpg, ppm, etc" );
    const std::vector< std::string >& s = comma::split( options, ',' );
    boost::optional< int > quality;
    bool do_index = false;
    bool no_header = false;
    bool numbered = false;
    std::string prefix;
    bool exit_if_done = false;
    std::vector< std::string > filenames;
    std::vector< std::pair< unsigned int, unsigned int > > ranges;
    bool force_filenames = false;
    bool ranges_union = false;
    for( unsigned int i = 1; i < s.size(); ++i )
    {
        if( s[i] == "index" )
        {
            do_index = true;
        }
        else if( s[i] == "numbered" )
        {
            numbered = true;
        }
        else if( s[i] == "no-header" )
        { 
            no_header = true;
        }
        else if( s[i] == "exit-if-done" )
        { 
            exit_if_done = true;
        }
        else if( s[i].substr( 0, 7 ) == "prefix:" ) // quick and dirty
        {
            prefix = s[i].substr( 7 );
        }
        else if( s[i].substr( 0, 10 ) == "filenames:" ) // quick and dirty
        {
            force_filenames = true;
            std::ifstream ifs( s[i].substr( 10 ) );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "file: failed to open '" << s[i].substr( 10 ) << "'" ); }
            while( ifs.good() && !ifs.eof() )
            {
                std::string g;
                std::getline( ifs, g );
                if( comma::strip( g, " \t" ).empty() ) { continue; }
                filenames.push_back( g );
            }
            ifs.close();
        }
        else if( s[i].substr( 0, 7 ) == "frames:" ) // quick and dirty
        { 
            std::ifstream ifs( s[i].substr( 7 ) );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "file: failed to open '" << s[i].substr( 7 ) << "'" ); }
            while( ifs.good() && !ifs.eof() )
            {
                std::string g;
                std::getline( ifs, g );
                if( comma::strip( g, " \t" ).empty() ) { continue; }
                auto f = boost::lexical_cast< unsigned int >( g );
                ranges.push_back( std::make_pair( f, f + 1 ) );
            }
            ifs.close();
        }
        else if( s[i].substr( 0, 7 ) == "ranges:" ) // quick and dirty
        { 
            std::ifstream ifs( s[i].substr( 7 ) );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "file: failed to open '" << s[i].substr( 7 ) << "'" ); }
            while( ifs.good() && !ifs.eof() )
            {
                std::string g;
                std::getline( ifs, g );
                if( comma::strip( g, " \t" ).empty() ) { continue; }
                ranges.push_back( comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( g ) );
                if( ranges.back().first >= ranges.back().second ) { COMMA_THROW( comma::exception, "expected ranges, got [" << ranges.back().first << "," << ranges.back().second << ") in line " << ( ranges.size() - 1 ) << ", which is an invalid range" ); }
            }
            ifs.close();
        }
        else if( s[i].substr( 0, 12 ) == "ranges-union" || s[i].substr( 0, 5 ) == "union" ) // quick and dirty
        {
            ranges_union = true;
        }
        else
        {
            quality = boost::lexical_cast< int >( s[i] );
        }
    }
    if( numbered && do_index ) { COMMA_THROW( comma::exception, "numbered and index are mutually exclusive in 'file=" << options << "'" ); }
    if( !numbered && prefix != "" ) { COMMA_THROW( comma::exception, "prefix is implemented only for numbered (just ask); got: 'file=" << options << "'" ); }
    if( numbered && !filenames.empty() ) { COMMA_THROW( comma::exception, "numbered and filenames:... are mutually exclusive in 'file=" << options << "'" ); }
    if( do_index && !filenames.empty() ) { COMMA_THROW( comma::exception, "index and filenames:... are mutually exclusive in 'file=" << options << "'" ); }
    if( filenames.empty() && ranges.empty() && exit_if_done ) { COMMA_THROW( comma::exception, "exit-if-done, but no filenames, frames, or ranges in 'file=" << options << "'" ); }
    if( !ranges.empty() )
    {
        if( ranges_union )
        {
            boost::icl::interval_set< unsigned int > intervals;
            for( const auto& range: ranges ) { intervals += boost::icl::interval< unsigned int >::right_open( range.first, range.second ); }
            ranges.clear();
            for( const auto& interval: intervals ) { ranges.push_back( std::make_pair( interval.lower(), interval.upper() ) ); }
        }
        else
        {
            for( unsigned int i = 1; i < ranges.size(); ++i )
            {
                if( ranges[i].first < ranges[i-1].second ) { COMMA_THROW( comma::exception, "expected sorted non-intersecting ranges, got [" << ranges[i-1].first << "," << ranges[i-1].second << ") followed by [" << ranges[i].first << "," << ranges[i].second << "); specify 'ranges-union' to flatten" ); }
            }
        }
    }
    return std::make_pair( file< H >( get_timestamp, s[0], no_header, quality, do_index, numbered, force_filenames, exit_if_done, filenames, ranges, prefix ), false );
}

template < typename H >
std::string file< H >::usage( unsigned int i )
{
    std::ostringstream oss;
    std::string indent( i, ' ' );
    oss << indent << "file=<format>[,<quality>][,<options>]:\n";
    oss << indent << "    write images to files with timestamp as name in the specified format;\n";
    oss << indent << "    if input images have no timestamp, system time is used\n";
    oss << indent << "    <format>\n";
    oss << indent << "        - anything that opencv imwrite can take, e.g. jpg, ppm, png, tiff etc\n";
    oss << indent << "        - 'bin' to write image as binary in cv-cat format\n";
    oss << indent << "        - 'gz' to write image as compressed binary\n";
    oss << indent << "    <quality>: for jpg files only, compression quality from 0 (smallest) to 100 (best)\n";
    oss << indent << "    <options>\n";
    oss << indent << "        exit-if-done: exit if filenames, ranges, or frames specified and the end of one of those lists is reached\n";
    oss << indent << "        index: for each timestamp, files will be named as: <timestamp>.<index>.<extension>,\n";
    oss << indent << "               e.g: 20170101T000000.123456.0.png, 20170101T000000.123456.1.png, etc\n";
    oss << indent << "        no-header: makes sense only for 'bin' format; if present, write image without header\n";
    oss << indent << "        numbered: output filenames will look like 0.png, 1.png, etc, i.e. <filename>: <frame-number>.<extension>\n";
    oss << indent << "        prefix:<prefix>: if numbered, output filenames like blah.0.png, blah.1.png, etc\n";
    oss << indent << "        filenames:<filenames>: file with a list of filenames\n";
    oss << indent << "        frames:<filename>: file with a sorted list of unique desired frame numbers to save\n";
    oss << indent << "        ranges:<filename>: file with a sorted list of non-intersecting (but see ranges-union) desired ranges\n";
    oss << indent << "                           of frame numbers to save as <begin>,<end> pairs, where <end> is not included\n";
    oss << indent << "        ranges-union,union: find union of ranges, even if they are not sorted and intersecting\n";
    return oss.str();
}

template class file< boost::posix_time::ptime >;
template class file< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
