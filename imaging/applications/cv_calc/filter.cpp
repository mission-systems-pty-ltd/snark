// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <functional>
#include <memory>
#include <sstream>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include <comma/base/none.h>
#include <comma/csv/ascii.h>
#include <comma/csv/binary.h>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include <comma/string/split.h>
#include <comma/visiting/traits.h>
#include "../../../imaging/cv_mat/filters.h"
#include "../../../imaging/cv_mat/traits.h"
#include "filter.h"

namespace snark { namespace cv_calc { namespace filter {
    
std::string options()
{
    return R"(

        todo

        --filters=<filters>[;<csv-options>]
            <filters>: filters filename, or socket, etc
            <csv-options>: ascii only for now; 'filters'
                           has to be the last field;
                           default fields: 'filters', then
                           all filters will be applied to
                           the first input image (i.e. block 0)
                fields: t,block,count,filters
                    t      : timestamp
                    block  : number of input image to apply to
                    count  : number of times to output image
                    filters: sequence of filters as in cv-cat )";
}

class reader
{
    public:
        struct record
        {
            boost::posix_time::ptime t;
            std::uint32_t block{0};
            std::uint32_t count{1};
            std::string filters;

            bool same_as( const record& rhs ) const { return t == rhs.t && block == rhs.block; }
        };

        reader( const comma::csv::options& csv )
            : _ascii( csv )
        {
            const auto& v = comma::split( csv.fields, ',' );
            for( ; _index < v.size() && v[_index] != "filters"; ++_index )
            {
                if( v[_index] == "t" ) { COMMA_THROW_BRIEF( comma::exception, "timestamp field 't': todo, just ask" ); }
            }
            COMMA_ASSERT_BRIEF( _index == v.size() - 1, "currently, only the last field can be 'filters'; got: '" << csv.fields << "'" );
        }

        boost::optional< record > read( std::istream& is ) const
        {
            std::string line;
            for( ; is.good() && line.empty(); std::getline( is, line ) );
            if( line.empty() ) { return boost::none; }
            if( _index == 0 ) { return boost::optional< record >( record{block: 0, filters: line} ); }
            boost::optional< record > r = _ascii.get( line );
            const auto& s = comma::split( line, ',' );
            r->filters = comma::strip( comma::join( s.begin() + _index, s.end(), ',' ), "\"" ); // quick and dirty; todo: implement in comma::string or alike
            return r;
        }

    private:
        comma::csv::ascii< record > _ascii;
        unsigned int _index{0};
};

} } } // namespace snark { namespace cv_calc { namespace filter {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_calc::filter::reader::record >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::filter::reader::record& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "count", p.count );
    }

    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::filter::reader::record& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "count", p.count );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace filter {

typedef snark::cv_mat::serialization::header::buffer_t first_t; // typedef boost::posix_time::ptime first_t;
typedef std::pair< first_t, cv::Mat > pair_t;
typedef snark::cv_mat::filter_with_header filter_t; // typedef snark::cv_mat::filter filter_t;
typedef snark::cv_mat::filters_with_header filters_t; // typedef snark::cv_mat::filters filters_t;
typedef std::function< boost::posix_time::ptime( const snark::cv_mat::serialization::header::buffer_t& ) > get_timestamp_t;

static pair_t filtered( const pair_t p, const std::string& filter_string, const get_timestamp_t& get_timestamp )
{
    const std::vector< filter_t >& filters = filters_t::make( filter_string, get_timestamp );
    pair_t r;
    r.first = p.first;
    p.second.copyTo( r.second );
    for( auto& f: filters ) { r = f( r ); }
    return r;
}

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    snark::cv_mat::serialization input_serialization( input_options );
    snark::cv_mat::serialization output_serialization( output_options );
    const comma::csv::binary< snark::cv_mat::serialization::header >* binary = input_serialization.header_binary();
    comma::csv::options csv( options );
    comma::csv::options filters_csv( comma::name_value::parser( "filename" ).get< comma::csv::options >( options.value< std::string >( "--filters" ) ) );
    if( filters_csv.fields.empty() ) { filters_csv.fields = "filters"; }
    COMMA_ASSERT_BRIEF( !filters_csv.binary(), "--filters='...;binary=...': todo" );
    comma::io::istream is( filters_csv.filename, filters_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
    COMMA_ASSERT_BRIEF( is->good(), "failed to open '" << filters_csv.filename << "'" );
    filter::reader reader( filters_csv );
    boost::optional< filter::reader::record > last = comma::silent_none< filter::reader::record >();
    auto get_timestamp = [&]( const snark::cv_mat::serialization::header::buffer_t& h )->boost::posix_time::ptime
    {
        if( h.empty() || !binary ) { return boost::posix_time::not_a_date_time; }
        snark::cv_mat::serialization::header d;
        return binary->get( d, &h[0] ).timestamp;
    };
    while( std::cin.good() && !std::cin.eof() )
    {
        pair_t p = input_serialization.read< first_t >( std::cin );
        if( p.second.empty() ) { return 0; }
        if( last ) { for( unsigned int i = 0; i < last->count; ++i ) { output_serialization.write_to_stdout( filtered( p, last->filters, get_timestamp ), csv.flush ); } }
        while( is->good() )
        {
            auto r = reader.read( *is );
            if( !r ) { return 0; }
            if( last && !r->same_as( *last ) ) { last = *r; break; }
            for( unsigned int i = 0; i < r->count; ++i ) { output_serialization.write_to_stdout( filtered( p, r->filters, get_timestamp ), csv.flush ); }
        }
    }
    if( !input_serialization.last_error().empty() ) { comma::say() << input_serialization.last_error() << std::endl; }
    if( !output_serialization.last_error().empty() ) { comma::say() << output_serialization.last_error() << std::endl; }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace filter {
