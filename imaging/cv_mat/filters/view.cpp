// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/string/string.h>
#include "../utils.h"
#include "view.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
std::pair< typename view< H >::functor_t, bool > view< H >::make( const std::string& options, const timestamp_functor_t& get_timestamp )
{
    double default_delay = 0.001; // todo!
    double delay = default_delay;
    std::string n;
    std::string suffix="ppm";
    boost::optional< std::pair< int, int > > position;
    boost::optional< std::pair< int, int > > size;
    const std::vector< std::string >& w = comma::split( options, ',' );
    if( w.size() > 0 && !w[0].empty() ) { delay = w[0] == "stay" ? -1 : boost::lexical_cast< double >( w[0] ); }
    if( w.size() > 1 && !w[1].empty() ) { n = w[1]; }
    if( w.size() > 2 && !w[2].empty() ) { suffix = w[2]; }
    if( w.size() > 4 && !w[3].empty() && !w[4].empty() ) { position = std::make_pair( boost::lexical_cast< int >( w[3] ), boost::lexical_cast< int >( w[4] ) ); }
    if( w.size() > 6 && !w[5].empty() && !w[6].empty() ) { size = std::make_pair( boost::lexical_cast< int >( w[5] ), boost::lexical_cast< int >( w[6] ) ); }
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( view< H >( get_timestamp, n, delay, suffix, position, size ), _1 ), false );
}

static std::string _make_name() { static unsigned int count = 0; return "view_" + boost::lexical_cast< std::string >( count++ ); } // uber-quick and dirty

template < typename H >
view< H >::view( const typename view< H >::timestamp_functor_t& get_timestamp
               , const std::string& title
               , double delay
               , const std::string& suffix
               , const boost::optional< std::pair< int, int > >& window_position
               , const boost::optional< std::pair< int, int > >& window_size )
    : _get_timestamp( get_timestamp )
    , _name( _make_name() )
    , _delay( delay >= 0 ? int( delay * 1000 ) : -1 )
    , _suffix( suffix )
{
    #if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2 // pain
        cv::namedWindow( title.empty() ? &_name[0] : &title[0] );
    #else
        cv::namedWindow( &_name[0], cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL );
        std::string t = title.empty() ? std::string( "view" ) : title;
        cv::setWindowTitle( &_name[0], &t[0] );
        if( window_position ) { cv::moveWindow( &_name[0], window_position->first, window_position->second ); }
        if( window_size ) { cv::resizeWindow( &_name[0], window_size->first, window_size->second ); }
    #endif
}

template < typename H >
std::pair< H, cv::Mat > view< H >::operator()( std::pair< H, cv::Mat > m )
{
    cv::imshow( &_name[0], m.second );
    char c = cv::waitKey( _delay );
    if( c == 27 ) { return std::pair< H, cv::Mat >(); } // HACK to notify application to exit
    if( c == ' ' ) { cv::imwrite( snark::cv_mat::make_filename( _get_timestamp( m.first ), _suffix ), m.second ); }
    if( c>='0' && c<='9') { cv::imwrite( snark::cv_mat::make_filename( _get_timestamp( m.first ), _suffix, unsigned( c - '0' ) ), m.second ); }
    return m;
}

template < typename H >
std::string view< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "view[=<wait-interval>[,<name>[,<suffix>[,<offset/x>,<offset/y>[,<size/x>,<size/y>]]]]]: view image;\n";
    oss << i << "                        press <space> to save image (timestamp or system time as filename); \n";
    oss << i << "                        press <esc>: to close\n";
    oss << i << "                        press numerical '0' to '9' to add the id (0-9) to the file name:\n";
    oss << i << "                            <timestamp>.<id>.<suffix>\n";
    oss << i << "                        press any other key to show the next frame\n";
    oss << i << "                        <wait-interval>: a hack for now; seconds to wait for image display or key press\n";
    oss << i << "                                         e.g. view=0.1: wait for 100ms\n";
    oss << i << "                                         view=-1 or view=stay: wait indefinitely\n";
    oss << i << "                                         default: 0.001 (1 millisecond)\n";
    oss << i << "                        <name>: view window name; default: the number of view occurence in the filter string\n";
    oss << i << "                        <suffix>: image suffix type e.g. png, default ppm\n";
    oss << i << "                        <offset/x>,<offset/y>: window position on screen in pixels\n";
    oss << i << "                        <size/x>,<size/y>: window size on screen in pixels (not very useful, but it's there)\n";
    oss << i << "        attention! it seems that lately using cv::imshow() in multithreaded context has been broken\n";
    oss << i << "                   in opencv or in underlying x window stuff therefore,\n";
    oss << i << "                   instead of: cv-cat 'view;do-something;view'\n";
    oss << i << "                          use: cv-cat 'view;do-something' | cv-cat 'view'\n";
    return oss.str();
}

template struct view< boost::posix_time::ptime >;
template struct view< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {