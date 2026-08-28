// Copyright (c) 2026 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5
    #include <opencv2/aruco.hpp>
#elif CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7 )
    #include <opencv2/objdetect/aruco_detector.hpp>
#endif
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include "../../../imaging/cv_mat/traits.h"
#include "aruco.h"

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

std::string options()
{
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
    oss << "    cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    std::ostringstream oss;
    oss << "        --dictionary,--dict=<dictionary>" << std::endl;
    oss << "        --output-corners-in-one-record,--all; output all marker corners in a single csv record," << std::endl;
    oss << "                                              assuming that there always are four corners in a detection" << std::endl;
    oss << "        --output-dictionaries,--dictionaries; output list of dictionary names to stdout and exit" << std::endl;
    oss << "        --output-fields; output csv fields to stdout and exit" << std::endl;
    oss << "        --output-format; output csv format to stdout and exit" << std::endl;
    // todo? output rejected?
    // todo: cv::aruco::DICT_4X4_250
    return oss.str();
}

struct output
{
    boost::posix_time::ptime t;
    unsigned int block{0};
    unsigned int id{0};
    unsigned int marker{0};
    cv::Point2f corner;
};

struct output_all
{
    boost::posix_time::ptime t;
    unsigned int block{0};
    unsigned int id{0};
    unsigned int marker{0};
    std::array< cv::Point2f, 4 > corners;
};

#if CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 5 )

struct dictionaries
{
    static const std::map< std::string, cv::aruco::PredefinedDictionaryType > types()
    {
        static const std::map< std::string, cv::aruco::PredefinedDictionaryType > t =   { { "4X4_50", cv::aruco::DICT_4X4_50 }
                                                                                        , { "4X4_100", cv::aruco::DICT_4X4_100 }
                                                                                        , { "4X4_250", cv::aruco::DICT_4X4_250 }
                                                                                        , { "4X4_1000", cv::aruco::DICT_4X4_1000 }
                                                                                        , { "5X5_50", cv::aruco::DICT_5X5_50 }
                                                                                        , { "5X5_100", cv::aruco::DICT_5X5_100 }
                                                                                        , { "5X5_250", cv::aruco::DICT_5X5_250 }
                                                                                        , { "5X5_1000", cv::aruco::DICT_5X5_1000 }
                                                                                        , { "6X6_50", cv::aruco::DICT_6X6_50 }
                                                                                        , { "6X6_100", cv::aruco::DICT_6X6_100 }
                                                                                        , { "6X6_250", cv::aruco::DICT_6X6_250 }
                                                                                        , { "6X6_1000", cv::aruco::DICT_6X6_1000 }
                                                                                        , { "7X7_50", cv::aruco::DICT_7X7_50 }
                                                                                        , { "7X7_100", cv::aruco::DICT_7X7_100 }
                                                                                        , { "7X7_250", cv::aruco::DICT_7X7_250 }
                                                                                        , { "7X7_1000", cv::aruco::DICT_7X7_1000 }
                                                                                        , { "ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL }
                                                                                        , { "APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 }
                                                                                        , { "APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9 }
                                                                                        , { "APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10 }
                                                                                        , { "APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 }
                                                                                        , { "ARUCO_MIP_36h12", cv::aruco::DICT_ARUCO_MIP_36h12 } };
        return t;
    }
    static cv::aruco::PredefinedDictionaryType type_from_string( const std::string& name )
    {
        auto i = types().find( name );
        COMMA_ASSERT_BRIEF( i != types().end(), "expected a dictionary name, got: '" << name << "'" );
        return i->second;
    }
};

#endif

} } } } // namespace snark { namespace cv_calc { namespace aruco { namespace detection {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_calc::aruco::detection::output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corner", p.corner );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corner", p.corner );
    }
};

template <> struct traits< snark::cv_calc::aruco::detection::output_all >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::aruco::detection::output_all& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::detection::output_all& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options )
{
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
        COMMA_THROW_BRIEF( comma::exception, "cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" );
    #endif
    bool output_corners_in_one_record = options.exists( "--output-corners-in-one-record,--all" );
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( output_corners_in_one_record ? comma::csv::names< output_all >() : comma::csv::names< output >(), ',' ) << std::endl; return 0; };
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format( output_corners_in_one_record ? comma::csv::format::value< output_all >() : comma::csv::format::value< output >() ).collapsed_string() << std::endl; return 0; }
    if( options.exists( "--output-dictionaries,--dictionaries" ) ) { for( const auto& t: dictionaries::types() ) { std::cout << t.first << "," << t.second << std::endl; } return 0; }
    snark::cv_mat::serialization input( input_options );
    comma::csv::output_stream< output > ostream( std::cout, comma::csv::options( options ) );
    comma::csv::output_stream< output_all > ostream_all( std::cout, comma::csv::options( options ) );
    bool flush = options.exists( "--flush" );
    #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5
        COMMA_THROW( comma::exception, "opencv " << CV_VERSION << ": todo soon"  );
    #else
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary( dictionaries::type_from_string( options.value< std::string >( "--dictionary,--dict" ) ) );
        cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
        cv::aruco::ArucoDetector detector( dictionary, params );
        output o;
        output_all oa;
        for( ; std::cin.good() && !std::cin.eof(); ++o.block, ++oa.block )
        {
            std::pair< boost::posix_time::ptime, cv::Mat > p = input.read< boost::posix_time::ptime >( std::cin );
            if( p.second.empty() ) { return 0; }
            std::vector< std::vector< cv::Point2f > > corners;
            std::vector< int > markers;
            std::vector< std::vector< cv::Point2f > > rejectedImgPoints;
            detector.detectMarkers( p.second, corners, markers, rejectedImgPoints );
            if( output_corners_in_one_record )
            {
                for( unsigned i = 0; i < markers.size(); ++i )
                {
                    oa.t = p.first;
                    oa.id = i;
                    oa.marker = markers[i];
                    for( unsigned int j = 0; j < corners[i].size(); ++j ) { oa.corners[j] = corners[i][j]; }
                    ostream_all.write( oa );
                }
            }
            else
            {
                for( unsigned i = 0; i < markers.size(); ++i )
                {
                    o.t = p.first;
                    o.id = i;
                    o.marker = markers[i];
                    for( unsigned int j = 0; j < corners[i].size(); ++j ) { o.corner = corners[i][j]; ostream.write( o ); }
                }
            }
            if( flush ) { std::cout.flush(); }
        }
    #endif
    return 0;
}

} } } } // namespace snark { namespace cv_calc { namespace aruco { namespace detection {
