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
#include "../../../math/pose.h"
#include "../../../visiting/traits.h"
#include "aruco.h"

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

std::string options()
{
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
    oss << "    cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    std::ostringstream oss;
    oss << "        --dictionary,--dict=<dictionary>" << std::endl;
    oss << "        --pinhole-config,--pinhole=<config>" << std::endl;
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
    std::array< cv::Point2f, 4 > corners;
    snark::pose pose;
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
        v.apply( "corners", p.corners );
        v.apply( "pose", p.pose );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
        v.apply( "pose", p.pose );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

#if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7 ) // how painful...
    static void estimate_poses( const std::vector< std::vector< cv::Point2f > >& corners, float marker_length, const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, std::vector< cv::Vec3d >& rvecs, std::vector< cv::Vec3d >& tvecs )
    { 	
        cv::aruco::estimatePoseSingleMarkers( corners, marker_length, camera_matrix, distortion_coeffs, rvecs, tvecs );
    }
#else
    static void estimate_poses( const std::vector< std::vector< cv::Point2f > >& corners, float marker_length, const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, std::vector< cv::Vec3d >& rvecs, std::vector< cv::Vec3d >& tvecs )
    {
        // todo
        rvecs.resize( corners.size() );
        tvecs.resize( corners.size() );
    }
#endif 

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options )
{
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
        COMMA_THROW_BRIEF( comma::exception, "cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" );
    #endif
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output >(), ',' ) << std::endl; return 0; };
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format( comma::csv::format::value< output >() ).collapsed_string() << std::endl; return 0; }
    if( options.exists( "--output-dictionaries,--dictionaries" ) ) { for( const auto& t: dictionaries::types() ) { std::cout << t.first << "," << t.second << std::endl; } return 0; }
    snark::cv_mat::serialization input( input_options );
    comma::csv::options csv( options );
    comma::csv::output_stream< output > ostream( std::cout, csv );
    bool flush = options.exists( "--flush" );
    bool has_corners = csv.fields.empty() || csv.has_paths( "corners" );
    bool has_pose = csv.fields.empty() || csv.has_paths( "pose" );
    #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5
        COMMA_THROW( comma::exception, "opencv " << CV_VERSION << ": todo soon"  );
    #else
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary( dictionaries::type_from_string( options.value< std::string >( "--dictionary,--dict" ) ) );
        cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
        cv::aruco::ArucoDetector detector( dictionary, params );
        double marker_length = options.value( "--marker-length", 0. );
        std::string camera_config = options.value< std::string >( "--camera-config", "" );
        cv::Mat camera_matrix, distortion_coeffs;
        COMMA_ASSERT_BRIEF( !has_pose || !camera_matrix.empty(), "asked to calculate marker poses, but got no --camera-config" );
        

        // todo!!!


        output o;
        std::vector< std::vector< cv::Point2f > > corners;
        std::vector< int > markers;
        std::vector< std::vector< cv::Point2f > > rejected;
        std::vector< cv::Vec3d > rvecs, tvecs;
        for( ; std::cin.good() && !std::cin.eof(); ++o.block )
        {
            std::pair< boost::posix_time::ptime, cv::Mat > p = input.read< boost::posix_time::ptime >( std::cin );
            if( p.second.empty() ) { return 0; }
            detector.detectMarkers( p.second, corners, markers, rejected );
            if( has_pose ) { estimate_poses( corners, marker_length, camera_matrix, distortion_coeffs, rvecs, tvecs ); }
            for( unsigned i = 0; i < markers.size(); ++i )
            {
                o.t = p.first;
                o.id = i;
                o.marker = markers[i];
                if( has_corners ) { for( unsigned int j = 0; j < corners[i].size(); ++j ) { o.corners[j] = corners[i][j]; } }
                if( has_pose ) { o.pose = snark::pose( Eigen::Vector3d( tvecs[i][0], tvecs[i][1], tvecs[i][2] ), roll_pitch_yaw::from_rodriques( tvecs[i][0], tvecs[i][1], tvecs[i][2] ) ); }
                ostream.write( o );
            }
            if( flush ) { std::cout.flush(); }
        }
    #endif
    return 0;
}

} } } } // namespace snark { namespace cv_calc { namespace aruco { namespace detection {
