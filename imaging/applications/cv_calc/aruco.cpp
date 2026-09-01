// Copyright (c) 2026 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5
    #include <opencv2/aruco.hpp>
#elif CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7 )
    #include <opencv2/objdetect/aruco_detector.hpp>
#endif
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include "../../../imaging/camera/pinhole.h"
#include "../../../imaging/camera/traits.h"
#include "../../../imaging/cv_mat/traits.h"
#include "../../../math/pose.h"
#include "../../../visiting/traits.h"
#include "aruco.h"

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

std::string options()
{
    std::ostringstream oss;
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
    oss << "    cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    oss << R"(        --dictionary,--dict=<dictionary>
        --pinhole-config,--pinhole=<config>; <config>: <filename>[:<path>]
        --output-dictionaries,--dictionaries; output list of dictionary names to stdout and exit
        --output-fields; output csv fields to stdout and exit
        --output-format; output csv format to stdout and exit
    examples
        realsense2-util color --fps 30 --width 640 \
            | cv-cat timestamp \
            | cv-cat view \
            | cv-calc aruco-detect --dict 4X4_50 \
                                   --fields block,id,marker,pose \
                                   --pinhole pinhole.json \
                                   --marker-length 0.1 \
                                   --binary 3ui,6d \
                                   --flush \
            | view-points '-;binary=3ui,6d;fields=,,id,x,y,z;weight=3')";
    return oss.str();
}

// realsense2-util color --fps 30 --width 640 --height 480 --verbose | cv-cat timestamp | cv-cat view | cv-calc aruco-detect --dict 4X4_50 --fields block,id,marker,pose --flush --pinhole pinhole.json --marker-length 0.1 --binary 3ui,6d | view-points '-;binary=3ui,6d;fields=,,id,x,y,z;weight=3' --camera-config <( echo '{"center":{"x":0,"y":0,"z":0},"world":{"translation":{"x":0,"y":0,"z":0},"rotation":{"x":0,"y":-0,"z":0}},"camera":{"translation":{"x":0,"y":0,"z":-1},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":1000,"field_of_view":45}}' )
// realsense2-util color --fps 30 --width 640 --height 480 --verbose | cv-cat timestamp | cv-cat view | cv-calc aruco-detect --dict 4X4_50 --fields block,id,marker,pose --flush --pinhole pinhole.json --marker-length 0.1 --binary 3ui,6d | csv-eval --binary 3ui,6d --fields ,,id 'id=(id+7)*2' --flush | view-points '-;binary=3ui,6d;fields=,,id,x,y,z;weight=10' --camera-config <( echo '{"center":{"x":0,"y":0,"z":0},"world":{"translation":{"x":0,"y":0,"z":0},"rotation":{"x":0,"y":-0,"z":0}},"camera":{"translation":{"x":0,"y":0,"z":-1},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":1000,"field_of_view":45}}' )

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
    #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5
        using type = cv::aruco::PREDEFINED_DICTIONARY_NAME;
    #else
        using type = cv::aruco::PredefinedDictionaryType;
    #endif
    static const std::map< std::string, type >& types()
    {
        static const std::map< std::string, type > t =  { { "4X4_50", cv::aruco::DICT_4X4_50 }
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
                                                        , { "ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL } };
                                                        // , { "APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 }
                                                        // , { "APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9 }
                                                        // , { "APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10 }
                                                        // , { "APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 }
                                                        // , { "ARUCO_MIP_36h12", cv::aruco::DICT_ARUCO_MIP_36h12 } };
        return t;
    }
    static type type_from_string( const std::string& name )
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
        float half_length = marker_length / 2.0f;
        static std::vector< cv::Point3f > obj_points = {
            cv::Point3f( -half_length,  half_length, 0 ), // top-left
            cv::Point3f(  half_length,  half_length, 0 ), // top-right
            cv::Point3f(  half_length, -half_length, 0 ), // bottom-right
            cv::Point3f( -half_length, -half_length, 0 )  // bottom-left
        };
        rvecs.resize( corners.size() );
        tvecs.resize( corners.size() );
        for( unsigned int i = 0; i < corners.size(); ++i ) { cv::solvePnP( obj_points, corners[i], camera_matrix, distortion_coeffs, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE ); }
    }
#endif 

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options )
{
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 5 )
        COMMA_THROW_BRIEF( comma::exception, "cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" );
    #else
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
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary( dictionaries::type_from_string( options.value< std::string >( "--dictionary,--dict" ) ) );
            cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        #else
            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary( dictionaries::type_from_string( options.value< std::string >( "--dictionary,--dict" ) ) );
            cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
            cv::aruco::ArucoDetector detector( dictionary, params );
        #endif
        double marker_length = options.value( "--marker-length", 0. );
        cv::Mat camera_matrix{}, distortion_coeffs{};
        if( has_pose )
        {
            COMMA_ASSERT_BRIEF( marker_length > 0, "please specify --marker-length" );
            auto s = options.value< std::string >( "--pinhole-config,--pinhole" );
            const auto& v = comma::split( options.value< std::string >( "--pinhole-config,--pinhole" ), ':', true );
            COMMA_ASSERT_BRIEF( v.size() == 1 || v.size() == 2, "expected --pinhole-config=<file>[:<path>]; got: '" << s << "'" );
            const auto& config = comma::read_json< snark::camera::pinhole::config_t >( v[0], v.size() == 2 ? v[1] : "" );
            camera_matrix = config.camera_matrix();
            if( config.distortion ) { distortion_coeffs = config.distortion->as< cv::Mat >(); }
        }
        output o;
        std::vector< std::vector< cv::Point2f > > corners;
        std::vector< int > markers;
        std::vector< std::vector< cv::Point2f > > rejected;
        std::vector< cv::Vec3d > rvecs, tvecs;
        for( ; std::cin.good() && !std::cin.eof(); ++o.block )
        {
            std::pair< boost::posix_time::ptime, cv::Mat > p = input.read< boost::posix_time::ptime >( std::cin );
            if( p.second.empty() ) { return 0; }
            #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5
                cv::aruco::detectMarkers( p.second, dictionary, corners, markers, params, rejected );
            #else
                detector.detectMarkers( p.second, corners, markers, rejected );
            #endif
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
        return 0;
    #endif
}

} } } } // namespace snark { namespace cv_calc { namespace aruco { namespace detection {
