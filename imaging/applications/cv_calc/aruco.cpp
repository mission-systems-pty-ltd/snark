// Copyright (c) 2026 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cinttypes>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#ifdef SNARK_OPENCV_CONTRIB
    #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
        #include <opencv2/aruco.hpp>
    #elif CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7 )
        #include <opencv2/objdetect/aruco_detector.hpp>
    #endif
#endif // SNARK_OPENCV_CONTRIB
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include "../../../imaging/camera/pinhole.h"
#include "../../../imaging/camera/traits.h"
#include "../../../imaging/cv_mat/traits.h"
#include "../../../math/pose.h"
#include "../../../math/rotation_matrix.h"
#include "../../../visiting/traits.h"
#include "aruco.h"

namespace snark { namespace cv_calc { namespace aruco {
    
namespace detection {

std::string options()
{
    std::ostringstream oss;
    #ifndef SNARK_OPENCV_CONTRIB
    oss << "    cv-calc built with cmake flag snark_build_imaging_opencv_contrib=OFF, which does not support aruco detection" << std::endl;
    #endif // #ifndef SNARK_OPENCV_CONTRIB
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
    oss << "    cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    oss << R"(        --dictionary,--dict=<dictionary>
        --pinhole-config,--pinhole=[<config>]; <config>: <filename>[:<path>]
        --output-dictionaries,--dictionaries; output list of dictionary names to stdout and exit
        --output-fields; output csv fields to stdout and exit
        --output-format; output csv format to stdout and exit
    examples
        realsense2-util color --fps 30 --width 640 \
            | cv-cat timestamp \
            | cv-cat view \
            | cv-calc aruco-detect --dict 4X4_50 \
                                   --fields block,marker,pose \
                                   --pinhole pinhole.json \
                                   --marker-length 0.1 \
                                   --binary 2ui,6d \
                                   --flush \
            | view-points '-;binary=2ui,6d;fields=,id,x,y,z;weight=3')";
    return oss.str();
}

// realsense2-util color --fps 30 --width 640 --height 480 --verbose | cv-cat timestamp | cv-cat view | cv-calc aruco-detect --dict 4X4_50 --fields block,id,marker,pose --flush --pinhole pinhole.json --marker-length 0.1 --binary 3ui,6d | view-points '-;binary=3ui,6d;fields=,,id,x,y,z;weight=3' --camera-config <( echo '{"center":{"x":0,"y":0,"z":0},"world":{"translation":{"x":0,"y":0,"z":0},"rotation":{"x":0,"y":-0,"z":0}},"camera":{"translation":{"x":0,"y":0,"z":-1},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":1000,"field_of_view":45}}' )
// realsense2-util color --fps 30 --width 640 --height 480 --verbose | cv-cat timestamp | cv-cat view | cv-calc aruco-detect --dict 4X4_50 --fields block,id,marker,pose --flush --pinhole pinhole.json --marker-length 0.1 --binary 3ui,6d | csv-eval --binary 3ui,6d --fields ,,id 'id=(id+7)*2' --flush | view-points '-;binary=3ui,6d;fields=,,id,x,y,z;weight=10' --camera-config <( echo '{"center":{"x":0,"y":0,"z":0},"world":{"translation":{"x":0,"y":0,"z":0},"rotation":{"x":0,"y":-0,"z":0}},"camera":{"translation":{"x":0,"y":0,"z":-1},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":1000,"field_of_view":45}}' )

struct output
{
    boost::posix_time::ptime t;
    std::uint32_t block{0};
    std::uint32_t marker{0};
    std::array< cv::Point2f, 4 > corners;
    snark::pose pose;
};

} // namespace detection {

namespace localization {

std::string options()
{
    std::ostringstream oss;
    #ifndef SNARK_OPENCV_CONTRIB
    oss << "    cv-calc built with cmake flag snark_build_imaging_opencv_contrib=OFF, which does not support aruco localization" << std::endl;
    #endif // #ifndef SNARK_OPENCV_CONTRIB
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
    oss << "    cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    oss << R"(        --dictionary,--dict=<dictionary>
        --marker-anchor-id,--anchor=<id>; output camera pose relative to this marker
        --markers-min-number,--min-number-of-markers=<n>; default=1
        --pinhole-config,--pinhole=<config>; <config>: <filename>[:<path>]
        --orientation-frame,--frame=<which>; default=camera
            <which>
                camera: right-down-forward, i.e. camera looking along z axis
                frd: forward-right-down, i.e. camera looking along x axis
        --output-fields; output csv fields to stdout and exit
        --output-format; output csv format to stdout and exit
    examples
        realsense2-util color --fps 30 --width 640 \
            | cv-cat timestamp \
            | cv-cat view \
            | cv-calc aruco-localize --dict 4X4_50 \
                                     --pinhole pinhole.json \
                                     --marker-length 0.1 \
                                     --marker-id 2 \
                                     --flush \
            | view-points '-;fields=...;weight=3')";
    return oss.str();
}

// view-points '0.csv;fields=,,,x,y,z;weight=2' '0.csv;fields=,,,x,y,z;shape=lines' '0.csv;fields=,,,x,y,z,roll,pitch,yaw;shape=axes;length=0.01' <( echo 0,0,0 )';fields=x,y,z;shape=axes;length=0.2' <( echo 0,0,0 )';weight=10;label=0,0,0'

// realsense2-util color --fps 30 --width 640             | cv-cat timestamp             | cv-cat view             | cv-calc aruco-detect --dict 4X4_50                                    --fields pose --max-number-of-markers 1                                    --pinhole pinhole.json --anchor 0                                   --marker-length 0.1                                    --binary 6d                                    --flush     | tee detect.bin        | view-points '-;binary=6d;fields=x,y,z,roll,pitch,yaw;shape=axes;length=0.05;weight=3;size=1' <(echo 0)';fields=x;shape=axes;length=0.1' <( echo 0 )';fields=x;weight=10;label=0,0,0' --camera-config <( echo '{"center":{"x":0.0802531689,"y":-0.0248078629,"z":0.549671054},"world":{"translation":{"x":-0.263532609,"y":0.406255633,"z":-0.299200088},"rotation":{"x":1.99405885,"y":0.0683527067,"z":0.00327160885}},"camera":{"translation":{"x":0.274289042,"y":-0.428598017,"z":-0.908777118},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":8.2387313842773438,"field_of_view":45}}' )

// realsense2-util color --fps 30 --width 640   | cv-cat view | cv-cat resize=2             | cv-calc aruco-detect --dict 4X4_50                                    --fields pose --max-number-of-markers 1                                    --pinhole pinhole.2.json --anchor 0                                   --marker-length 0.1                                    --binary 6d                                    --flush    | view-points '-;binary=6d;fields=x,y,z,roll,pitch,yaw;shape=axes;length=0.1;weight=3;size=1' <(echo 0)';fields=x;shape=axes;length=0.1' <( echo 0 )';fields=x;weight=10;label=0,0,0' --camera-config <( echo '{"center":{"x":0.0802531689,"y":-0.0248078629,"z":0.549671054},"world":{"translation":{"x":-0.263532609,"y":0.406255633,"z":-0.299200088},"rotation":{"x":1.99405885,"y":0.0683527067,"z":0.00327160885}},"camera":{"translation":{"x":0.274289042,"y":-0.428598017,"z":-0.908777118},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":8.2387313842773438,"field_of_view":45}}' )
// realsense2-util color --fps 30 --width 640   | cv-cat view | cv-cat resize=2             | cv-calc aruco-detect --dict 4X4_50                                    --fields pose --max-number-of-markers 1                                    --pinhole pinhole.2.json --anchor 0                                   --marker-length 0.1                                    --binary 6d                                    --flush    | csv-paste value='0,0,0,0,0,0;binary=6d' '-;binary=6d' --flush | points-frame --fields x,y,z,roll,pitch,yaw,frame --binary 12d --to --flush | csv-shuffle --fields ,,,,,,,,,,,,x,y,z,roll,pitch,yaw --binary 18d -e --flush        | view-points '-;binary=6d;fields=x,y,z,roll,pitch,yaw;shape=axes;length=0.05;weight=3;size=1' <(echo 0)';fields=x;shape=axes;length=0.1' <( echo 0 )';fields=x;weight=10;label=0,0,0' --camera-config <( echo '{"center":{"x":0.0802531689,"y":-0.0248078629,"z":0.549671054},"world":{"translation":{"x":-0.263532609,"y":0.406255633,"z":-0.299200088},"rotation":{"x":1.99405885,"y":0.0683527067,"z":0.00327160885}},"camera":{"translation":{"x":0.274289042,"y":-0.428598017,"z":-0.908777118},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":8.2387313842773438,"field_of_view":45}}' )

// realsense2-util color --fps 30 --width 640   | cv-cat view | cv-cat resize=2             | cv-calc aruco-detect --dict 4X4_50                                    --fields t,pose --max-number-of-markers 1                                    --pinhole pinhole.2.json --anchor 0                                   --marker-length 0.1                                    --binary t,6d                                    --flush    | csv-paste '-;binary=t,6d' value='0,0,0,0,0,0;binary=6d' --flush | points-frame --fields ,frame,x,y,z,roll,pitch,yaw --binary t,12d --to --flush | csv-shuffle --fields t,,,,,,,,,,,,,x,y,z,roll,pitch,yaw --binary t,18d -e --flush | math-kalman-filter --measurement-size 3                                  --fields t,measurement --binary t,6d                                  --measurement-noise 0.05                                  --process-noise 0.000001 --flush | csv-shuffle --fields t,,,,roll,pitch,yaw,x,y,z --output-fields t,x,y,z,roll,pitch,yaw --binary t,12d --flush
// realsense2-util color --fps 30 --width 640   | cv-cat view | cv-cat resize=2             | cv-calc aruco-detect --dict 4X4_50                                    --fields t,pose --max-number-of-markers 1                                    --pinhole pinhole.2.json --anchor 0                                   --marker-length 0.1                                    --binary t,6d                                    --flush    | csv-paste '-;binary=t,6d' value='0,0,0,0,0,0;binary=6d' --flush | points-frame --fields ,frame,x,y,z,roll,pitch,yaw --binary t,12d --to --flush | csv-shuffle --fields t,,,,,,,,,,,,,x,y,z,roll,pitch,yaw --binary t,18d -e --flush | math-kalman-filter --measurement-size 3                                  --fields t,measurement --binary t,6d                                  --measurement-noise 0.05                                  --process-noise 0.000001 --flush | csv-shuffle --fields t,,,,roll,pitch,yaw,x,y,z --output-fields t,x,y,z,roll,pitch,yaw --binary t,12d --flush | view-points '-;binary=6d;fields=x,y,z,roll,pitch,yaw;shape=axes;length=0.1;weight=5;size=1' <(echo 0)';fields=x;shape=axes;length=0.1' <( echo 0 )';fields=x;weight=10;label=0,0,0' --camera-config <( echo '{"center":{"x":0.0802531689,"y":-0.0248078629,"z":0.549671054},"world":{"translation":{"x":-0.263532609,"y":0.406255633,"z":-0.299200088},"rotation":{"x":1.99405885,"y":0.0683527067,"z":0.00327160885}},"camera":{"translation":{"x":0.274289042,"y":-0.428598017,"z":-0.908777118},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":8.2387313842773438,"field_of_view":45}}' )
// realsense2-util color --fps 30 --width 640   | cv-cat view | cv-cat resize=2             | cv-calc aruco-detect --dict 4X4_50                                    --fields t,pose --max-number-of-markers 1                                    --pinhole pinhole.2.json --anchor 0                                   --marker-length 0.1                                    --binary t,6d                                    --flush    | csv-paste '-;binary=t,6d' value='0,0,0,0,0,0;binary=6d' --flush | points-frame --fields ,frame,x,y,z,roll,pitch,yaw --binary t,12d --to --flush | csv-shuffle --fields t,,,,,,,,,,,,,x,y,z,roll,pitch,yaw --binary t,18d -e --flush | tee 0.bin | math-kalman-filter --measurement-size 3                                  --fields t,measurement --binary t,6d                                  --measurement-noise 0.05                                  --process-noise 0.001 --flush | view-points '-;binary=t,12d;fields=t,,,,roll,pitch,yaw,x,y,z;shape=axes;length=0.1;weight=5;size=1' <(echo 0)';fields=x;shape=axes;length=0.1' <( echo 0 )';fields=x;weight=10;label=0,0,0' --camera-config <( echo '{"center":{"x":0.0802531689,"y":-0.0248078629,"z":0.549671054},"world":{"translation":{"x":-0.263532609,"y":0.406255633,"z":-0.299200088},"rotation":{"x":1.99405885,"y":0.0683527067,"z":0.00327160885}},"camera":{"translation":{"x":0.274289042,"y":-0.428598017,"z":-0.908777118},"rotation":{"x":0,"y":0,"z":0}},"projection":{"up":{"x":0,"y":0,"z":-1},"orthographic":false,"near_plane":0.01,"far_plane":8.2387313842773438,"field_of_view":45}}' )

struct output
{
    boost::posix_time::ptime t;
    std::uint32_t block{0};
    std::uint32_t number_of_markers{0};
    snark::pose pose;
};

} // namespace localization {

} } } // namespace snark { namespace cv_calc { namespace aruco {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_calc::aruco::detection::output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
        v.apply( "pose", p.pose );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
        v.apply( "pose", p.pose );
    }
};

template <> struct traits< snark::cv_calc::aruco::localization::output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::aruco::localization::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "number_of_markers", p.number_of_markers );
        v.apply( "pose", p.pose );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::localization::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "number_of_markers", p.number_of_markers );
        v.apply( "pose", p.pose );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace aruco {

#ifdef SNARK_OPENCV_CONTRIB

namespace detection {

#if CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 5 )

struct dictionaries
{
    #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
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
    // gemini slop
    // static void estimate_poses( const std::vector< std::vector< cv::Point2f > >& corners, float marker_length, const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, std::vector< cv::Vec3d >& rvecs, std::vector< cv::Vec3d >& tvecs )
    // {
    //     float half_length = marker_length / 2.0f;
    //     std::vector< cv::Point3f > obj_points =
    //     {
    //         cv::Point3f( -half_length,  -half_length, 0 ),
    //         cv::Point3f(  half_length,  -half_length, 0 ),
    //         cv::Point3f(  half_length,   half_length, 0 ),
    //         cv::Point3f( -half_length,   half_length, 0 )
    //     };
    //     rvecs.resize( corners.size() );
    //     tvecs.resize( corners.size() );
    //     for( unsigned int i = 0; i < corners.size(); ++i )
    //     {
    //         std::vector< cv::Vec3d > r, t;
    //         cv::solvePnPGeneric( obj_points, corners[i], camera_matrix, distortion_coeffs, r, t, false, cv::SOLVEPNP_IPPE_SQUARE );
    //         if( r.empty() ) { cv::solvePnP( obj_points, corners[i], camera_matrix, distortion_coeffs, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE ); continue; }
    //         if( r.size() == 1 ) { rvecs[i] = r[0]; tvecs[i] = t[0]; continue; }
    //         double a0 = cv::norm( r[0] );
    //         double a1 = cv::norm( r[1] );
    //         //std::cerr << "==> r.size(): " << r.size() << " a0: " << a0 << " (" << ( a0 * 180 / M_PI ) << ") " << " a1: " << a1 << "(" << ( a1 * 180 / M_PI ) << ")" << std::endl; 
    //         rvecs[i] = r[0]; tvecs[i] = t[0];
    //     }
    // }
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
        #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
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
            #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
                cv::aruco::detectMarkers( p.second, dictionary, corners, markers, params, rejected );
            #else
                detector.detectMarkers( p.second, corners, markers, rejected );
            #endif
            if( has_pose ) { estimate_poses( corners, marker_length, camera_matrix, distortion_coeffs, rvecs, tvecs ); }
            for( unsigned i = 0; i < markers.size(); ++i )
            {
                o.t = p.first;
                o.marker = markers[i];
                if( has_corners ) { for( unsigned int j = 0; j < corners[i].size(); ++j ) { o.corners[j] = corners[i][j]; } }
                if( has_pose ) { o.pose = snark::pose( Eigen::Vector3d( tvecs[i][0], tvecs[i][1], tvecs[i][2] ), roll_pitch_yaw::from_rodriques( rvecs[i][0], rvecs[i][1], rvecs[i][2] ) ); }
                ostream.write( o );
            }
            if( flush ) { std::cout.flush(); }
        }
        return 0;
    #endif
}

} // namespace detection {

namespace localization {

class map
{
    public:
        map( unsigned int anchor_id, const std::set< unsigned int >& expected = std::set< unsigned int >{} );

        map( unsigned int anchor_id, unsigned min_number_of_landmarks = 3, const std::set< unsigned int >& expected = std::set< unsigned int >{} );
        
        std::optional< pose > update( const std::vector< std::pair< unsigned int, snark::pose > >& marks );

    private:
        unsigned int _anchor;
        unsigned int _min_number_of_landmarks{3};
        bool _initialised{false};
        std::unordered_map< unsigned int, std::optional< pose > > _anchored;
};

map::map( unsigned int anchor_id, const std::set< unsigned int >& expected )
    : _anchor( anchor_id )
{
    _anchored[_anchor];
    for( unsigned int i: expected ) { _anchored[i]; }
}

map::map( unsigned int anchor_id, unsigned int min_number_of_landmarks, const std::set< unsigned int >& expected )
    : _anchor( anchor_id )
    , _min_number_of_landmarks( min_number_of_landmarks )
{
    _anchored[_anchor];
    for( unsigned int i: expected ) { _anchored[i]; }
}

static Eigen::Quaterniond as_quaternion( const snark::roll_pitch_yaw& rpy ) // quick and dirty for now 
{
    Eigen::AngleAxisd raa( rpy.roll(), Eigen::Vector3d::UnitX() );
    Eigen::AngleAxisd paa( rpy.pitch(), Eigen::Vector3d::UnitY() );
    Eigen::AngleAxisd yaa( rpy.yaw(), Eigen::Vector3d::UnitZ() );
    return yaa * paa * raa;
}

std::optional< snark::pose > map::update( const std::vector< std::pair< unsigned int, snark::pose > >& marks )
{
    if( marks.size() < _min_number_of_landmarks ) { return std::optional< snark::pose >{}; }
    auto anchor = marks.begin();
    for( ; anchor != marks.end() && anchor->first != _anchor; ++anchor );
    if( anchor != marks.end() ) // kinda could do more, but whatever
    {
        _initialised = true;
        _anchored[_anchor] = snark::pose{};
        for( const auto& m: marks ) // lousy for now; todo: interpolation from multple updates
        {
            if( m.first == _anchor ) { continue; } // quick and dirty
            auto& a = _anchored[m.first];
            *a = m.second;
            a->to( anchor->second );
        }
    }
    if( !_initialised ) { return std::optional< snark::pose >{}; }
    if( marks.size() == 1 && marks[0].first == _anchor ) { return snark::pose{}.to( marks[0].second ); }
    snark::pose p{};
    Eigen::Matrix4d qsum = Eigen::Matrix4d::Zero();
    for( const auto& m: marks )
    {
        auto j = _anchored.find( m.first );
        snark::pose pm{};
        pm.to( m.second ).from( *j->second );
        p.translation += pm.translation;
        auto q = as_quaternion( p.rotation ); // todo: quick and dirty, reduce number of forth-and-back conversions
        Eigen::Vector4d v( q.w(), q.x(), q.y(), q.z() );
        qsum += v * v.transpose();
    }
    p.translation /= marks.size();
    Eigen::SelfAdjointEigenSolver< Eigen::Matrix4d > solver( qsum );
    Eigen::Vector4d mean = solver.eigenvectors().col( 3 ); // eigenvectors are sorted in ascending order
    p.rotation = snark::rotation_matrix( Eigen::Quaterniond( mean( 0 ), mean( 1 ), mean( 2 ), mean( 3 ) ).normalized() ).roll_pitch_yaw();  // todo: quick and dirty, reduce number of forth-and-back conversions
    return p;
}

// static Eigen::Matrix3d swap()
// {
//     Eigen::Matrix3d m;
//     m << 0, 0, 1
//        , 1, 0, 0
//        , 0, 1, 0;
//     return m;
// }

// static snark::roll_pitch_yaw to_frd( const snark::roll_pitch_yaw& a ) // quick and dirty, watch performance
// {
//     static Eigen::Matrix3d s = swap();
//     static Eigen::Matrix3d t = s.transpose();
//     return snark::rotation_matrix::roll_pitch_yaw( s * snark::rotation_matrix::rotation( a ).transpose() * t );
// }

static Eigen::Matrix3d swap()
{
    Eigen::Matrix3d m;
    m << 0, 0, 1
       , 1, 0, 0
       , 0, 1, 0;
    return m;
}

static snark::roll_pitch_yaw to_frd( const snark::roll_pitch_yaw& a ) // quick and dirty, watch performance
{
    static Eigen::Matrix3d t = swap().transpose();
    return snark::rotation_matrix::roll_pitch_yaw( t * snark::rotation_matrix::rotation( snark::roll_pitch_yaw( a.yaw(), a.roll(), a.pitch() ) ) );
}

// static snark::roll_pitch_yaw to_frd( const snark::roll_pitch_yaw& a ) // quick and dirty, watch performance
// {
//     return snark::roll_pitch_yaw( a.yaw(), a.pitch(), a.roll() );
// }

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options )
{
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 5 )
        COMMA_THROW_BRIEF( comma::exception, "cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" );
    #else
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output >(), ',' ) << std::endl; return 0; };
        if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format( comma::csv::format::value< output >() ).collapsed_string() << std::endl; return 0; }
        snark::cv_mat::serialization input( input_options );
        comma::csv::options csv( options );
        comma::csv::output_stream< output > ostream( std::cout, csv );
        bool flush = options.exists( "--flush" );
        #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary( detection::dictionaries::type_from_string( options.value< std::string >( "--dictionary,--dict" ) ) );
            cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        #else
            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary( detection::dictionaries::type_from_string( options.value< std::string >( "--dictionary,--dict" ) ) );
            cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
            cv::aruco::ArucoDetector detector( dictionary, params );
        #endif
        double marker_length = options.value< double >( "--marker-length" );
        localization::map map( options.value< unsigned int >( "--marker-anchor-id,--anchor" ), options.value( "--markers-min-number,--min-number-of-markers", 1 ) );
        std::string reference_frame = options.value< std::string >( "--orientation-frame,--frame", "camera" );
        COMMA_ASSERT_BRIEF( reference_frame == "camera" || reference_frame == "frd", "expected --orientation-frame 'camera' or 'frd'; got: --orientation-frame='" << reference_frame << "'" );
        bool use_frd = reference_frame == "frd";
        cv::Mat camera_matrix{}, distortion_coeffs{};
        auto s = options.value< std::string >( "--pinhole-config,--pinhole" );
        const auto& v = comma::split( options.value< std::string >( "--pinhole-config,--pinhole" ), ':', true );
        COMMA_ASSERT_BRIEF( v.size() == 1 || v.size() == 2, "expected --pinhole-config=<file>[:<path>]; got: '" << s << "'" );
        const auto& config = comma::read_json< snark::camera::pinhole::config_t >( v[0], v.size() == 2 ? v[1] : "" );
        camera_matrix = config.camera_matrix();
        if( config.distortion ) { distortion_coeffs = config.distortion->as< cv::Mat >(); }
        output o;
        std::vector< std::vector< cv::Point2f > > corners;
        std::vector< int > markers;
        std::vector< std::vector< cv::Point2f > > rejected;
        std::vector< cv::Vec3d > rvecs, tvecs;
        std::vector< std::pair< unsigned int, snark::pose > > poses;
        for( ; std::cin.good() && !std::cin.eof(); ++o.block )
        {
            std::pair< boost::posix_time::ptime, cv::Mat > i = input.read< boost::posix_time::ptime >( std::cin );
            if( i.second.empty() ) { return 0; }
            #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
                cv::aruco::detectMarkers( i.second, dictionary, corners, markers, params, rejected );
            #else
                detector.detectMarkers( i.second, corners, markers, rejected );
            #endif
            detection::estimate_poses( corners, marker_length, camera_matrix, distortion_coeffs, rvecs, tvecs );
            poses.resize( markers.size() );
            for( unsigned i = 0; i < markers.size(); ++i ) { poses[i] = std::make_pair( markers[i], snark::pose( Eigen::Vector3d( tvecs[i][0], tvecs[i][1], tvecs[i][2] ), roll_pitch_yaw::from_rodriques( rvecs[i][0], rvecs[i][1], rvecs[i][2] ) ) ); }
            const auto& p = map.update( poses );
            if( !p ) { continue; }
            o.t = i.first;
            o.number_of_markers = markers.size();
            o.pose = use_frd ? snark::pose( p->translation, to_frd( p->rotation ) ) : *p;
            ostream.write( o );
            if( flush ) { std::cout.flush(); }
        }
        return 0;
    #endif
}

} // namespace localization {

#else // #ifdef SNARK_OPENCV_CONTRIB

namespace detection {

int run( const comma::command_line_options&, const snark::cv_mat::serialization::options& )
{
    COMMA_THROW_BRIEF( comma::exception,    "aruco-detect: built with cmake flag"
                            << std::endl << "    snark_build_imaging_opencv_contrib=OFF, which does"
                            << std::endl << "    not support aruco detection; please rebuild with"
                            << std::endl << "    snark_build_imaging_opencv_contrib=ON" );
}

} // namespace detection {

namespace localization {

int run( const comma::command_line_options&, const snark::cv_mat::serialization::options& )
{
    COMMA_THROW_BRIEF( comma::exception,    "aruco-localize: built with cmake flag"
                            << std::endl << "    snark_build_imaging_opencv_contrib=OFF, which does"
                            << std::endl << "    not support aruco localization; please rebuild with"
                            << std::endl << "    snark_build_imaging_opencv_contrib=ON" );
}

} // namespace localization {

#endif // #ifdef SNARK_OPENCV_CONTRIB

} } } // namespace snark { namespace cv_calc { namespace aruco {
