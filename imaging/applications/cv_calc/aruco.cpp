// Copyright (c) 2026 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cinttypes>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2 // i hate opencv so much...
    #define ARUCO_SUPPORTED false
    #define NEEDS_CONTRIB false
#elif CV_MAJOR_VERSION < 4
    #define ARUCO_SUPPORTED false
    #define NEEDS_CONTRIB false
#else
    #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
        #if CV_MINOR_VERSION <= 5
            #define ARUCO_SUPPORTED false
            #define NEEDS_CONTRIB false
        #else
            #ifdef SNARK_OPENCV_CONTRIB
                #define ARUCO_SUPPORTED true
                #define NEEDS_CONTRIB false
                #include <opencv2/aruco.hpp>
            #else
                #define ARUCO_SUPPORTED true
                #define NEEDS_CONTRIB true
            #endif
        #endif
    #else
        #define ARUCO_SUPPORTED true
        #define NEEDS_CONTRIB false
        #include <opencv2/objdetect/aruco_detector.hpp>
    #endif
#endif
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
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
    #if ARUCO_SUPPORTED
        #if NEEDS_CONTRIB
            oss << "    cv-calc built with cmake flag snark_build_imaging_opencv_contrib=OFF; set snark_build_imaging_opencv_contrib=ON and rebuild" << std::endl;
        #endif
    #else
        oss << "    cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    oss << R"(        --dictionary,--dict=<dictionary>
        --marker-length=[<marker_length>]; length of marker side
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
    #if ARUCO_SUPPORTED
        #if NEEDS_CONTRIB
            oss << "        cv-calc built with cmake flag snark_build_imaging_opencv_contrib=OFF; set snark_build_imaging_opencv_contrib=ON and rebuild" << std::endl;
        #endif
    #else
        oss << "        cv-calc built with opencv " << CV_VERSION << ", which does not support aruco detection" << std::endl;
    #endif
    oss << R"(        --dictionary,--dict=<dictionary>
        --marker=<id>[,<length>[,<x>,<y>,<z>[,<roll>,<pitch>,<yaw>]]]
            ATTENTION: marker pose is in ned/frd, i.e. if marker lies on the floor
                       its x axis is considered pointing from the bottom of the
                       marker to its top; its y axis points to the right; and
                       its z axis points down
        --marker-length=[<marker_length>]; length of marker side, also, see --marker
        --markers=[<csv_file>]; csv file, fields: id,length,x,y,z,roll,pitch,yaw
        --markers-ignore-unknown,--ignore-unknown; ignore unregistered markers
        --markers-min-number,--min-number-of-markers=<n>; default=1; min number of markers
                visible in a single frame (only 1 supported for now)
        --pinhole-config,--pinhole=<config>; <config>: <filename>[:<path>]
        --reference-frame,--frame=<which>; default=camera
            <which>
                camera: right-down-forward, i.e. camera looking along z axis
                frd   : forward-right-down, i.e. camera looking along x axis
                raw   : as comes from aruco detection (z axis along marker z axis)
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

struct marker
{
    std::uint32_t id{0};
    double length{0.};
    snark::pose pose;
};

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

template <> struct traits< snark::cv_calc::aruco::localization::marker >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::aruco::localization::marker& p, Visitor& v )
    {
        v.apply( "id", p.id );
        v.apply( "length", p.length );
        v.apply( "pose", p.pose );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::localization::marker& p, Visitor& v )
    {
        v.apply( "id", p.id );
        v.apply( "length", p.length );
        v.apply( "pose", p.pose );
    }
};

} } // namespace comma { namespace visiting {

#if ARUCO_SUPPORTED && !NEEDS_CONTRIB

namespace snark { namespace cv_calc { namespace aruco {

namespace detection {

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

#if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7 ) // how painful...
    static void estimate_poses( const std::vector< std::vector< cv::Point2f > >& corners, float marker_length, const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, std::vector< cv::Vec3d >& rvecs, std::vector< cv::Vec3d >& tvecs )
    { 	
        cv::aruco::estimatePoseSingleMarkers( corners, marker_length, camera_matrix, distortion_coeffs, rvecs, tvecs );
    }
#else
    static void estimate_poses( const std::vector< std::vector< cv::Point2f > >& corners, float marker_length, const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, std::vector< cv::Vec3d >& rvecs, std::vector< cv::Vec3d >& tvecs )
    {
        float half_length = marker_length / 2.0f;
        std::vector< cv::Point3f > obj_points = {
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
    #if CV_MAJOR_VERSION < 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 5 )
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

namespace todo {

class map
{
    public:
        map( unsigned int anchor_id, const std::set< unsigned int >& expected = std::set< unsigned int >{} );

        map( unsigned int anchor_id, unsigned min_number_of_landmarks = 3, const std::set< unsigned int >& expected = std::set< unsigned int >{} );
        
        std::optional< pose > update( const std::vector< std::pair< unsigned int, snark::pose > >& marks, bool raw, bool frd );

    private:
        unsigned int _anchor;
        unsigned int _min_number_of_landmarks{1};
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
    Eigen::AngleAxisd r( rpy.roll(), Eigen::Vector3d::UnitX() );
    Eigen::AngleAxisd p( rpy.pitch(), Eigen::Vector3d::UnitY() );
    Eigen::AngleAxisd y( rpy.yaw(), Eigen::Vector3d::UnitZ() );
    return y * p * r;
}

std::optional< snark::pose > map::update( const std::vector< std::pair< unsigned int, snark::pose > >& marks, bool raw, bool frd )
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
    static const snark::pose marker_offset( Eigen::Vector3d( 0, 0, 0 ), snark::roll_pitch_yaw( M_PI, 0, M_PI / 2 ) );
    static const snark::pose camera_offset( Eigen::Vector3d( 0, 0, 0 ), snark::roll_pitch_yaw( M_PI / 2, 0, M_PI / 2 ) );
    if( marks.size() == 1 && marks[0].first == _anchor )
    {
        snark::pose p{};
        if( frd ) { p.to( camera_offset ); }
        p.to( marks[0].second );
        if( !raw ) { p.from( marker_offset ); }
        return p;
    }
    // todo! solver! return in correct reference frame!
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

} // namespace todo {

class map
{
    public:
        map( unsigned int min_number_of_landmarks = 1, bool do_update = false ): _min_number_of_landmarks( min_number_of_landmarks ), _do_update( do_update ) { COMMA_ASSERT_BRIEF( !_do_update, "dynamic update: todo" ); }
        
        void insert( const std::pair< unsigned int, localization::marker >& p ) { _landmarks[p.first] = p.second; }

        std::optional< pose > update( const std::vector< std::pair< unsigned int, snark::pose > >& marks, bool raw, bool frd );

        const std::unordered_map< unsigned int, localization::marker >& landmarks() const { return _landmarks; }

    private:
        unsigned int _min_number_of_landmarks{1};
        bool _do_update{false};
        std::unordered_map< unsigned int, localization::marker > _landmarks;
};

std::optional< pose > map::update( const std::vector< std::pair< unsigned int, snark::pose > >& marks, bool raw, bool frd )
{
    if( _do_update )
    {
        // todo
    }
    if( marks.size() < _min_number_of_landmarks ) { return {}; }
    auto j = _landmarks.end();
    snark::pose q;
    for( const auto& m: marks ) // todo: quick and dirty for now; solve on multiple landmarks
    {
        auto i = _landmarks.find( m.first );
        if( i != _landmarks.end() && ( j == _landmarks.end() || j->second.length < i->second.length ) ) { j = i; q = m.second; }
    }
    if( j == _landmarks.end() ) { return {}; }
    static const snark::pose marker_offset( Eigen::Vector3d( 0, 0, 0 ), snark::roll_pitch_yaw( M_PI, 0, M_PI / 2 ) );
    static const snark::pose camera_offset( Eigen::Vector3d( 0, 0, 0 ), snark::roll_pitch_yaw( M_PI / 2, 0, M_PI / 2 ) );
    snark::pose p{};
    if( frd ) { p.to( camera_offset ); }
    p.to( q );
    return ( raw ? p : p.from( marker_offset ) ).from( j->second.pose );
}

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
        auto marker_length = options.optional< double >( "--marker-length" );
        localization::map map( options.value( "--markers-min-number,--min-number-of-markers", 1 ) );
        comma::csv::ascii< localization::marker > ascii;
        std::vector< double > lengths;
        bool ignore_unknown_markers = options.exists( "--markers-ignore-unknown,--ignore-unknown" );
        auto init_marker = [&]( const localization::marker& n )
        {
            COMMA_ASSERT_BRIEF( n.length > 1e-6 || marker_length, "aruco-localize: marker with id: " << n.id << ": no length specified; please specify marker length least or --marker-length" );
            auto m = n;
            if( m.length <= 1e-6 ) { m.length = *marker_length; }
            map.insert( { m.id, m } );
            bool found = false;
            for( double length: lengths ) { if( comma::math::equal( m.length, length ) ) { found = true; break; } }
            if( !found ) { lengths.push_back( m.length ); }
        };
        for( const auto& s: options.values< std::string >( "--marker" ) ) { init_marker( ascii.get( s, true ) ); }
        std::string markers_filename = options.value< std::string >( "--markers", "" );
        if( !markers_filename.empty() ) { for( const auto& m: comma::csv::read_as< std::vector< localization::marker > >( markers_filename ) ) { init_marker( m ); } }
        std::string reference_frame = options.value< std::string >( "--reference-frame,--frame", "camera" );
        COMMA_ASSERT_BRIEF( reference_frame == "camera" || reference_frame == "frd" || reference_frame == "raw", "aruco-localize: expected --reference-frame 'raw', 'camera', or 'frd'; got: --reference-frame='" << reference_frame << "'" );
        bool frd = reference_frame == "frd";
        bool raw = reference_frame == "raw";
        cv::Mat camera_matrix{}, distortion_coeffs{};
        auto s = options.value< std::string >( "--pinhole-config,--pinhole" );
        const auto& v = comma::split( options.value< std::string >( "--pinhole-config,--pinhole" ), ':', true );
        COMMA_ASSERT_BRIEF( v.size() == 1 || v.size() == 2, "expected --pinhole-config=<file>[:<path>]; got: '" << s << "'" );
        const auto& config = comma::read_json< snark::camera::pinhole::config_t >( v[0], v.size() == 2 ? v[1] : "" );
        camera_matrix = config.camera_matrix();
        if( config.distortion ) { distortion_coeffs = config.distortion->as< cv::Mat >(); }
        output o;
        std::vector< std::vector< cv::Point2f > > corners;
        std::vector< int > ids;
        std::vector< std::vector< cv::Point2f > > rejected;
        std::vector< cv::Vec3d > rvecs, tvecs;
        std::vector< std::pair< unsigned int, snark::pose > > poses;
        for( ; std::cin.good() && !std::cin.eof(); ++o.block )
        {
            std::pair< boost::posix_time::ptime, cv::Mat > i = input.read< boost::posix_time::ptime >( std::cin );
            if( i.second.empty() ) { return 0; }
            #if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7
                cv::aruco::detectMarkers( i.second, dictionary, corners, ids, params, rejected );
            #else
                detector.detectMarkers( i.second, corners, ids, rejected );
            #endif
            if( ids.empty() ) { continue; }
            poses.clear();
            poses.reserve( ids.size() );
            if( lengths.size() == 1 ) // quick and dirty, a minor optimisation
            {
                detection::estimate_poses( corners, lengths[0], camera_matrix, distortion_coeffs, rvecs, tvecs );
                for( unsigned i = 0; i < ids.size(); ++i ) { poses.emplace_back( std::make_pair( ids[i], snark::pose( Eigen::Vector3d( tvecs[i][0], tvecs[i][1], tvecs[i][2] ), roll_pitch_yaw::from_rodriques( rvecs[i][0], rvecs[i][1], rvecs[i][2] ) ) ) ); }
            }
            else
            {
                for( double length: lengths )
                {
                    std::vector< int > i;
                    std::vector< std::vector< cv::Point2f > > c;
                    for( unsigned j = 0; j < ids.size(); ++j )
                    {
                        auto m = map.landmarks().find( ids[j] );
                        if( m == map.landmarks().end() )
                        {
                            if( ignore_unknown_markers ) { continue; }
                            COMMA_ASSERT_BRIEF( marker_length, "got unregistered marker with id: " << ids[j] << "; please specify --marker-length" );
                            if( !comma::math::equal( *marker_length, length ) ) { continue; }
                        }
                        if( !comma::math::equal( m->second.length, length ) ) { continue; }
                        i.push_back( ids[j] );
                        c.push_back( corners[j] );
                    }
                    if( i.empty() ) { continue; }
                    std::vector< cv::Vec3d > r, t;
                    detection::estimate_poses( c, length, camera_matrix, distortion_coeffs, r, t );
                    for( unsigned j = 0; j < i.size(); ++j ) { poses.emplace_back( std::make_pair( i[j], snark::pose( Eigen::Vector3d( t[j][0], t[j][1], t[j][2] ), roll_pitch_yaw::from_rodriques( r[j][0], r[j][1], r[j][2] ) ) ) ); }
                }
            }
            const auto& p = map.update( poses, raw, frd );
            if( !p ) { continue; }
            o.t = i.first;
            o.number_of_markers = poses.size();
            o.pose = *p;
            ostream.write( o );
            if( flush ) { std::cout.flush(); }
        }
        return 0;
    #endif
}

} // namespace localization {

} } } // namespace snark { namespace cv_calc { namespace aruco {

#else // #if ARUCO_SUPPORTED && !NEEDS_CONTRIB
    #if ARUCO_SUPPORTED
        namespace snark { namespace cv_calc { namespace aruco {

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

        } } } // namespace snark { namespace cv_calc { namespace aruco {
    #else
        namespace snark { namespace cv_calc { namespace aruco {

        namespace detection {

        int run( const comma::command_line_options&, const snark::cv_mat::serialization::options& )
        {
            COMMA_THROW_BRIEF( comma::exception,    "aruco-detect: not supported in opencv version " << CV_VERSION );
        }

        } // namespace detection {

        namespace localization {

        int run( const comma::command_line_options&, const snark::cv_mat::serialization::options& )
        {
            COMMA_THROW_BRIEF( comma::exception,    "aruco-localize: not supported in opencv version " << CV_VERSION );
        }

        } // namespace localization {

        } } } // namespace snark { namespace cv_calc { namespace aruco {
    #endif
#endif // #if ARUCO_SUPPORTED && !NEEDS_CONTRIB

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// using Pose2D = Eigen::Transform<double, 2, Eigen::Isometry>;
// using Vector3d = Eigen::Vector3d;

// struct PlanarMeasurement {
//     int vehicle_idx;
//     int landmark_idx;
//     Pose2D T_v_l; // 2D projected measurement of landmark relative to vehicle
// };

// // Normalises an angle to the range [-pi, pi]
// double normalizeAngle(double theta) {
//     return std::atan2(std::sin(theta), std::cos(theta));
// }

// class PlanarMapOptimizer {
// public:
//     int num_vehicles;
//     int num_landmarks;
    
//     std::vector<Pose2D> T_0_v; // Vehicle 2D poses relative to Marker 0
//     std::vector<Pose2D> T_0_l; // Marker 2D poses relative to Marker 0
//     std::vector<PlanarMeasurement> measurements;

//     PlanarMapOptimizer(int nv, int nl) : num_vehicles(nv), num_landmarks(nl) {
//         T_0_v.resize(nv, Pose2D::Identity());
//         T_0_l.resize(nl, Pose2D::Identity());
//     }

//     void optimize(int max_iterations = 15) {
//         // State dimension: 3 variables (x, y, yaw) per vehicle + (landmarks - 1)
//         // Marker 0 is perfectly locked at (0, 0, 0) to resolve gauge freedom
//         int state_dim = (num_vehicles + (num_landmarks - 1)) * 3;

//         for (int iter = 0; iter < max_iterations; ++iter) {
//             Eigen::MatrixXd H = Eigen::MatrixXd::Zero(state_dim, state_dim);
//             Eigen::VectorXd g = Eigen::VectorXd::Zero(state_dim);
//             double total_chi2 = 0.0;

//             for (const auto& meas : measurements) {
//                 int v_idx = meas.vehicle_idx;
//                 int l_idx = meas.landmark_idx;

//                 Pose2D T_0_v_curr = T_0_v[v_idx];
//                 Pose2D T_0_l_curr = T_0_l[l_idx];

//                 // Predicted measurement: T_v_l = T_0_v^-1 * T_0_l
//                 Pose2D T_v_l_pred = T_0_v_curr.inverse() * T_0_l_curr;

//                 // Difference matrix
//                 Pose2D Error_T = meas.T_v_l.inverse() * T_v_l_pred;
                
//                 // Extract 3DoF vector error
//                 Vector3d residual;
//                 residual(0) = Error_T.translation().x();
//                 residual(1) = Error_T.translation().y();
//                 // Extract angle from the 2D rotation matrix component
//                 residual(2) = normalizeAngle(std::atan2(Error_T.linear()(1,0), Error_T.linear()(0,0)));

//                 total_chi2 += residual.squaredNorm();

//                 // Compute explicit analytical Jacobians for 2D rigid transformations
//                 Eigen::Matrix3d J_v = Eigen::Matrix3d::Zero();
//                 Eigen::Matrix3d J_l = Eigen::Matrix3d::Zero();

//                 double theta_v = std::atan2(T_0_v_curr.linear()(1,0), T_0_v_curr.linear()(0,0));
//                 double cos_v = std::cos(theta_v);
//                 double sin_v = std::sin(theta_v);

//                 // Jacobian with respect to vehicle state updates [dx, dy, dtheta]
//                 J_v.block<2,2>(0,0) = -T_0_v_curr.linear().transpose();
//                 double tx = T_0_l_curr.translation().x() - T_0_v_curr.translation().x();
//                 double ty = T_0_l_curr.translation().y() - T_0_v_curr.translation().y();
//                 J_v(0, 2) = -sin_v * tx + cos_v * ty;
//                 J_v(1, 2) = -cos_v * tx - sin_v * ty;
//                 J_v(2, 2) = -1.0;

//                 // Jacobian with respect to landmark state updates (skipped for fixed Marker 0)
//                 if (l_idx > 0) {
//                     J_l.block<2,2>(0,0) = T_0_v_curr.linear().transpose();
//                     J_l(2, 2) = 1.0;
//                 }

//                 // Map blocks to global Hessian system matrix
//                 int v_block = v_idx * 3;
//                 int l_block = (num_vehicles * 3) + ((l_idx - 1) * 3);

//                 H.block<3, 3>(v_block, v_block) += J_v.transpose() * J_v;
//                 g.segment<3>(v_block) -= J_v.transpose() * residual;

//                 if (l_idx > 0) {
//                     H.block<3, 3>(l_block, l_block) += J_l.transpose() * J_l;
//                     g.segment<3>(l_block) -= J_l.transpose() * residual;

//                     // Inter-pose cross correlation constraints
//                     H.block<3, 3>(v_block, l_block) += J_v.transpose() * J_l;
//                     H.block<3, 3>(l_block, v_block) += J_l.transpose() * J_v;
//                 }
//             }

//             std::cout (j * 3);
//                 T_0_v[j] = T_0_v[j] * createPose2D(upd(0), upd(1), upd(2));
//             }
//             for (int i = 1; i < num_landmarks; ++i) {
//                 int l_block = (num_vehicles * 3) + ((i - 1) * 3);
//                 Vector3d upd = delta.segment<3>(l_block);
//                 T_0_l[i] = T_0_l[i] * createPose2D(upd(0), upd(1), upd(2));
//             }
//         }
//     }
// };

// #include <iostream>
// #include <vector>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// using Pose3D = Eigen::Isometry3d;
// using Vector6d = Eigen::Matrix<double, 6, 1>;

// struct Measurement {
//     int vehicle_idx;    // Index of the vehicle position (j)
//     int landmark_idx;   // Index of the landmark observed (i)
//     Pose3D T_v_l;       // Landmark pose measured relative to the vehicle
// };

// // Converts a 6D twist vector (w, v) into a 4x4 perturbation matrix
// Pose3D expMap(const Vector6d& twist) {
//     Eigen::Vector3d w = twist.head<3>();
//     Eigen::Vector3d v = twist.tail<3>();
    
//     // Angle-axis for rotation
//     double angle = w.norm();
//     Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
//     if (angle > 1e-6) {
//         R = Eigen::AngleAxisd(angle, w.normalized()).toRotationMatrix();
//     }
    
//     Pose3D T = Pose3D::Identity();
//     T.linear() = R;
//     T.translation() = v; // Using standard first-order approximation for translation
//     return T;
// }

// class BatchMapOptimizer {
// public:
//     int num_vehicles;
//     int num_landmarks;
    
//     std::vector<Pose3D> T_0_v; // Vehicle poses relative to Landmark 0
//     std::vector<Pose3D> T_0_l; // Landmark poses relative to Landmark 0
//     std::vector<Measurement> measurements;

//     BatchMapOptimizer(int nv, int nl) : num_vehicles(nv), num_landmarks(nl) {
//         T_0_v.resize(nv, Pose3D::Identity());
//         T_0_l.resize(nl, Pose3D::Identity());
//     }

//     // Call this before optimizing! Ensure your log initializes poses 
//     // transitively so they aren't all just identity matrices.
//     void initializeMapFromLogs() {
//         T_0_l[0] = Pose3D::Identity(); // Landmark 0 is the global anchor
//         // TODO: Users should seed T_0_v and T_0_l using direct chain calculations 
//         // e.g., T_0_v[j] = T_0_l[0] * T_v_l_measurement.inverse()
//     }

//     void optimize(int max_iterations = 10) {
//         // State vector size: (num_vehicles + num_landmarks - 1) * 6
//         // We subtract 1 because Landmark 0 is strictly fixed at Identity (gauge freedom)
//         int num_poses_to_optimize = num_vehicles + (num_landmarks - 1);
//         int state_dim = num_poses_to_optimize * 6;

//         for (int iter = 0; iter < max_iterations; ++iter) {
//             Eigen::MatrixXd H = Eigen::MatrixXd::Zero(state_dim, state_dim);
//             Eigen::VectorXd g = Eigen::VectorXd::Zero(state_dim);
//             double total_error = 0.0;

//             for (const auto& meas : measurements) {
//                 int v_idx = meas.vehicle_idx;
//                 int l_idx = meas.landmark_idx;

//                 // Current estimates
//                 Pose3D T_0_v_curr = T_0_v[v_idx];
//                 Pose3D T_0_l_curr = T_0_l[l_idx];

//                 // Predicted measurement: T_v_l = T_0_v^-1 * T_0_l
//                 Pose3D T_v_l_pred = T_0_v_curr.inverse() * T_0_l_curr;

//                 // Compute Residual Matrix (Error in measurement space)
//                 // Residual = Log( Meas^-1 * Pred )
//                 Pose3D Error_T = meas.T_v_l.inverse() * T_v_l_pred;
                
//                 // Extract 6D vector residual (3 rotation elements, 3 translation elements)
//                 Eigen::AngleAxisd aa(Error_T.linear());
//                 Vector6d residual;
//                 residual.head<3>() = aa.angle() * aa.axis();
//                 residual.tail<3>() = Error_T.translation();

//                 total_error += residual.squaredNorm();

//                 // State mapping helpers
//                 int v_state_block = v_idx * 6;
//                 // Landmark 0 isn't optimized, so scale back indices by 1
//                 int l_state_block = (num_vehicles * 6) + ((l_idx - 1) * 6); 

//                 // Compute Numerical Jacobians for this measurement
//                 Eigen::Matrix<double, 6, 6> J_v = Eigen::Matrix<double, 6, 6>::Zero();
//                 Eigen::Matrix<double, 6, 6> J_l = Eigen::Matrix<double, 6, 6>::Zero();
//                 double eps = 1e-6;

//                 for (int k = 0; k < 6; ++k) {
//                     Vector6d perturbation = Vector6d::Zero();
//                     perturbation(k) = eps;
//                     Pose3D delta_T = expMap(perturbation);

//                     // Perturb vehicle pose
//                     if (true) {
//                         Pose3D T_v_perturbed = T_0_v_curr * delta_T;
//                         Pose3D E_perturbed = meas.T_v_l.inverse() * (T_v_perturbed.inverse() * T_0_l_curr);
//                         Eigen::AngleAxisd aa_p(E_perturbed.linear());
//                         Vector6d res_p;
//                         res_p.head<3>() = aa_p.angle() * aa_p.axis();
//                         res_p.tail<3>() = E_perturbed.translation();
//                         J_v.col(k) = (res_p - residual) / eps;
//                     }

//                     // Perturb landmark pose (skip if it is Landmark 0)
//                     if (l_idx > 0) {
//                         Pose3D T_l_perturbed = T_0_l_curr * delta_T;
//                         Pose3D E_perturbed = meas.T_v_l.inverse() * (T_0_v_curr.inverse() * T_l_perturbed);
//                         Eigen::AngleAxisd aa_p(E_perturbed.linear());
//                         Vector6d res_p;
//                         res_p.head<3>() = aa_p.angle() * aa_p.axis();
//                         res_p.tail<3>() = E_perturbed.translation();
//                         J_l.col(k) = (res_p - residual) / eps;
//                     }
//                 }

//                 // Populate global Hessian (H) and Gradient vector (g)
//                 // Contribution from Vehicle Pose
//                 H.block<6, 6>(v_state_block, v_state_block) += J_v.transpose() * J_v;
//                 g.segment<6>(v_state_block) -= J_v.transpose() * residual;

//                 // Contribution from Landmark Pose
//                 if (l_idx > 0) {
//                     H.block<6, 6>(l_state_block, l_state_block) += J_l.transpose() * J_l;
//                     g.segment<6>(l_state_block) -= J_l.transpose() * residual;

//                     // Cross terms between vehicle and landmark
//                     H.block<6, 6>(v_state_block, l_state_block) += J_v.transpose() * J_l;
//                     H.block<6, 6>(l_state_block, v_state_block) += J_l.transpose() * J_v;
//                 }
//             }

//             std::cout (j * 6));
//             }
//             for (int i = 1; i < num_landmarks; ++i) {
//                 int l_state_block = (num_vehicles * 6) + ((i - 1) * 6);
//                 T_0_l[i] = T_0_l[i] * expMap(delta.segment<6>(l_state_block));
//             }
            
//             if (delta.norm() < 1e-5) {
//                 std::cout << "Converged early at iteration " << iter << "!\n";
//                 break;
//             }
//         }
//     }
// };
