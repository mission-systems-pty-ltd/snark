// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <comma/base/exception.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include "../../math/rotation_matrix.h"
#include "stereo.h"

namespace snark { namespace camera { namespace stereo {

pair::pair( const config_t& config ): _cameras( config.first, config.second ) {}

pair::pair( const pinhole::config_t& pinhole, double baseline ) : pair( pinhole, pinhole, baseline ) {}

pair::pair( const pinhole::config_t& first, const pinhole::config_t& second, double baseline )
    : _cameras( camera_t( first, snark::pose( Eigen::Vector3d( baseline / 2, 0, 0 ), snark::roll_pitch_yaw( 0, 0, 0 ) ) )
              , camera_t( first, snark::pose( Eigen::Vector3d( -baseline / 2, 0, 0 ), snark::roll_pitch_yaw( 0, 0, 0 ) ) ) )
{
}

static ::Eigen::Affine3d affine_( const snark::pose& pose )
{
    ::Eigen::Translation3d translation;
    translation.vector() = pose.translation;
    ::Eigen::Affine3d affine = translation * snark::rotation_matrix::rotation( pose.rotation );
    return affine;
}

static ::Eigen::Affine3d ned_affine_ = affine_( snark::pose( Eigen::Vector3d::Zero(), snark::roll_pitch_yaw( M_PI / 2, 0, M_PI / 2 ) ) );

std::pair< Eigen::Vector3d, Eigen::Vector3d > pair::to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second ) const { return to_cartesian( first, second, _cameras.first.pose, _cameras.second.pose ); }

std::pair< Eigen::Vector3d, Eigen::Vector3d > pair::to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& pose ) const  { return to_cartesian( first, second, snark::pose(), pose ); }

std::pair< Eigen::Vector3d, Eigen::Vector3d > pair::to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& first_pose, const snark::pose& second_pose ) const
{
    const auto& first_affine = affine_( first_pose ); // todo! precalc affine! currently, performance sucks
    const Eigen::Vector3d& fp = first_affine * ( ned_affine_ * _cameras.first.pinhole.to_cartesian( first ) );
    const Eigen::Vector3d& fc = first_affine * Eigen::Vector3d::Zero();
    const auto& second_affine = affine_( second_pose ); // todo! precalc affine! currently, performance sucks
    const Eigen::Vector3d& sp = second_affine * ( ned_affine_ * _cameras.second.pinhole.to_cartesian( second ) );
    const Eigen::Vector3d& sc = second_affine * Eigen::Vector3d::Zero();
    const Eigen::Vector3d& f = ( fp - fc ).normalized();
    const Eigen::Vector3d& s = ( sp - sc ).normalized();
    if( comma::math::equal( f.dot( s ), f.norm() * s.norm() ) ) { COMMA_THROW( comma::exception, "got collinear projection vectors on pixels: " << first.transpose() << " and " << second.transpose() ); }
    const Eigen::Vector3d& m = s.cross( f ).normalized();
    const Eigen::Vector3d& n = s.cross( m ).normalized();
    const Eigen::Vector3d& d = sp - fp;
    const Eigen::Vector3d& a = fp + f * n.dot( d ) / n.dot( f );
    const Eigen::Vector3d& b = a + m * m.dot( d );

//     std::cerr << "========================================================================================" << std::endl;
//     std::cerr << "                                                    first: " << first.transpose() << std::endl;
//     std::cerr << "                                               first pose: " << first_pose.translation.transpose() << " " << first_pose.rotation.roll() << " " << first_pose.rotation.pitch() << " " << first_pose.rotation.yaw() << std::endl;
//     std::cerr << "                     _cameras.first.pinhole.to_cartesian( first ): " << _cameras.first.pinhole.to_cartesian( first ).transpose() << std::endl;
//     std::cerr << "       ned_affine_ * _cameras.first.pinhole.to_cartesian( first ): " << ( ned_affine_ * _cameras.first.pinhole.to_cartesian( first ) ).transpose() << std::endl;
//     std::cerr << "                                                       fp: " << fp.transpose() << std::endl;
//     std::cerr << "                                                       fc: " << fc.transpose() << std::endl;
//     std::cerr << "                                                        a: " << a.transpose() << std::endl;
//     std::cerr << "========================================================================================" << std::endl;
//     std::cerr << "                                                   second: " << second.transpose() << std::endl;
//     std::cerr << "                                              second pose: " << second_pose.translation.transpose() << " " << second_pose.rotation.roll() << " " << second_pose.rotation.pitch() << " " << second_pose.rotation.yaw() << std::endl;
//     std::cerr << "                   _cameras.second.pinhole.to_cartesian( second ): " << _cameras.second.pinhole.to_cartesian( second ).transpose() << std::endl;
//     std::cerr << "     ned_affine_ * _cameras.second.pinhole.to_cartesian( second ): " << ( ned_affine_ * _cameras.second.pinhole.to_cartesian( second ) ).transpose() << std::endl;
//     std::cerr << "                                                       sp: " << sp.transpose() << std::endl;
//     std::cerr << "                                                       sc: " << sc.transpose() << std::endl;
//     std::cerr << "                                                        b: " << b.transpose() << std::endl;
//     std::cerr << std::endl;

    return std::make_pair( a, b );
}

std::pair< std::vector< cv::Mat >, std::vector< cv::Mat > > pair::rectify_map( unsigned int width, unsigned int height, int type ) const
{
    if( type == CV_32FC2 ) { COMMA_THROW( comma::exception, "type: CV_32FC2 (2f): todo" ); }
    const auto& translation = _cameras.second.pose.translation - _cameras.first.pose.translation;
    const auto& rotation = snark::rotation_matrix::rotation( _cameras.second.pose.rotation ) * snark::rotation_matrix::rotation( _cameras.first.pose.rotation ).transpose();
    std::pair< std::vector< cv::Mat >, std::vector< cv::Mat > > maps = {{ cv::Mat(), cv::Mat() }, { cv::Mat(), cv::Mat() }};
    cv::Size image_size( width, height );
    cv::Mat r1, r2, p1, p2, q;

    // Vector5d distortion( Vector5d::Zero() );
    // cv::Mat m_leftCamera;
    // cv::Mat m_leftDistortion;
    // cv::Mat m_rightCamera;
    // cv::Mat m_rightDistortion;
    // cv::Size m_imageSize;
    // cv::Mat m_rotation;
    // cv::Mat m_translation;
    //
    // cv::Mat m_R1;
    // cv::Mat m_R2;
    // cv::Mat m_P1;
    // cv::Mat m_P2;
    // cv::Mat m_Q;
    //
    // cv::Mat m_map11;
    // cv::Mat m_map12;
    // cv::Mat m_map21;
    // cv::Mat m_map22;
    // cv::eigen2cv( leftCamera, m_leftCamera );
    // cv::eigen2cv( leftDistortion, m_leftDistortion );
    // cv::eigen2cv( rightCamera, m_rightCamera );
    // cv::eigen2cv( rightDistortion, m_rightDistortion );
    // m_imageSize.width = imageWidth;
    // m_imageSize.height = imageHeight;
    // cv::eigen2cv( rotation, m_rotation );
    // cv::eigen2cv( translation, m_translation );
    //
    // cv::stereoRectify( m_leftCamera, m_leftDistortion, m_rightCamera, m_rightDistortion, m_imageSize, m_rotation, m_translation,
    //                    m_R1, m_R2, m_P1, m_P2, m_Q );
    //
    // if ( !rectified && ( leftDistortion.norm() > 1e-5 || rightDistortion.norm() > 1e-5 || !rotation.isApprox( Eigen::Matrix3d::Identity() ) ) )
    // {
    //     cv::initUndistortRectifyMap( m_leftCamera, m_leftDistortion, m_R1, m_P1, m_imageSize, CV_16SC2, m_map11, m_map12 );
    //     cv::initUndistortRectifyMap( m_rightCamera, m_rightDistortion, m_R2, m_P2, m_imageSize, CV_16SC2, m_map21, m_map22);
    // }
    // cv::stereoRectify( m_leftCamera, m_leftDistortion, m_rightCamera, m_rightDistortion, m_imageSize, m_rotation, m_translation,
    //                    m_R1, m_R2, m_P1, m_P2, m_Q );
    //
    // if ( !rectified && ( leftDistortion.norm() > 1e-5 || rightDistortion.norm() > 1e-5 || !rotation.isApprox( Eigen::Matrix3d::Identity() ) ) )
    // {
    //     cv::initUndistortRectifyMap( m_leftCamera, m_leftDistortion, m_R1, m_P1, m_imageSize, CV_16SC2, m_map11, m_map12 );
    //     cv::initUndistortRectifyMap( m_rightCamera, m_rightDistortion, m_R2, m_P2, m_imageSize, CV_16SC2, m_map21, m_map22);
    // }
    //std::cout.write( reinterpret_cast< const char* >( x.datastart ), x.dataend - x.datastart );
    //std::cout.write( reinterpret_cast< const char* >( y.datastart ), y.dataend - y.datastart );

    return maps;
}

} } } // namespace snark { namespace camera { namespace stereo {
