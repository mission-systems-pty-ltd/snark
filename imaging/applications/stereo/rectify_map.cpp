// Copyright (c) 2011 The University of Sydney

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rectify_map.h"

namespace snark { namespace imaging {

/// constructor from parameters
rectify_map::rectify_map ( const Eigen::Matrix3d& leftCamera, const Vector5d& leftDistortion, const Eigen::Matrix3d& rightCamera, const Vector5d& rightDistortion,
                           unsigned int imageWidth, unsigned int imageHeight, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation, bool rectified )
{
    cv::eigen2cv( leftCamera, m_leftCamera );
    cv::eigen2cv( leftDistortion, m_leftDistortion );
    cv::eigen2cv( rightCamera, m_rightCamera );
    cv::eigen2cv( rightDistortion, m_rightDistortion );
    m_imageSize.width = imageWidth;
    m_imageSize.height = imageHeight;
    cv::eigen2cv( rotation, m_rotation );
    cv::eigen2cv( translation, m_translation );

    cv::stereoRectify( m_leftCamera, m_leftDistortion, m_rightCamera, m_rightDistortion, m_imageSize, m_rotation, m_translation,
                       m_R1, m_R2, m_P1, m_P2, m_Q );

    if ( !rectified && ( leftDistortion.norm() > 1e-5 || rightDistortion.norm() > 1e-5 || !rotation.isApprox( Eigen::Matrix3d::Identity() ) ) )
    {
        cv::initUndistortRectifyMap( m_leftCamera, m_leftDistortion, m_R1, m_P1, m_imageSize, CV_16SC2, m_map11, m_map12 );
        cv::initUndistortRectifyMap( m_rightCamera, m_rightDistortion, m_R2, m_P2, m_imageSize, CV_16SC2, m_map21, m_map22);
    }
    else
    {
//     no rectification is needed ( pre-rectified images ), only compute Q
    }
}

/// constructor from maps
rectify_map::rectify_map ( const Eigen::Matrix3d& leftCamera, const Eigen::Matrix3d& rightCamera, const Eigen::Vector3d& translation,
                           const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y, bool rectified ):
    m_map11( left_x ),
    m_map12( left_y ),
    m_map21( right_x ),
    m_map22( right_y )
{
    Vector5d distortion( Vector5d::Zero() );
    cv::eigen2cv( leftCamera, m_leftCamera );
    cv::eigen2cv( distortion, m_leftDistortion );
    cv::eigen2cv( rightCamera, m_rightCamera );
    cv::eigen2cv( distortion, m_rightDistortion );
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    cv::eigen2cv( I, m_rotation );
    m_imageSize.width = m_map11.cols;
    m_imageSize.height = m_map11.rows;
    cv::eigen2cv( translation, m_translation );

    cv::stereoRectify( m_leftCamera, m_leftDistortion, m_rightCamera, m_rightDistortion, m_imageSize, m_rotation, m_translation,
                       m_R1, m_R2, m_P1, m_P2, m_Q );
}

cv::Mat rectify_map::remap_left ( const cv::Mat& left ) const
{
    if( m_map11.cols == 0 ) { return left; }
    cv::Mat result;
    cv::remap( left, result, m_map11, m_map12, cv::INTER_LINEAR );
    return result;
}

cv::Mat rectify_map::remap_right ( const cv::Mat& right ) const
{
    if( m_map21.cols == 0 ) { return right; }
    cv::Mat result;
    cv::remap( right, result, m_map21, m_map22, cv::INTER_LINEAR );
    return result;
}

} }
