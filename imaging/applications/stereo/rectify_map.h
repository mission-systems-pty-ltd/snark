// Copyright (c) 2011 The University of Sydney

#pragma once

#include <opencv2/calib3d.hpp>
#include <Eigen/Core>

namespace snark { namespace imaging {

/// compute rectify maps for a camera pair
class rectify_map
{
public:

    typedef Eigen::Matrix< double, 5, 1 > Vector5d;

    rectify_map( const Eigen::Matrix3d& leftCamera, const Vector5d& leftDistortion, const Eigen::Matrix3d& rightCamera, const Vector5d& rightDistortion,
                 unsigned int imageWidth, unsigned int imageHeight, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation, bool rectified = false );

    rectify_map( const Eigen::Matrix3d& leftCamera, const Eigen::Matrix3d& rightCamera, const Eigen::Vector3d& translation,
                 const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y, bool rectified = false );

    /// return the Q matrix
    const cv::Mat& Q() const { return m_Q; }

    const cv::Mat& map11() const { return m_map11; };
    const cv::Mat& map12() const { return m_map12; };
    const cv::Mat& map21() const { return m_map21; };
    const cv::Mat& map22() const { return m_map22; };

    cv::Mat remap_left( const cv::Mat& left ) const;
    cv::Mat remap_right( const cv::Mat& right ) const;

private:
    cv::Mat m_leftCamera;
    cv::Mat m_leftDistortion;
    cv::Mat m_rightCamera;
    cv::Mat m_rightDistortion;
    cv::Size m_imageSize;
    cv::Mat m_rotation;
    cv::Mat m_translation;

    cv::Mat m_R1;
    cv::Mat m_R2;
    cv::Mat m_P1;
    cv::Mat m_P2;
    cv::Mat m_Q;

    cv::Mat m_map11;
    cv::Mat m_map12;
    cv::Mat m_map21;
    cv::Mat m_map22;
};

} }
