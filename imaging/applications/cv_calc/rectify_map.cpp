// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <type_traits>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/visiting/traits.h>
#include "../../../visiting/eigen.h"
#include "rectify_map.h"

namespace snark { namespace cv_calc { namespace rectify_map {

// cv::stereoRectify( m_leftCamera, m_leftDistortion, m_rightCamera, m_rightDistortion, m_imageSize, m_rotation, m_translation,
//                    m_R1, m_R2, m_P1, m_P2, m_Q );
//
// if ( !rectified && ( leftDistortion.norm() > 1e-5 || rightDistortion.norm() > 1e-5 || !rotation.isApprox( Eigen::Matrix3d::Identity() ) ) )
// {
//     cv::initUndistortRectifyMap( m_leftCamera, m_leftDistortion, m_R1, m_P1, m_imageSize, CV_16SC2, m_map11, m_map12 );
//     cv::initUndistortRectifyMap( m_rightCamera, m_rightDistortion, m_R2, m_P2, m_imageSize, CV_16SC2, m_map21, m_map22);
// }

std::string options()
{
    std::ostringstream oss;
    oss << "        --image-size,--size=<image-size>" << std::endl;
    oss << "        --left-camera,--left=<values>" << std::endl;
    oss << "        --left-pose=<values>" << std::endl;
    oss << "        --right-camera,--right=<values>" << std::endl;
    oss << "        --right-pose=<values>" << std::endl;
    oss << "        --output-new-camera-matrix;" << std::endl;
    return oss.str();
}

//std::cout.write( reinterpret_cast< const char* >( x.datastart ), x.dataend - x.datastart );
//std::cout.write( reinterpret_cast< const char* >( y.datastart ), y.dataend - y.datastart );

int run( const comma::command_line_options& options )
{
    COMMA_THROW( comma::exception, "todo" );
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace rectify_map {
