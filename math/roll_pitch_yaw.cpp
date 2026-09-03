// Copyright (c) 2026 Vsevolod Vlaskine

#include "roll_pitch_yaw.h"
#include "rotation_matrix.h"

#include <iostream>

namespace snark {

// roll_pitch_yaw roll_pitch_yaw::from_rodriques( const Eigen::Vector3d& r ) { return rotation_matrix::roll_pitch_yaw( Eigen::AngleAxisd( r.norm(), r.normalized() ).toRotationMatrix() ); }

// roll_pitch_yaw roll_pitch_yaw::from_rodriques( const Eigen::Vector3d& r )
// {
//     double angle = r.norm();
//     const Eigen::Vector3d& axis = angle > 1e-6? r.normalized() : Eigen::Vector3d::UnitX();
//     roll_pitch_yaw rpy( rotation_matrix::roll_pitch_yaw( Eigen::AngleAxisd( angle, axis ).toRotationMatrix() ) );
//     //std::cerr << "==> roll_pitch_yaw::from_rodriques: r: " << r[0] << ","  << r[1] << ","  << r[2] << " angle: " << ( r.norm() * 180 / M_PI ) << " euler: " << ( rpy.roll() * 180 / M_PI ) << "," << ( rpy.pitch() * 180 / M_PI ) << ","  << ( rpy.yaw() * 180 / M_PI ) << std::endl;
//     return rpy;
// }

roll_pitch_yaw roll_pitch_yaw::from_rodriques( const Eigen::Vector3d& r )
{
    double angle = r.norm();
    Eigen::Vector3d axis = angle > 1e-6 ? r.normalized() : Eigen::Vector3d::UnitX();
    Eigen::Matrix3d r = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    // double true_z_twist_rad = std::atan2(R(1, 0), R(0, 0));
    // double true_z_twist_deg = true_z_twist_rad * 180.0 / M_PI;
    //std::cerr << "==> PURE CAMERA Z-TWIST ANGLE: " << true_z_twist_deg << "°" << std::endl;
    Eigen::Vector3d euler_zyx = r.eulerAngles(2, 1, 0); // Z, Y, X order
    return rotation_matrix::roll_pitch_yaw(R);
}

roll_pitch_yaw roll_pitch_yaw::from_rodriques( double x, double y, double z ) { return from_rodriques( Eigen::Vector3d( x, y, z ) ); }

} // namespace snark {
