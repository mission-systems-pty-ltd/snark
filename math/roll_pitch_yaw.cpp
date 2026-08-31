// Copyright (c) 2026 Vsevolod Vlaskine

#include "roll_pitch_yaw.h"
#include "rotation_matrix.h"

namespace snark {

roll_pitch_yaw roll_pitch_yaw::from_rodriques( const Eigen::Vector3d& r ) { return rotation_matrix::roll_pitch_yaw( Eigen::AngleAxisd( r.norm(), r.normalized() ).toRotationMatrix() ); }

roll_pitch_yaw roll_pitch_yaw::from_rodriques( double x, double y, double z ) { return from_rodriques( Eigen::Vector3d( x, y, z ) ); }

} // namespace snark {
