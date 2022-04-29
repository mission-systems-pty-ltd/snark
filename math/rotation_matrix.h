// Copyright (c) 2011 The University of Sydney

#pragma once

#include <Eigen/Geometry>
#include "angle.h"
#include "roll_pitch_yaw.h"

namespace snark {

class rotation_matrix
{
    public:
        rotation_matrix( const Eigen::Matrix3d& rotation =  Eigen::Matrix3d::Identity() ) : m_rotation( rotation ) {}

        rotation_matrix( const Eigen::Quaterniond& quaternion ) : m_rotation( quaternion.normalized() ) {}

        rotation_matrix( const Eigen::Vector3d& rpy ) : m_rotation( rotation( rpy.x(), rpy.y(), rpy.z() ) ) {}

        rotation_matrix( const Eigen::AngleAxisd& angle_axis ) : m_rotation( angle_axis ) {}

        const Eigen::Matrix3d& rotation() const { return m_rotation; }

        Eigen::Quaterniond quaternion() const { return Eigen::Quaterniond( m_rotation ); }

        Eigen::Vector3d roll_pitch_yaw() const { return roll_pitch_yaw( m_rotation ); }

        Eigen::Vector3d angle_axis() const;

        static Eigen::Vector3d roll_pitch_yaw( const ::Eigen::Matrix3d& m );

        static Eigen::Matrix3d rotation( const snark::roll_pitch_yaw& rpy );

        static Eigen::Matrix3d rotation( double roll, double pitch, double yaw );

        static Eigen::Matrix3d rotation( const ::Eigen::Vector3d& rpy );

    private:
        Eigen::Matrix3d m_rotation;
};

} // namespace snark {
