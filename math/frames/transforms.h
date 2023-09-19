// Copyright (c) 2011 The University of Sydney
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../roll_pitch_yaw.h"

namespace snark { namespace frames {

///Denavit-Hartenberg parameters for robotic link
struct dh_transform
{
    double d;
    double theta;
    double r;
    double alpha;

    dh_transform() : d(0), theta(0), r(0), alpha(0) {}

    dh_transform(double d_, double theta_, double r_, double alpha_) : d(d_), theta(theta_), r(r_), alpha(alpha_) {}
};

struct transform
{
    Eigen::Vector3d translation;
    snark::roll_pitch_yaw rotation;

    transform() : translation( Eigen::Vector3d::Zero() ), rotation( 0, 0, 0 ) {}

    transform( const Eigen::Vector3d& translation, const snark::roll_pitch_yaw& rotation ): translation( translation ), rotation( rotation ) {}

    ::Eigen::Affine3d affine() const;

    ::Eigen::Affine3d inverse_affine() const;
};

struct tr_transform
{
    Eigen::Vector3d translation;
    Eigen::Quaternion<double> rotation;
    Eigen::Matrix4d to_matrix() const;

    tr_transform() : translation(Eigen::Vector3d::Zero()), rotation(1,0,0,0){}
};

/// inverts a homogeneous transform using transpose formula
Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T);

/// provides the homogeneous transform from rotation matrix and translation vector
Eigen::Matrix4d homogeneous_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

/// converts homogeneous transform to tr
tr_transform matrix_to_tr(const Eigen::Matrix4d& T);

/// provides the homogeneous transform from the dh parameters
Eigen::Matrix4d dh_to_matrix(const dh_transform& T_dh);

/// dh to tr
tr_transform dh_to_tr(const dh_transform& T_dh);

} } // namespace snark { namespace frames {
