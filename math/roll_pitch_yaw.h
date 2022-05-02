// Copyright (c) 2011 The University of Sydney

#pragma once

#include <Eigen/Core>

namespace snark {

struct roll_pitch_yaw : public Eigen::Vector3d
{
    double roll() const { return x(); }
    double pitch() const { return y(); }
    double yaw() const { return z(); }

    void roll( double r ) { x() = r; }
    void pitch( double p ) { y() = p; }
    void yaw( double w ) { z() = w; }

    roll_pitch_yaw() : Eigen::Vector3d( Eigen::Vector3d::Zero() ) {}
    roll_pitch_yaw( const Eigen::Vector3d& v ) : Eigen::Vector3d( v ) {}
    roll_pitch_yaw( double roll, double pitch, double yaw ) : Eigen::Vector3d( roll, pitch, yaw ) {}
};

} // namespace snark {
