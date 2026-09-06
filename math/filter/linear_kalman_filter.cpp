// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2026 Vsevolod Vlaskine
// All rights reserved.

#include <utility>
#include <comma/base/exception.h>
#include "linear_kalman_filter.h"

namespace snark {

linear_kalman_filter::linear_kalman_filter( unsigned int state_dimensions
                                          , unsigned int measurement_dimensions
                                          , double process_noise
                                          , double measurement_noise )
    : _state_dimensions( state_dimensions )
    , _measurement_dimensions( measurement_dimensions )
    , x( Eigen::VectorXd::Zero( state_dimensions ) )
    , P( Eigen::MatrixXd::Identity( state_dimensions, state_dimensions ) * 1.0 )
    , H( Eigen::MatrixXd::Zero(_measurement_dimensions, state_dimensions ) )
    , _q_variance( process_noise )
    , _r_variance( measurement_noise )
{    
}

void linear_kalman_filter::measurement_matrix( const Eigen::MatrixXd& m ) { H = std::move( m ); }

void linear_kalman_filter::state( const Eigen::VectorXd& x0 ) { x = std::move( x0 ); }

void linear_kalman_filter::covariance( const Eigen::MatrixXd& p0 ) { P = std::move( p0 ); }

const Eigen::VectorXd& update( const Eigen::VectorXd& measurement, double dt )
{
    // todo!
    // Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    // F(0, 3) = dt; // x_new  = x_old  + (vx * dt)
    // F(1, 4) = dt; // y_new  = y_old  + (vy * dt)
    // F(2, 5) = dt; // z_new  = z_old  + (vz * dt)
    COMMA_THROW( comma::exception, "implementing..." );
}

const Eigen::VectorXd& linear_kalman_filter::update( const Eigen::VectorXd& measurement, double dt, const Eigen::MatrixXd& f )
{
    COMMA_ASSERT( dt >= 0, "expected non-negative time step; got: " << dt );
    F = std::move( f );
    if( dt < 1e-6 ) { return x; }

    // 1. Time Update (Predict)
    // With uniform noise, Q simplifies to the transition matrix uncertainty footprint
    // scaled uniformly by your process variance coefficient
    Eigen::MatrixXd Q = (F * F.transpose()) * _q_variance * dt; 
    
    x = F * x;
    P = F * P * F.transpose() + Q;

    // 2. Measurement Update (Correct)
    // R is generated instantly on the stack as an identity matrix multiplied by the scalar variance
    Eigen::MatrixXd S = H * P * H.transpose();
    S.diagonal().array() += _r_variance; 

    // Robust solve to find Kalman Gain
    Eigen::MatrixXd K = P * H.transpose() * S.colPivHouseholderQr().solve(Eigen::MatrixXd::Identity(_measurement_dimensions, _measurement_dimensions));

    x = x + K * (measurement - H * x);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_state_dimensions, _state_dimensions);
    P = (I - K * H) * P;

    return x;
}

} // namespace snark {
