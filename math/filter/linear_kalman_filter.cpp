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
    , H( Eigen::MatrixXd::Zero( _measurement_dimensions, state_dimensions ) )
    , _q_variance( process_noise )
    , _r_variance( measurement_noise )
{    
}

void linear_kalman_filter::measurement_matrix( const Eigen::MatrixXd& m ) { H = std::move( m ); }

void linear_kalman_filter::state( const Eigen::VectorXd& x0 ) { x = std::move( x0 ); }

void linear_kalman_filter::covariance( const Eigen::MatrixXd& p0 ) { P = std::move( p0 ); }

const Eigen::VectorXd& linear_kalman_filter::update( const Eigen::VectorXd& measurement, double dt ) // quick and dirty
{
    COMMA_ASSERT( dt >= 0, "expected non-negative time step; got: " << dt );
    F = Eigen::MatrixXd::Identity( _state_dimensions, _state_dimensions );
    static unsigned int half = _state_dimensions / 2;
    for( unsigned int i = 0; i < half; ++i ) { F( i, half + i ) = dt; } // new = old + v * dt
    return _update( measurement, dt );
}

const Eigen::VectorXd& linear_kalman_filter::update( const Eigen::VectorXd& measurement, double dt, const Eigen::MatrixXd& f )
{
    COMMA_ASSERT( dt >= 0, "expected non-negative time step; got: " << dt );
    F = std::move( f );
    return _update( measurement, dt );
}

const Eigen::VectorXd& linear_kalman_filter::_update( const Eigen::VectorXd& measurement, double dt )
{
    if( dt < 1e-6 ) { return x; }
    Eigen::MatrixXd Q = ( F * F.transpose() ) * _q_variance * dt; 
    x = F * x;
    P = F * P * F.transpose() + Q;
    Eigen::MatrixXd S = H * P * H.transpose();
    S.diagonal().array() += _r_variance; 
    static const auto state_identity = Eigen::MatrixXd::Identity(_state_dimensions, _state_dimensions );
    static const auto measurement_identity = Eigen::MatrixXd::Identity( _measurement_dimensions, _measurement_dimensions );
    Eigen::MatrixXd K = P * H.transpose() * S.colPivHouseholderQr().solve( measurement_identity );
    P = ( state_identity - K * H ) * P;
    x = x + K * ( measurement - H * x );
    return x;
}

} // namespace snark {
