// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2026 Vsevolod Vlaskine
// All rights reserved.

#pragma once

#include <optional>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace snark {

class linear_kalman_filter
{
    public:
        linear_kalman_filter( unsigned int state_dimensions
                            , unsigned int measurement_dimensions
                            , double process_noise
                            , double measurement_noise );

        void measurement_matrix( const Eigen::MatrixXd& measurement_matrix );

        void state( const Eigen::VectorXd& x0 );
        
        void covariance( const Eigen::MatrixXd& p0 );

        const Eigen::VectorXd& update( const Eigen::VectorXd& measurement, double dt );

        const Eigen::VectorXd& update( const Eigen::VectorXd& measurement, double dt, const Eigen::MatrixXd& F );

        const Eigen::VectorXd& state() const { return x; }

    private:
        int _state_dimensions;
        int _measurement_dimensions;
        Eigen::VectorXd x; // state vector [_state_dimensions x 1]
        Eigen::MatrixXd F;
        Eigen::MatrixXd P; // estimate covariance matrix [_state_dimensions x _state_dimensions]
        Eigen::MatrixXd H; // measurement mapping matrix [_measurement_dimensions x _state_dimensions]
        double _q_variance; // uniform process noise variance (scalar)
        double _r_variance; // uniform measurement noise variance (scalar)
        std::optional< double > _dt{};
        const Eigen::VectorXd& _update( const Eigen::VectorXd& measurement, double dt );
};

} // namespace snark {
