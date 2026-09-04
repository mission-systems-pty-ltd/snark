// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace snark { 

/// generic kalman filter
template< class State, class Model >
class kalman_filter
{
public:
    /// constructor
    /// @param state container for state and covariance
    /// @param model motion model
    kalman_filter( const State& initial_state, Model& model ) : state( initial_state ), model_( model ) {}

    /// predict step
    /// @param deltaT time in seconds since last prediction
    void predict( double deltaT ) { predict( deltaT, model_ ); }

    template < typename M >
    void predict( double deltaT, M& model )
    {
        const Eigen::Matrix< double, State::dimension, State::dimension >& A = model.jacobian( state, deltaT );
        state.covariance = A * state.covariance * A.transpose() + model.noise_covariance( deltaT );
        model.update_state( state, deltaT );
    }

    /// update step
    /// @param m measurement model with jacobian and covariance
    /// @param chi_squared_threshold mahalanobis rejection threshold chosen from chi-squared table or empirically
    template< class Measurement >
    bool update( const Measurement& m, double chi_squared_threshold=0.0)
    {
        const Eigen::Matrix<double,Measurement::dimension,State::dimension>& H = m.measurement_jacobian( state );
        const Eigen::Matrix<double,Measurement::dimension,Measurement::dimension>& R = m.measurement_covariance( state );
        const Eigen::Matrix<double,Measurement::dimension,1>& innovation = m.innovation( state );

        const Eigen::Matrix<double,State::dimension, Measurement::dimension> PHt = state.covariance * H.transpose();
        const Eigen::LDLT< Eigen::Matrix<double,Measurement::dimension,Measurement::dimension> > S = ( H * PHt + R ).ldlt();

        if ( chi_squared_threshold > 0.0 )
        {
            double mahalanobis_distance = sqrt((innovation.transpose() * S.solve(innovation))(0,0));
            if ( mahalanobis_distance > chi_squared_threshold) { return false; }
        }
        state.covariance -= PHt * S.solve( PHt.transpose() );
        state.set_innovation( PHt * S.solve( innovation ) );
        return true;
    }

    State state;

private:
    Model& model_; /// process model
};

} // namespace snark {
