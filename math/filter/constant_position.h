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

namespace snark { namespace constant_position {
    
struct state
{
    state(): covariance( Eigen::Matrix< double, dimension, dimension >::Identity() ) {}

    void set_innovation( const Eigen::Vector3d& innovation ) { position += innovation; }

    static const unsigned int dimension = 3u;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};

class model
{
    public:
        model()
            : _jacobian( Eigen::Matrix< double, state::dimension, state::dimension >::Identity() )
            , _noise( Eigen::Matrix< double, state::dimension, state::dimension >::Zero() )
            , _noise_sigma( Eigen::Matrix< double, state::dimension, 1 >::Ones() )
        {}

        Eigen::Matrix< double, state::dimension, state::dimension >& jacobian( const state& state, double dt ) { return _jacobian; }

        void update_state( state& state, const double dt ) {}

        Eigen::Matrix< double, state::dimension, state::dimension >& noise_covariance( double dt )
        {
            for( unsigned int i = 0; i < 3; i++ ) { _noise( i, i ) = dt * sigma[i]; }
            return noise;
        }

    private:
        Eigen::Matrix< double, state::dimension, state::dimension > _jacobian;
        Eigen::Matrix< double, state::dimension, state::dimension > _noise;
        Eigen::Matrix< double, state::dimension, 1 > _noise_sigma;
};

struct position
{
    static const int dimension = 3;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
    Eigen::Matrix3d jacobian;

    position(): position( Eigen::Vector3d::Zero() ), covariance( Eigen::Matrix3d::Identity() ), jacobian( Eigen::Matrix3d::Identity() ) {}
    const Eigen::Matrix3d& measurement_jacobian( const state & state ) const { return jacobian; }
    const Eigen::Matrix3d measurement_covariance( const state & state ) const { return covariance; }
    const Eigen::Matrix< double, dimension, 1 > innovation( const state & state ) const { return position; }
};

} } // namespace snark { namespace constant_position {
