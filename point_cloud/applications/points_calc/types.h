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

namespace snark { namespace points_calc {

struct plane
{
    Eigen::Vector3d normal;
    Eigen::Vector3d point;
    
    plane() : normal( Eigen::Vector3d::Zero() ), point( Eigen::Vector3d::Zero() ) {}
    plane( const Eigen::Vector3d& normal, const Eigen::Vector3d& point ) : normal( normal ), point( point ) {}
};

struct line
{
    typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > pair;
    
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    
    line() : origin( Eigen::Vector3d::Zero() ), direction( Eigen::Vector3d::Zero() ) {}
    line( const Eigen::Vector3d& origin, const Eigen::Vector3d& direction ) : origin( origin ), direction( direction ) {}
    line( const pair& p ) : origin( p.first ), direction( p.second - p.first ) {}
};

} } // namespace snark { namespace points_calc {
