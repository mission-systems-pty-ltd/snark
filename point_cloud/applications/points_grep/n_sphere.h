// Copyright (c) 2014 The University of Sydney

#pragma once

#include <Eigen/Core>

namespace snark { namespace geometry {

class n_sphere
{
    public:
        /// @param center of the n-sphere
        /// @param radius of the n-sphere
        n_sphere( const Eigen::VectorXd& center, const double radius );

        /// @return true, this n-sphere contains the point p
        bool contains( const Eigen::VectorXd& p, const double epsilon = 1e-9 ) const;

        /// @return true, this n-sphere intersects with the n-sphere s
        bool intersects( const n_sphere& s, const double epsilon = 1e-9 ) const;

    private:
        const Eigen::VectorXd center;
        const double radius;
        const double squared_radius;
};

} } // namespace snark { namepsace geometry {
