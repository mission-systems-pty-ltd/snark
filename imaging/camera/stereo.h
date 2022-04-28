// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include "../../math/frame_transforms.h"
#include "pinhole.h"

namespace snark { namespace camera { namespace stereo {

class pair
{
    public:
        typedef std::pair< camera::config, camera::config > config_t;

        struct camera_t
        {
            /// pinhole camera
            snark::camera::pinhole pinhole;

            /// pose in north-east-down frame
            snark::pose pose;

            /// constructor
            camera_t( const snark::camera::config& config ): pinhole( config.pinhole ), pose( config.pose ) {}

            /// constructor
            camera_t( const snark::camera::pinhole::config_t& config, const snark::pose& pose ): pinhole( config ), pose( pose ) {}
        };

        pair( const config_t& config );

        pair( const pinhole::config_t& pinhole, double baseline );

        pair( const pinhole::config_t& first, const pinhole::config_t& second, double baseline );

        const camera_t& first() const { return first_; }

        const camera_t& second() const { return second_; }

        std::pair< Eigen::Vector3d, Eigen::Vector3d > to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second ) const;

        std::pair< Eigen::Vector3d, Eigen::Vector3d > to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& pose ) const;

        std::pair< Eigen::Vector3d, Eigen::Vector3d > to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& first_pose, const snark::pose& second_pose ) const;

    private:
        camera_t first_;
        camera_t second_;
};

} } } // namespace snark { namespace camera { namespace stereo {
