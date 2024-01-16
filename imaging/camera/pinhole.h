// Copyright (c) 2011 The University of Sydney

/// @authors vsevolod vlaskine, zhe xu

#pragma once

#include <vector>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <comma/sync/lazy.h>
#include "../../math/pose.h"

namespace snark { namespace camera {

class pinhole
{
    public:
        struct config_t
        {
            struct distortion_t
            {
                struct radial_t
                {
                    double k1, k2, k3;

                    radial_t( double k1 = 0, double k2 = 0, double k3 = 0 ) : k1( k1 ), k2( k2 ), k3( k3 ) {}

                    bool empty() const;
                };

                struct tangential_t
                {
                    double p1, p2;

                    tangential_t( double p1 = 0, double p2 = 0 ) : p1( p1 ), p2( p2 ) {}

                    bool empty() const;
                };

                radial_t radial;

                tangential_t tangential;

                std::string map_filename;

                bool empty() const;

                distortion_t( const radial_t& radial = radial_t(), const tangential_t tangential = tangential_t() ) : radial( radial ), tangential( tangential ) {}

                template < typename V > V as() const;
            };

            /// focal length in metres (if sensor_size is empty then focal length is effectively in pixels)
            double focal_length;
            //quick and dirty: if sensor_size is empty we are taking pixel size to be 1 meter !?

            /// sensor size in metres
            boost::optional< Eigen::Vector2d > sensor_size;

            /// image size in pixels
            /// @todo should it be Eigen::Vector2d?
            Eigen::Vector2i image_size;

            /// principal point in pixels; if not defined, then image_size / 2
            boost::optional< Eigen::Vector2d > principal_point;

            /// distortion
            boost::optional< distortion_t > distortion;

            /// default constructor
            config_t();

            /// return pixel size in metres or 1,1 if sensor_size is empty
            Eigen::Vector2d pixel_size() const;

            /// return principal_point or if empty returns half image size in pixels
            Eigen::Vector2d image_centre() const;

            /// return radially corrected pixel
            Eigen::Vector2d radially_corrected( const Eigen::Vector2d& p ) const;

            /// return tangentially corrected pixel
            Eigen::Vector2d tangentially_corrected( const Eigen::Vector2d& p ) const;

            /// return camera matrix
            cv::Mat camera_matrix() const;

            /// throw, if basic checks on config fail
            void validate();
        };

        pinhole( const pinhole::config_t& config );

        /// return pixel coordinates in camera frame
        Eigen::Vector3d to_cartesian( const Eigen::Vector2d& p, bool undistort = true ) const;

        //returns converts from camera frame to image pixel col,row
        Eigen::Vector2d to_pixel( const Eigen::Vector2d& p ) const;

        /// return radially and then tangentially corrected pixel
        Eigen::Vector2d undistorted( const Eigen::Vector2d& p ) const;

        /// reverse undistorted projection using the projection map
        Eigen::Vector2d distort( const Eigen::Vector2d& p ) const;

        const pinhole::config_t& config() const { return config_; }

        static std::string usage();

        class distortion_map_t
        {
            public:
                cv::Mat map_x;

                cv::Mat map_y;

                distortion_map_t( const pinhole::config_t& config );

                void write( std::ostream& os ) const;

            private:
                friend class pinhole; // quick and dirty
                std::vector< float > x_rows_; // quick and dirty, hide for now, refactor later; rows of undistort map x
                std::vector< float > y_cols_; // quick and dirty, hide for now, refactor later; transposed columns of undistort map y
        };

        const boost::optional< distortion_map_t >& distortion_map() const;

    private:
        pinhole::config_t config_;
        Eigen::Vector2d pixel_size_;
        Eigen::Vector2d image_centre_;
        comma::lazy< boost::optional< distortion_map_t > > distortion_map_; // since it's a slow operation, initialize on demand
};

struct config
{
    camera::pinhole::config_t pinhole;

    snark::pose pose;

    config() {}

    config( const pinhole::config_t& p, const snark::pose& t ): pinhole( p ), pose( t ) {}
};

} } // namespace snark { namespace camera {
