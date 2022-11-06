// Copyright (c) 2018-2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <array>
#include <string>
#include <Eigen/Core>
#include <boost/date_time/posix_time/ptime.hpp>
#include "packet.h"

namespace snark { namespace robosense {

class calculator
{
    public:
        struct point
        {
            boost::posix_time::ptime t;
            comma::uint32 scan;
            comma::uint32 id;
            double range;
            double bearing;
            double elevation;
            comma::uint32 reflectivity;
            Eigen::Vector3d coordinates;
            
            point(): scan( 0 ), id( 0 ), range( 0 ), bearing( 0 ), elevation( 0 ), reflectivity( 0 ), coordinates( Eigen::Vector3d::Zero() ) {}
            
            bool valid() const;
        };
        
        class scan
        {
            public:
                struct data
                {
                    comma::uint32 id;
                    comma::uint32 begin;
                    comma::uint32 end;
                    comma::uint32 size;
                    
                    data(): id( 0 ), begin( 0 ), end( 0 ), size( 0 ) {}
                    
                    bool is_new() const { return size == 1; }
                };
                
                scan( unsigned int max_number_of_missing_packets = 100, bool accumulate = false ); // todo: arbitrary: 10 missing packets by default will trigger a new scan and detect the current scan as incomplete
                
                void update( const boost::posix_time::ptime& timestamp, const robosense::msop::data& data );
                
                bool is_complete( const data& d ) const;
                
                const data& current() const { return scans_[current_]; }
                
                const data& last() const { return scans_[ 1 - current_ ]; }

            private:
                unsigned int max_number_of_missing_packets_;
                boost::posix_time::time_duration max_time_gap_;
                comma::uint32 estimated_max_angle_gap_;
                bool accumulate_;
                std::array< scan::data, 2 > scans_;
                unsigned int current_;
                boost::posix_time::ptime last_timestamp_;
        };

        typedef std::array< double, robosense::msop::data::number_of_lasers > angles_t;
        
        calculator();

        calculator( const angles_t& elevation, double range_resolution );
        
        calculator( const angles_t& azimuth, const angles_t& elevation, double range_resolution );
        
        calculator( const std::string& elevation, const std::string& channel_num, double range_resolution ); // todo: generalize to 32 beams; todo: azimuth usage semantics
        
        double range( unsigned int r, unsigned int laser, unsigned int temperature ) const;
        
        ::Eigen::Vector3d to_cartesian( unsigned int laser, double range, double angle ) const;
        
        double intensity( unsigned int laser, unsigned char intensity, double distance ) const; // todo

        const angles_t& azimuth() const { return azimuth_; }
        
        const angles_t& elevation() const { return elevation_; }
        
        point make_point( comma::uint32 scan, const boost::posix_time::ptime& t, const robosense::msop::const_iterator& it, unsigned int temperature );
        
        double range_resolution() const { return range_resolution_; }
        
    private:
        angles_t azimuth_;
        angles_t elevation_;
        typedef std::array< std::array< double, 41 >, robosense::msop::data::number_of_lasers > channel_num_t_;
        boost::optional< channel_num_t_ > channel_num_;
        double range_resolution_;
        struct laser_
        {
            double sin;
            double cos;
            
            laser_(): sin( 0 ), cos( 0 ) {}
            laser_( unsigned int index, const std::array< double, 16 >& elevation ) : sin( std::sin( elevation[ index ] ) ), cos( std::cos( elevation[ index ] ) ) {}
        };
        typedef std::array< laser_, robosense::msop::data::number_of_lasers > lasers_t_;
        lasers_t_ lasers_;
        void init_lasers_();
};

} } // namespace snark { namespace robosense {
