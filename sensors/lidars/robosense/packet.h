// Copyright (c) 2018 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <map>
#include <boost/array.hpp>
#include <boost/static_assert.hpp>
#include <Eigen/Core>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/big_endian.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>

namespace snark { namespace robosense {

struct models
{
    enum values { lidar_16 = 0x01
                , lidar_32 = 0x02
                , bpearl = 0x03
                , ruby = 0x04
                , ruby_lite = 0x05
                , helios_5515 = 0x06
                , helios_16p = 0x07
                , helios_1615 = 0x07 /* sic, helios_16p and helios_1615 have the same 0x07 model value */ };
    static std::string to_string( values value );
    static values from_string( const std::string& name );
    static const std::map< std::string, values > names;
};

struct msop
{
    struct packet : public comma::packed::packed_struct< packet, 1248 >
    {
        struct header_t: public comma::packed::packed_struct< header_t, 42 >
        {
            static const char* sentinel_value() { return "\x55\xAA\x05\x0a\x5a\xa5\x50\xA0"; }

            models::values model_value() const { return static_cast< models::values >( model() ); }

            comma::packed::string< 8 > sentinel;
            boost::array< char, 12 > reserved_0;
            boost::array< char, 10 > timestamp;
            comma::packed::byte model;
            boost::array< char, 11 > reserved_1;
        };
        
        struct data_t : public comma::packed::packed_struct< data_t, 1206 > // todo: parametrize on lidar type (ok for now since only 16-beam lidars: rs_lidar_16 and rs_helios_16p are supported)
        {
            enum { number_of_lasers = 16
                 , number_of_blocks = 12
                 , number_of_subblocks = 2
                 , number_of_returns_per_packet = number_of_lasers * number_of_subblocks * number_of_blocks };
            
            struct laser_return: public comma::packed::packed_struct< laser_return, 3 >
            {
                static double firing_interval() { return 0.000003; } // 3 microseconds, see Appendix A
                
                comma::packed::big_endian::uint16 range; // see 5.1.2.3
                comma::packed::byte reflectivity;
            };
            
            struct block: public comma::packed::packed_struct< block, 2 + 2 + number_of_lasers * number_of_subblocks * laser_return::size >
            {
                static double firing_interval() { return 0.0001; } // 100 microseconds, see 5.1.2.2
        
                static const char* sentinel_value() { return "\xFF\xEE"; }
                
                comma::packed::string< 2 > sentinel;
                comma::packed::big_endian::uint16 azimuth; // 0 is Y axis positive direction; see 5.1.2.1
                boost::array< boost::array< laser_return, number_of_lasers >, 2 > channels;
                
                double azimuth_as_radians() const { return ( 0.01 * azimuth() ) * M_PI / 180; }
            };
            
            struct tail_t: public comma::packed::packed_struct< tail_t, 6 >
            {
                static const char* sentinel_value() { return "\x00\xFF"; }
                comma::packed::big_endian::uint32 reserved;
                comma::packed::string< 2 > sentinel;
            };

            boost::array< block, number_of_blocks > blocks;
            tail_t tail;
            
            std::pair< double, double > azimuths( unsigned int block ) const; // see 5.1.2.2
        };
        
        header_t header;
        data_t data;
        
        class const_iterator;
        
        bool valid() const;
    };
};

class msop::packet::const_iterator
{
    public:
        struct value_type
        {
            comma::uint32 id;
            double azimuth_step;
            double delay;
            unsigned int range;
            double azimuth;
            comma::uint32 reflectivity;

            value_type() : id( 0 ), azimuth_step( 0 ), delay( 0 ), range( 0 ), azimuth( 0 ), reflectivity( 0 ) {}
            
            bool valid() const;
        };

        const_iterator();

        const_iterator( const packet* p );

        void operator++();

        const value_type* operator->() const { return &value_; }

        const value_type& operator*() const { return value_; }

        bool done() const { return done_; }

    private:
        const msop::packet* packet_;
        unsigned int block_;
        unsigned int subblock_;
        value_type value_;
        bool done_;
        void update_value_();
        void update_value_( double azimuth );
        void update_value_( const std::pair< double, double >& azimuths );
};

namespace lidar_16 {

struct difop
{
    struct packet: public comma::packed::packed_struct< packet, 1248 >
    {
        struct header_t: public comma::packed::packed_struct< header_t, 8 >
        {
            std::array< char, 8 > sentinel;
            static const char* sentinel_value() { return "\xA5\xFF\x00\x5A\x11\x11\x55\x55"; }
            bool valid() const { return ::memcmp( sentinel.data(), sentinel_value(), 8 ) == 0; } // todo: quick and dirty; implement packed::bytes
        };
        
        struct data_t: public comma::packed::packed_struct< data_t, 1238 >
        {
            comma::packed::big_endian::uint16 motor_rotation_speed;
            comma::packed::string< 26 > ethernet;
            comma::packed::big_endian::uint16 corrected_static_base;
            comma::packed::big_endian::uint16 motor_phase_lock;
            struct top_board_firmware_version_t: public comma::packed::packed_struct< top_board_firmware_version_t, 5 >
            {
                comma::packed::string< 5 > value;
                double range_resolution() const;
            };
            struct bottom_board_firmware_version_t: public comma::packed::packed_struct< bottom_board_firmware_version_t, 5 >
            {
                comma::packed::string< 5 > value;
            };
            top_board_firmware_version_t top_board_firmware_version;
            bottom_board_firmware_version_t bottom_board_firmware_version;
            std::array< char, 240 > corrected_intensity_curves_coefficient;
            comma::packed::string< 2 > reserved_0;
            comma::packed::string< 6 > serial_number;
            comma::packed::string< 3 > reserved_1;
            comma::packed::string< 2 > upper_computer_compatibility;
            comma::packed::string< 10 > utc_time;
            comma::packed::string< 18 > operation_status;
            comma::packed::string< 11 > reserved_2;
            std::array< char, 40 > fault_diagnosis;
            comma::packed::string< 86 >  gpsrmc;
            std::array< char, 697 > corrected_static;
            struct corrected_vertical_angles_t: public comma::packed::packed_struct< corrected_vertical_angles_t, 3 * msop::packet::data_t::number_of_lasers >
            {
                /// rs-lidar-16 user's manual, section b6: The value of the pitch is unsigned integer.
                ///                                        Channel 1 to Channel 8 pitches downwards,
                ///                                        channel 9 to channel 16 pitches upwards.
                std::array< comma::packed::big_endian::uint24, msop::packet::data_t::number_of_lasers > values;
                double as_radians( unsigned int i ) const { return values[i]() * 0.0001 * M_PI / 180 * ( i < msop::packet::data_t::number_of_lasers / 2 ? -1 : 1 ); }
                bool empty() const;
            };
            corrected_vertical_angles_t corrected_vertical_angles;
            comma::packed::string< 33 > reserved_3;
        };
        
        struct tail_t: public comma::packed::packed_struct< tail_t, 2 >
        {
            static const char* sentinel_value() { return "\x0F\xF0"; }
            comma::packed::string< 2 > sentinel;
        };
        
        header_t header;
        data_t data;
        tail_t tail;
    };
};

} // namespace lidar_16 {

namespace helios_16p {

struct difop // todo!
{
    struct packet: public comma::packed::packed_struct< packet, 1248 >
    {
        struct header_t: public comma::packed::packed_struct< header_t, 8 >
        {
            std::array< char, 8 > sentinel;
            static const char* sentinel_value() { return "\xA5\xFF\x00\x5A\x11\x11\x55\x55"; }
            bool valid() const { return ::memcmp( sentinel.data(), sentinel_value(), 8 ) == 0; } // todo: quick and dirty; implement packed::bytes
        };
        
        struct data_t: public comma::packed::packed_struct< data_t, 1238 >
        {
            comma::packed::big_endian::uint16 motor_rotation_speed;
            comma::packed::string< 26 > ethernet;
            comma::packed::big_endian::uint16 corrected_static_base;
            comma::packed::big_endian::uint16 motor_phase_lock;
            struct top_board_firmware_version_t: public comma::packed::packed_struct< top_board_firmware_version_t, 5 >
            {
                comma::packed::string< 5 > value;
                double range_resolution() const;
            };
            struct bottom_board_firmware_version_t: public comma::packed::packed_struct< bottom_board_firmware_version_t, 5 >
            {
                comma::packed::string< 5 > value;
            };
            top_board_firmware_version_t top_board_firmware_version;
            bottom_board_firmware_version_t bottom_board_firmware_version;
            std::array< char, 240 > corrected_intensity_curves_coefficient;
            comma::packed::string< 2 > reserved_0;
            comma::packed::string< 6 > serial_number;
            comma::packed::string< 3 > reserved_1;
            comma::packed::string< 2 > upper_computer_compatibility;
            comma::packed::string< 10 > utc_time;
            comma::packed::string< 18 > operation_status;
            comma::packed::string< 11 > reserved_2;
            std::array< char, 40 > fault_diagnosis;
            comma::packed::string< 86 >  gpsrmc;
            std::array< char, 697 > corrected_static;
            struct corrected_vertical_angles_t: public comma::packed::packed_struct< corrected_vertical_angles_t, 3 * msop::packet::data_t::number_of_lasers >
            {
                /// rs-lidar-16 user's manual, section b6: The value of the pitch is unsigned integer.
                ///                                        Channel 1 to Channel 8 pitches downwards,
                ///                                        channel 9 to channel 16 pitches upwards.
                std::array< comma::packed::big_endian::uint24, msop::packet::data_t::number_of_lasers > values;
                double as_radians( unsigned int i ) const { return values[i]() * 0.0001 * M_PI / 180 * ( i < msop::packet::data_t::number_of_lasers / 2 ? -1 : 1 ); }
                bool empty() const;
            };
            corrected_vertical_angles_t corrected_vertical_angles;
            comma::packed::string< 33 > reserved_3;
        };
        
        struct tail_t: public comma::packed::packed_struct< tail_t, 2 >
        {
            static const char* sentinel_value() { return "\x0F\xF0"; }
            comma::packed::string< 2 > sentinel;
        };
        
        header_t header;
        data_t data;
        tail_t tail;
    };
};

} // namespace helios_16p {

} } // namespace snark { namespace robosense {
