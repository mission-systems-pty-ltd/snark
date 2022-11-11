// Copyright (c) 2018-2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <array>
#include <map>
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
                , helios_16p = 0x06
                , helios_1610 = 0x06
                , helios_1615 = 0x06
                , helios = 0x06 };
    static std::string to_string( values value );
    static values from_string( const std::string& name );
    static const std::map< std::string, values > names;
};

struct utc_time: public comma::packed::packed_struct< utc_time, 10 > // helios-16p spec B.13; currently assuming it's the same time representation in lidar-16 (strong assumption)
{
    comma::packed::string< 6 > seconds; // 6-byte big endian... todo... sigh...
    comma::packed::big_endian::uint32 nanoseconds; // is it microseconds or nanoseconds? helios-16p spec B.13 has a typo
    double as_seconds() const; // todo: convert to seconds
};

template < typename Header, typename Data, typename Tail >
struct packet : public comma::packed::packed_struct< packet< Header, Data, Tail >, 1248 >
{
    typedef Header header_t;
    typedef Data data_t;
    typedef Tail tail_t;

    Header header;
    Data data;
    Tail tail;
};

struct msop
{
    struct data : public comma::packed::packed_struct< data, 1200 > // todo: parametrize on lidar type (ok for now since only 16-beam lidars: rs_lidar_16 and rs_helios_16p are supported)
    {
        enum { number_of_lasers = 16
             , number_of_blocks = 12
             , number_of_subblocks = 2
             , number_of_returns_per_packet = number_of_lasers * number_of_subblocks * number_of_blocks };
        
        struct laser_return: public comma::packed::packed_struct< laser_return, 3 >
        {
            static double firing_interval() { return 0.000003; } // 3 microseconds, see Appendix A
            
            comma::packed::big_endian::uint16 range; // see lidar-16 spec 5.1.2.3
            comma::packed::byte reflectivity;
        };
        
        struct block: public comma::packed::packed_struct< block, 2 + 2 + number_of_lasers * number_of_subblocks * laser_return::size >
        {
            static double firing_interval() { return 0.0001; } // 100 microseconds, see lidar-16 spec 5.1.2.2

            static const char* sentinel_value() { return "\xFF\xEE"; }
            
            comma::packed::string< 2 > sentinel;
            comma::packed::big_endian::uint16 azimuth; // 0 is Y axis positive direction; see lidar-16 spec 5.1.2.1
            std::array< std::array< laser_return, number_of_lasers >, 2 > channels;
            
            double azimuth_as_radians() const { return ( 0.01 * azimuth() ) * M_PI / 180; }
        };

        std::array< block, number_of_blocks > blocks;

        std::pair< double, double > azimuths( unsigned int block ) const; // see lidar-16 spec 5.1.2.2
    };

    struct tail: public comma::packed::packed_struct< tail, 6 >
    {
        static const char* sentinel_value() { return "\x00\xFF"; }
        comma::packed::big_endian::uint32 reserved;
        comma::packed::string< 2 > sentinel;
    };

    template < typename Packet >
    static bool valid( const Packet& p ) { return p.data.blocks[0].azimuth() <= 36000; } // quick and dirty; what is azimuth 0xffff?

    static models::values detect_model( const char* header ); // quick and dirty; really sucks because for a weird undisclosed reason, the model field has a different byte offset in the msop packet header for different models

    class const_iterator
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

            const_iterator( const data& d );

            void operator++();

            const value_type* operator->() const { return &value_; }

            const value_type& operator*() const { return value_; }

            bool done() const { return done_; }

        private:
            const msop::data* data_;
            unsigned int block_;
            unsigned int subblock_;
            value_type value_;
            bool done_;
            void update_value_();
            void update_value_( double azimuth );
            void update_value_( const std::pair< double, double >& azimuths );
    };
};

struct difop
{
    struct header: public comma::packed::packed_struct< header, 8 >
    {
        std::array< char, 8 > sentinel;
        static const char* sentinel_value() { return "\xA5\xFF\x00\x5A\x11\x11\x55\x55"; }
        bool valid() const { return ::memcmp( sentinel.data(), sentinel_value(), 8 ) == 0; } // todo: quick and dirty; implement packed::bytes
    };

    struct tail: public comma::packed::packed_struct< tail, 2 >
    {
        static const char* sentinel_value() { return "\x0F\xF0"; }
        comma::packed::string< 2 > sentinel;
    };
};

struct lidar_16
{
    struct msop
    {
        struct header: public comma::packed::packed_struct< header, 42 >
        {
            static const char* sentinel_value() { return "\x55\xAA\x05\x0a\x5a\xa5\x50\xA0"; }

            comma::packed::string< 8 > sentinel;
            std::array< char, 12 > reserved_0;
            std::array< char, 10 > timestamp;
            comma::packed::byte model;
            std::array< char, 11 > reserved_1;
        };

        typedef robosense::msop::data data;

        typedef robosense::packet< lidar_16::msop::header, robosense::msop::data, robosense::msop::tail > packet;
    };

    struct difop
    {
        struct data: public comma::packed::packed_struct< data, 1238 >
        {
            struct version_t: public comma::packed::packed_struct< version_t, 5 >
            {
                comma::packed::string< 5 > value;
            };

            struct top_board_firmware_version_t: public version_t // todo? is it the same structure as for rs-lidar-16?
            {
                double range_resolution() const;
            };

            struct corrected_angles: public comma::packed::packed_struct< corrected_angles, 3 * robosense::msop::data::number_of_lasers >
            {
                /// rs-lidar-16 user's manual, section b6: The value of the pitch is unsigned integer.
                ///                                        Channel 1 to Channel 8 pitches downwards,
                ///                                        channel 9 to channel 16 pitches upwards.
                std::array< comma::packed::big_endian::uint24, robosense::msop::data::number_of_lasers > values;
                double as_radians( unsigned int i ) const { return values[i]() * 0.0001 * M_PI / 180 * ( i < robosense::msop::data::number_of_lasers / 2 ? -1 : 1 ); }
                bool empty() const;
            };

            double range_resolution() const { return top_board_firmware_version.range_resolution(); }

            static const std::array< double, robosense::msop::data::number_of_lasers >& corrected_horizontal_angles_default();

            static const std::array< double, robosense::msop::data::number_of_lasers >& corrected_vertical_angles_default();

            comma::packed::big_endian::uint16 motor_rotation_speed;
            comma::packed::string< 26 > ethernet;
            comma::packed::big_endian::uint16 corrected_static_base;
            comma::packed::big_endian::uint16 motor_phase_lock;
            top_board_firmware_version_t top_board_firmware_version;
            version_t bottom_board_firmware_version;
            std::array< char, 240 > corrected_intensity_curves_coefficient;
            comma::packed::string< 2 > reserved_0;
            comma::packed::string< 6 > serial_number;
            comma::packed::string< 3 > reserved_1;
            comma::packed::string< 2 > upper_computer_compatibility;
            robosense::utc_time utc_time;
            comma::packed::string< 18 > operation_status;
            comma::packed::string< 11 > reserved_2;
            std::array< char, 40 > fault_diagnosis;
            comma::packed::string< 86 >  gprmc;
            std::array< char, 697 > corrected_static;
            corrected_angles corrected_vertical_angles;
            comma::packed::string< 33 > reserved_3;
        };

        typedef robosense::packet< robosense::difop::header, lidar_16::difop::data, robosense::difop::tail > packet;
    };

    static double range_resolution_default() { return 0.01; }

    static double range_resolution( const difop::packet* d, const msop::packet* ) { return d ? d->data.range_resolution() : range_resolution_default(); }
};

struct helios
{
    struct models
    {
        enum values { helios_5515 = 0x01, helios_1615 = 0x02, helios_16p = 0x03, helios_1610 = 0x04 };
        static std::string to_string( values m );
        static values from_string( const std::string& name );
        static const std::map< std::string, values > names;
    };
};

struct helios_16p // todo? move packet definitions to helios; then: struct helios_16p: public helios
{
    struct msop
    {
        struct header: public comma::packed::packed_struct< header, 42 >
        {
            static const char* sentinel_value() { return "\x55\xAA\x05\x5a"; }

            comma::packed::string< 4 > sentinel;
            std::array< comma::packed::byte, 2 > protocol_version; // helios-16p spec is unclear on the format
            std::array< char, 2 > reserved_0;
            comma::packed::big_endian::uint32 top_board_sending_packet_count; // helios-16p spec 6.2.1: form a sequence with a (sic) increment of 3
            comma::packed::big_endian::uint32 bottom_board_sending_packet_count;
            std::array< char, 1 > reserved_1;
            comma::packed::byte range_resolution; // helios-16p spec 6.2.1: 1: 0.25cm; 0: 0.5cm
            comma::packed::big_endian::uint16 angle_pulse_interval_count; // helios-16p spec 6.2.1: unit: us
            robosense::utc_time timestamp;
            std::array< char, 1 > reserved_2;
            comma::packed::byte type; // helios-16p spec 6.2.1 table 9
            comma::packed::byte model; // helios-16p spec 6.2.1 table 9
            std::array< char, 9 > reserved_3;
        };

        typedef robosense::msop::data data;

        struct packet: public robosense::packet< helios_16p::msop::header, robosense::msop::data, robosense::msop::tail >
        {
            double range_resolution() const { return header.range_resolution() == 0 ? 0.005 : header.range_resolution() == 1 ? 0.0025 : 0.; } // helios-16p spec 6.2.1: 1: 0.25cm; 0: 0.5cm
        };
    };

    struct difop
    {
        struct data: public comma::packed::packed_struct< data, 1238 >
        {
            struct motor_rotation_speed_t: public comma::packed::packed_struct< motor_rotation_speed_t, 2 >
            {
                comma::packed::string< 2 > value;
                unsigned int rps() const { return value() == "\x04\xb0" ? 20 : value() == "\x02\x58" ? 10 : 0; }
            };

            struct fov_setting_t
            {
                comma::packed::big_endian::uint16 start;
                comma::packed::big_endian::uint16 end;
                double start_radians() const; // todo: implement byte layout once required; see helios-16p spec B.2
                double end_radians() const; // todo: implement byte layout once required; see helios-16p spec B.2
            };

            struct version_t: public comma::packed::packed_struct< version_t, 5 >
            {
                comma::packed::string< 5 > value;
            };

            struct corrected_angles: public comma::packed::packed_struct< corrected_angles, 3 * robosense::msop::data::number_of_lasers >
            {
                struct angle: public comma::packed::packed_struct< angle, 3 >
                {
                    comma::packed::byte sign;
                    comma::packed::big_endian::uint16 value;
                    double radians() const; // helios-16p spec B.10
                };
                std::array< angle, robosense::msop::data::number_of_lasers > values;
                double as_radians( unsigned int i ) const { return values[i].radians(); }
                std::array< double, snark::robosense::msop::data::number_of_lasers > as_radians() const;
                bool empty() const;
            };

            static const std::array< double, robosense::msop::data::number_of_lasers >& corrected_horizontal_angles_default();

            static const std::array< double, robosense::msop::data::number_of_lasers >& corrected_vertical_angles_default();

            motor_rotation_speed_t motor_rotation_speed;
            comma::packed::string< 22 > ethernet; // todo: implement byte layout once required; see helios-16p spec B.2
            fov_setting_t fov_setting; // todo: implement byte layout once required; see helios-16p spec B.2
            std::array< char, 2 > reserved_0;
            comma::packed::big_endian::uint16 motor_phase_lock;
            version_t top_board_firmware_version;
            version_t bottom_board_firmware_version;
            version_t bottom_board_software_version;
            version_t motor_firmware_version;
            comma::packed::string< 3 > sensor_hardware_version;
            comma::packed::string< 4 > web_page_cgi_verion;
            comma::packed::big_endian::uint32 top_board_backup_crc; // spec 6.3: what is it? do we care?
            comma::packed::big_endian::uint32 bottom_board_backup_crc; // helios-16p spec 6.3: what is it? do we care?
            comma::packed::big_endian::uint32 software_app_backup_crc; // helios-16p spec 6.3: what is it? do we care?
            comma::packed::big_endian::uint32 web_page_cgi_backup_crc; // helios-16p spec 6.3: what is it? do we care?
            std::array< comma::packed::byte, 4 > ethernet_gateway;
            std::array< comma::packed::byte, 4 > subnet_mask;
            std::array< char, 201 > reserved_1;
            comma::packed::string< 6 > serial_number;
            comma::packed::big_endian::uint16 zero_angle_offset; // todo! looks important
            comma::packed::byte return_mode;
            comma::packed::byte time_synchronization_mode;
            comma::packed::byte synchronization_status;
            robosense::utc_time time;
            std::array< char, 12 > operating_status; // todo: helios-16p spec B.14
            std::array< char, 17 > reserved_2;
            std::array< char, 18 > fault_diagnosis; // todo: helios-16p spec B.15
            comma::packed::byte code_wheel_is_calibrated;
            comma::packed::byte gps_pps_pulse_trigger_mode;
            std::array< char, 20 > reserved_3;
            comma::packed::string< 86 > gprmc; // todo: helios-16p spec B.16
            corrected_angles corrected_vertical_angles;
            std::array< char, 48 > reserved_4;
            corrected_angles corrected_horizontal_angles;
            std::array< char, 634 > reserved_5;
        };

        typedef robosense::packet< robosense::difop::header, helios_16p::difop::data, robosense::difop::tail > packet;
    };

    static double range_resolution_default() { return 0.005; }

    static double range_resolution( const difop::packet*, const msop::packet* m ) { return m ? m->range_resolution() : range_resolution_default(); }
};

} } // namespace snark { namespace robosense {
