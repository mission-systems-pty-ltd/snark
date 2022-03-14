// This file is part of comma, a generic and flexible library
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

/// @author vsevolod vlaskine

#pragma once

#include <memory>
#include <boost/array.hpp>
#include <comma/base/types.h>
#include <comma/packed/bits.h>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/big_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>

namespace snark { namespace las {

struct version : public comma::packed::packed_struct< version, 2 >
{
    comma::packed::byte major;
    comma::packed::byte minor;
};

template < typename T >
struct xyz : public comma::packed::packed_struct< xyz< T >, sizeof( T ) * 3 >
{
    T x;
    T y;
    T z;
};

/// version 1.3-R11, see http://www.asprs.org/a/society/committees/standards/LAS_1_3_r11.pdf
struct header: public comma::packed::packed_struct< header, 227 >
{
    comma::packed::string< 4 > signature;
    comma::packed::little_endian::uint16 source_id;
    comma::packed::little_endian::uint16 global_encoding; // todo: do bit decoding
    comma::packed::little_endian::uint32 guid_1;
    comma::packed::little_endian::uint16 guid_2;
    comma::packed::little_endian::uint16 guid_3;
    comma::packed::string< 8 > guid_4;
    las::version version;
    comma::packed::string< 32 > system_id; // todo: do bit decoding
    comma::packed::string< 32 > generating_software;
    comma::packed::little_endian::uint16 file_creation_day_of_year;
    comma::packed::little_endian::uint16 file_creation_year;
    comma::packed::little_endian::uint16 header_size;
    comma::packed::little_endian::uint32 offset_to_point_data;
    comma::packed::little_endian::uint32 number_of_variable_length_records;
    comma::packed::byte point_data_format; // 0-99
    comma::packed::little_endian::uint16 point_data_record_length;
    comma::packed::little_endian::uint32 number_of_point_records;
    boost::array< comma::packed::little_endian::uint32, 5 > number_of_points_by_return;
    las::xyz< comma::packed::float64 > scale_factor;
    las::xyz< comma::packed::float64 > offset;
    comma::packed::float64 max_x;
    comma::packed::float64 min_x;
    comma::packed::float64 max_y;
    comma::packed::float64 min_y;
    comma::packed::float64 max_z;
    comma::packed::float64 min_z;
};

namespace variable_length_records
{
    struct header: public comma::packed::packed_struct< variable_length_records::header, 54 >
    {
        comma::packed::little_endian::uint16 reserved;
        comma::packed::string< 16 > user_id;
        comma::packed::little_endian::uint16 record_id;
        comma::packed::little_endian::uint16 record_length_after_header;
        comma::packed::string< 32 > description;
    };

    struct record
    {
        struct body_t
        {
            virtual ~body_t() {}
        };

        variable_length_records::header header;
        std::unique_ptr< body_t > body;
    };

    struct geo_key_directory_tag
    {
        enum { record_id = 34735 };

        struct header: public comma::packed::packed_struct< variable_length_records::geo_key_directory_tag::header, 8 >
        {
            comma::packed::little_endian::uint16 key_directory_version;
            comma::packed::little_endian::uint16 key_revision;
            comma::packed::little_endian::uint16 minor_revision;
            comma::packed::little_endian::uint16 number_of_keys;
        };

        struct key: public comma::packed::packed_struct< variable_length_records::geo_key_directory_tag::key, 8 >
        {
            comma::packed::little_endian::uint16 id;
            comma::packed::little_endian::uint16 tiff_tag_location;
            comma::packed::little_endian::uint16 count;
            comma::packed::little_endian::uint16 offset;
        };

        struct body: public variable_length_records::record::body_t
        {
            geo_key_directory_tag::header header;
            std::vector< geo_key_directory_tag::key > keys;
        };
    };

    struct geo_ascii_params_tag
    {
        enum { record_id = 34737 };

        struct body: public variable_length_records::record::body_t
        {
            std::string tag;
        };
    };
} // namespace variable_length_records

/// @todo: other point data record formats
template < unsigned int I > struct point;

struct returns { unsigned char number: 3, size: 3, scan_direction: 1, edge_of_flight_line: 1; };

template <> struct point< 0 > : public comma::packed::packed_struct< point< 0 >, 20 >
{
    typedef las::returns returns_t;
    las::xyz< comma::packed::little_endian32 > coordinates;
    comma::packed::little_endian::uint16 intensity;
    comma::packed::bits< returns_t > returns;
    comma::packed::byte classification;
    comma::packed::byte scan_angle; // -90 to 90
    comma::packed::byte user_data;
    comma::packed::little_endian::uint16 point_source_id;
};

template <> struct point< 1 > : public comma::packed::packed_struct< point< 1 >, 28 >
{
    typedef las::returns returns_t;
    las::xyz< comma::packed::little_endian32 > coordinates;
    comma::packed::little_endian::uint16 intensity;
    comma::packed::bits< returns_t > returns;
    comma::packed::byte classification;
    comma::packed::byte scan_angle; // -90 to 90
    comma::packed::byte user_data;
    comma::packed::little_endian::uint16 point_source_id;
    comma::packed::float64 gps_time;
};

struct colour : public comma::packed::packed_struct< colour, 6 >
{
    comma::packed::little_endian::uint16 red;
    comma::packed::little_endian::uint16 green;
    comma::packed::little_endian::uint16 blue;
};

template <> struct point< 2 > : public comma::packed::packed_struct< point< 2 >, 26 >
{
    typedef las::returns returns_t;
    las::xyz< comma::packed::little_endian32 > coordinates;
    comma::packed::little_endian::uint16 intensity;
    comma::packed::bits< returns_t > returns;
    comma::packed::byte classification;
    comma::packed::byte scan_angle; // -90 to 90
    comma::packed::byte user_data;
    comma::packed::little_endian::uint16 point_source_id;
    las::colour colour;
};

template <> struct point< 3 > : public comma::packed::packed_struct< point< 3 >, 34 >
{
    typedef las::returns returns_t;
    las::xyz< comma::packed::little_endian32 > coordinates;
    comma::packed::little_endian::uint16 intensity;
    comma::packed::bits< returns_t > returns;
    comma::packed::byte classification;
    comma::packed::byte scan_angle; // -90 to 90
    comma::packed::byte user_data;
    comma::packed::little_endian::uint16 point_source_id;
    comma::packed::float64 gps_time; // todo? order of gps_time and colour: las spec says: colour, gps_time, but shows in the table gps_time, colour
    las::colour colour;
};

} }  // namespace snark { namespace las {
