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

/// @author vsevolod vlaskine

#pragma once

#include <vector>
#include <comma/visiting/traits.h>
#include "packets.h"

namespace comma { namespace visiting {

template <> struct traits< snark::las::version >
{
    template< typename K, typename V > static void visit( const K&, const snark::las::version& t, V& v ) // todo
    {
        // a workaround for some systems
        // todo: remove once deprecated (see the following compilation error)
        //
        // In file included from snark/sensors/lidars/las/applications/las-to-csv.cpp:42:0:
        // snark/sensors/lidars/las/applications/../traits.h:44:13: warning: In the GNU C Library, "major" is defined
        //  by <sys/sysmacros.h>. For historical compatibility, it is
        //  currently defined by <sys/types.h> as well, but we plan to
        //  remove this soon. To use "major", include <sys/sysmacros.h>
        //  directly. If you did not intend to use a system-defined macro
        //  "major", you should undefine it after including <sys/types.h>.
        //          v.apply( "major", t.major() ); // todo: somehow it does not compile
        //              ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #ifdef major
        #undef major
        #endif
        v.apply( "major", t.major() ); // todo: somehow it does not compile
        v.apply( "minor", t.minor() ); // todo: somehow it does not compile
    }
};

template < typename T > struct traits< snark::las::xyz< T > >
{
    template< typename K, typename V > static void visit( const K&, const snark::las::xyz< T >& t, V& v )
    {
        v.apply( "x", t.x() );
        v.apply( "y", t.y() );
        v.apply( "z", t.z() );
    }
};

template <> struct traits< snark::las::header >
{
    template< typename K, typename V > static void visit( const K&, const snark::las::header& t, V& v )
    {
        v.apply( "signature", t.signature() );
        v.apply( "source_id", t.source_id() );
        v.apply( "global_encoding", t.global_encoding() ); // todo: do bit decoding
        v.apply( "guid_1", t.guid_1() );
        v.apply( "guid_2", t.guid_2() );
        v.apply( "guid_3", t.guid_3() );
        v.apply( "guid_4", t.guid_4() );
        v.apply( "version", t.version );
        v.apply( "system_id", t.system_id() ); // todo: do bit decoding
        v.apply( "generating_software", t.generating_software() );
        v.apply( "file_creation_day_of_year", t.file_creation_day_of_year() );
        v.apply( "file_creation_year", t.file_creation_year() );
        v.apply( "header_size", t.header_size() );
        v.apply( "offset_to_point_data", t.offset_to_point_data() );
        v.apply( "number_of_variable_length_records", t.number_of_variable_length_records() );
        v.apply( "point_data_format", t.point_data_format() ); // 0-99
        v.apply( "point_data_record_length", t.point_data_record_length() );
        v.apply( "number_of_point_records", t.number_of_point_records() );
        std::vector< comma::uint32 > s( t.number_of_points_by_return.size() ); // quick and dirty
        for( unsigned int i = 0; i < s.size(); s[i] = t.number_of_points_by_return[i](), ++i );
        v.apply( "number_of_points_by_return", s ); // quick and dirty
        v.apply( "scale_factor", t.scale_factor );
        v.apply( "offset", t.offset );
        v.apply( "max_x", t.max_x() );
        v.apply( "min_x", t.min_x() );
        v.apply( "max_y", t.max_y() );
        v.apply( "min_y", t.min_y() );
        v.apply( "max_z", t.max_z() );
        v.apply( "min_z", t.min_z() );
    }
};

} } // namespace comma { namespace visiting {
