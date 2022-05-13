// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2018 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

#include <sstream>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../../../visiting/eigen.h"
#include "lines_nearest.h"

namespace snark { namespace points_calc { namespace lines_nearest {

typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > line_t;
typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > output_t;
typedef std::pair< line_t, line_t > lines_t;

std::string traits::usage()
{
    std::ostringstream oss;
    oss
        << "    lines-nearest: TODO" << std::endl
        << "        read two lines from stdin, for each line append the nearest point to the other line" << std::endl
        << std::endl
        << "        input fields: " << comma::join( comma::csv::names< lines_t >( true ), ',' ) << std::endl
        << std::endl
        << "        options:" << std::endl
        << "            --discard-collinear: don't output records when lines are parallel" << std::endl
        << "            --first=<" << comma::join( comma::csv::names< line_t >(true), ',') <<  ">: default values for first line" << std::endl
        << "            --second=<" << comma::join( comma::csv::names< line_t >(true), ',') <<  ">: default values for second line" << std::endl
        << std::endl
        << "        example:" << std::endl
        << "            todo" << std::endl
        << std::endl;
    return oss.str();    
}

std::string traits::input_fields() { return comma::join( comma::csv::names< lines_t >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< lines_t >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output_t >( true ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output_t >(); }

int traits::run( const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    bool discard_collinear = options.exists( "--discard-collinear" );
    line_t first_default = comma::csv::ascii< line_t >().get( options.value< std::string >( "--first", "0,0,0,0,0,0" ) );
    line_t second_default = comma::csv::ascii< line_t >().get( options.value< std::string >( "--second", "0,0,0,0,0,0" ) );
    comma::csv::input_stream< lines_t > istream( std::cin, csv, std::make_pair( first_default, second_default ) );
    comma::csv::output_stream < output_t > ostream( std::cout, csv.binary(), false, csv.flush );
    comma::csv::tied< lines_t, output_t > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const lines_t* r = istream.read();
        if( !r ) { break; }
        const Eigen::Vector3d& f = ( r->first.second - r->first.first ).normalized();
        const Eigen::Vector3d& s = ( r->second.second - r->second.first ).normalized();
        if( comma::math::equal( f.dot( s ), f.norm() * s.norm() ) )
        {
            if( discard_collinear ) { continue; }
            std::cerr << "points-calc: lines-nearest: got collinear lines (" << r->first.first.transpose() << " , " << r->first.second.transpose() << ") and (" << r->second.first.transpose() << " , " << r->second.second.transpose() << "), please use --discard collinear to discard" << std::endl;
            return 1;
        }
        const Eigen::Vector3d& m = s.cross( f ).normalized();
        const Eigen::Vector3d& n = s.cross( m ).normalized();
        const Eigen::Vector3d& d = r->second.first - r->first.first;
        const Eigen::Vector3d& a = r->first.first + f * n.dot( d ) / n.dot( f );
        const Eigen::Vector3d& b = a + m * m.dot( d );
        tied.append( output_t( a, b ) );
    }
    return 0;
}

} } } // namespace snark { namespace points_calc { namespace lines_nearest {
