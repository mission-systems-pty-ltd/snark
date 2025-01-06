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
#include <vector>
#include <unordered_map>
#include <iostream>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>

struct shuffle
{
    struct field
    {
        std::string name;
        unsigned int offset{0};
        unsigned int size{0};
        field( const std::string& name, unsigned offset, unsigned size = 0 ) : name( name ), offset( offset ), size( size ) {}
    };
    std::vector< field > fields;
    char csv_delimiter{','};

    shuffle( const comma::csv::options& csv, const std::string& shuffle_fields, const std::string& array_sizes )
        : csv_delimiter( csv.delimiter )
    {
        std::unordered_map<std::string,unsigned> array_sizes_map;
        for(const auto& s : comma::split(array_sizes,',')) //make array size map from input string
        {
            const auto& v = comma::split( s, '=' );
            COMMA_ASSERT( v.size() == 2, "expected <name>=<size> for array size, got: '" << s << "'" );
            array_sizes_map.insert( std::make_pair( v[0], boost::lexical_cast< unsigned int >( v[1] ) ) );
        }
        std::vector<field> input_fields;
        unsigned int index{0};
        for( const std::string& f: comma::split( csv.fields, ',' ) ) //calculate input fields offset,size
        {
            unsigned int size = 1;
            auto it = f.empty() ? array_sizes_map.cend() : array_sizes_map.find(f);
            if( it != array_sizes_map.cend() ) { size = it->second; }
            if( csv.binary() )
            {
                unsigned bin_offset=csv.format().offset(index).offset;
                unsigned bin_size=0;
                for(unsigned i=0;i<size;i++) { bin_size+=csv.format().offset(index+i).size; }
                input_fields.push_back(field(f,bin_offset,bin_size));
            }
            else
            {
                input_fields.push_back( field( f, index, size ) );
            }
            index += size;
        }
        for( const std::string& s : comma::split( shuffle_fields, ',' ) ) //copy to shuffle fields
        {
            COMMA_ASSERT( !s.empty(), "shuffle field name cannot be empty" );
            const auto& it = std::find_if( input_fields.cbegin(), input_fields.cend(), [&](const field& f) { return f.name==s; } );
            //or add trailing fields?
            COMMA_ASSERT( it != input_fields.cend(), "shuffle field name '" << s << "' not found in input fields" );
            fields.push_back(*it);
        }
    }

    void binary_write( const char* buf, std::size_t size, std::ostream& os )
    {
        for( const field& f : fields ) { os.write( &buf[f.offset], f.size ); }
    }

    void ascii_write( const std::vector< std::string >& v, std::ostream& os )
    {
        std::string d;
        for( const shuffle::field& f : fields )
        {
            for( unsigned i = 0; i < f.size; ++i, d = csv_delimiter ) { os << d << v[ f.offset + i ]; }
        }
    }
};

template < typename S, typename T >
struct shuffle_tied
{
    const comma::csv::input_stream< S >& is;
    comma::csv::output_stream< T >& os;
    std::unique_ptr< ::shuffle > shuffle;

    shuffle_tied( const comma::csv::input_stream< S >& i
                , comma::csv::output_stream< T >& o
                , const comma::csv::options& csv
                , const boost::optional<std::string>& shuffle_fields
                , const std::string& array_sizes )
        : is( i )
        , os( o )
    {
        if( shuffle_fields ) { shuffle.reset( new ::shuffle(csv,*shuffle_fields,array_sizes  )); }
    }
    
    void append( const T& data )
    {
        std::ostream& ostream = std::cout; // std::ostream& ostream=is.is_binary()?os.binary().os_:os.ascii().os_;
        if( is.is_binary())
        {
            if( shuffle ) { shuffle->binary_write( is.binary().last(), is.binary().size(), ostream ); }
            else { ostream.write( is.binary().last(), is.binary().size() ); }
            os.write( data );
        }
        else
        {
            if( shuffle ) { shuffle->ascii_write( is.ascii().last(), ostream ); }
            else { ostream << comma::join( is.ascii().last(), os.ascii().ascii().delimiter() ); }
            static std::string sbuf;
            os.ascii().ascii().put( data, sbuf );
            ostream << os.ascii().ascii().delimiter() << sbuf << std::endl;
        }
    }
};

