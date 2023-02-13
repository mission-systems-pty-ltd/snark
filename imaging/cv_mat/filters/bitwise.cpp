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

#include "bitwise.h"

#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace {
    using namespace snark::cv_mat::bitwise;

    struct resolver : boost::static_visitor< expr >
    {
        resolver( const std::map< std::string, expr > & m ) : m_( m ) {}
        const std::map< std::string, expr > & m_;

        expr operator()( const std::string & s ) const
        {
            for ( const auto & e : m_ )
            {
                if ( s == e.first ) { return e.second; }
                std::size_t pos = s.find( e.first );
                if ( pos == std::string::npos ) { continue; }
                if ( e.second.which() != 0 ) { COMMA_THROW( comma::exception, "in string '" << s << "' value " << e.first << " is a logical expression '" << e.second << "', not a string" ); }
                const std::string & replacement = '(' + boost::get< std::string >( e.second ) + ')';
                const std::string & news = boost::replace_all_copy( s, e.first, replacement );
                return (*this)( news );
            }
            // never resolved anything
            return expr( s );
        }

        template< typename Op >
        expr operator()( const binary_op< Op > & b ) const {
            expr opl = boost::apply_visitor( *this, b.oper1 );
            expr opr = boost::apply_visitor( *this, b.oper2 );
            return binary_op< Op >( opl, opr );
        }
        expr operator()( const unary_op< op_not > & u ) const {
            expr op = boost::apply_visitor( *this, u.oper1 );
            return unary_op< op_not >( op );
        }
    };

    struct converter : boost::static_visitor< void >
    {
        converter() : next_( 0 ), name_( converter::get_name() ), subexpressions_(), value_() {}

        void operator()( const std::string & s ) {
            if ( s == "and" ) { value_ += " and "; return; }
            if ( s == "xor" ) { value_ += " xor "; return; }
            if ( s == "or"  ) { value_ += " or " ; return; }
            if ( s == "not" ) { value_ += " not "; return; }
            value_ += s;
        }
        void operator()( const brackets::bracket & b ) {
            converter inner;
            expr sub = inner( b.s_ );
            std::string id = name_ + "_" + boost::lexical_cast< std::string >( next_++ );
            subexpressions_[ id ] = sub;
            value_ += id;
        }
        void operator()( const brackets::element & e ) { boost::apply_visitor( *this, e ); }

        expr operator()( const brackets::sequence & s ) {
            for ( const auto& e : s ) { boost::apply_visitor( *this, e ); }
            expr result = logical::parse( value_ );
            resolver r( subexpressions_ );
            return boost::apply_visitor( r, result );
        }

        private:
            unsigned int next_;
            std::string name_;
            std::map< std::string, expr > subexpressions_;
            std::string value_;

            static std::string get_name()
            {
                static boost::uuids::random_generator g_;
                return "e" + boost::algorithm::erase_all_copy( boost::uuids::to_string( g_() ), "-" );
            }
    };

} // anonymous

namespace snark { namespace cv_mat {

namespace bitwise
{
    void printer::print( const std::string & op, const expr & l, const expr & r) const
    {
        _os << "(";
        boost::apply_visitor( *this, l );
        _os << op;
        boost::apply_visitor( *this, r );
        _os << ")";
    }

    void printer::operator()( const unary_op< op_not > & u ) const
    {
        _os << "(";
        _os << "~";
        boost::apply_visitor( *this, u.oper1 );
        _os << ")";
    }

    namespace logical
    {
        expr parse( const std::string & s )
        {
            auto f( std::begin( s ) ), l( std::end( s ) );
            parser< decltype( f ) > p;
            expr result;
            bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
            if ( !ok ) { COMMA_THROW( comma::exception, "cannot parse the string '" << s << "' into logical expressions" ); }
            if ( f != l ) { COMMA_THROW( comma::exception, "string '" << s << "', unparsed remainder '" << std::string( f, l ) << "'" ); }
            return result;
        }

    } // namespace logical

    expr parse( const std::string & s )
    {
        auto f( std::begin( s ) ), l( std::end( s ) );
        brackets::parser< decltype( f ) > p;
        brackets::sequence result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        if ( !ok ) { COMMA_THROW( comma::exception, "cannot parse the string '" << s << "' into a sequence of words" ); }
        if ( f != l ) { COMMA_THROW( comma::exception, "string '" << s << "', unparsed remainder '" << std::string( f, l ) << "'" ); }
        converter c;
        return c( result );
    }

} // namespace bitwise

} } // namespace snark { namespace cv_mat {
