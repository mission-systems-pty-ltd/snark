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

#include <iostream>
#include <string>

// #define BOOST_SPIRIT_DEBUG
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/variant/recursive_wrapper.hpp>
#include <boost/container/vector.hpp>

#include <comma/base/exception.h>

namespace snark { namespace cv_mat {

namespace bitwise
{
    struct op_or  {};
    struct op_and {};
    struct op_xor {};
    struct op_not {};

    template< typename T > struct binary_op;
    template< typename T > struct unary_op;
    typedef boost::variant< std::string
                          , boost::recursive_wrapper< unary_op< op_not > >
                          , boost::recursive_wrapper< binary_op< op_and > >
                          , boost::recursive_wrapper< binary_op< op_xor > >
                          , boost::recursive_wrapper< binary_op< op_or > >
                          > expr;

    template< typename T > struct binary_op
    {
       explicit binary_op( const expr & l, const expr & r ) : oper1( l ), oper2( r ) { }
       expr oper1, oper2;
    };

    template< typename T > struct unary_op
    {
       explicit unary_op( const expr & o ) : oper1( o ) { }
       expr oper1;
    };

    template< typename I, typename O, typename C >
    struct visitor : boost::static_visitor< std::pair< boost::function< O ( I ) >, bool > >
    {
        typedef typename boost::static_visitor< std::pair< boost::function< O ( I ) >, bool > >::result_type result_type;

        visitor( const C & c ) : c_( c ) {}
        const C & c_;

        result_type operator()( const std::string & s ) const { return c_.term( s ); }

        result_type operator()( const binary_op< op_and > & b ) const {
            result_type opl = boost::apply_visitor( *this, b.oper1 );
            result_type opr = boost::apply_visitor( *this, b.oper2 );
            return c_.op_and( opl, opr );
        }
        result_type operator()( const binary_op< op_or  > & b ) const {
            result_type opl = boost::apply_visitor( *this, b.oper1 );
            result_type opr = boost::apply_visitor( *this, b.oper2 );
            return c_.op_or( opl, opr );
        }
        result_type operator()( const binary_op< op_xor > & b ) const {
            result_type opl = boost::apply_visitor( *this, b.oper1 );
            result_type opr = boost::apply_visitor( *this, b.oper2 );
            return c_.op_xor( opl, opr );
        }
        result_type operator()( const unary_op< op_not > & u )  const {
            result_type op = boost::apply_visitor( *this, u.oper1 );
            return c_.op_not( op );
        }
    };

    struct printer : boost::static_visitor< void >
    {
        printer( std::ostream & os ) : _os(os) {}
        std::ostream & _os;

        void operator()( const std::string & s ) const { _os << s; }
        void operator()( const binary_op< op_and > & b ) const { print( " & ", b.oper1, b.oper2 ); }
        void operator()( const binary_op< op_or  > & b ) const { print( " | ", b.oper1, b.oper2 ); }
        void operator()( const binary_op< op_xor > & b ) const { print( " ^ ", b.oper1, b.oper2 ); }
        void print( const std::string & op, const expr & l, const expr & r) const;
        void operator()( const unary_op< op_not > & u ) const;
    };

    inline std::ostream & operator<<( std::ostream & os, const expr & e ) {
        boost::apply_visitor( printer( os ), e );
        return os;
    }

    namespace logical
    {
        template< typename It, typename Skipper = boost::spirit::qi::space_type >
        struct parser : boost::spirit::qi::grammar< It, expr(), Skipper >
        {
            parser() : parser::base_type(expr_)
            {
                namespace qi = boost::spirit::qi;

                expr_ = or_.alias();

                or_  = ( xor_ >> "or"  >> or_  ) [ qi::_val = boost::phoenix::construct< binary_op< op_or  > >( boost::spirit::_1, boost::spirit::_2 ) ] | xor_   [ qi::_val = boost::spirit::_1 ];
                xor_ = ( and_ >> "xor" >> xor_ ) [ qi::_val = boost::phoenix::construct< binary_op< op_xor > >( boost::spirit::_1, boost::spirit::_2 ) ] | and_   [ qi::_val = boost::spirit::_1 ];
                and_ = ( not_ >> "and" >> and_ ) [ qi::_val = boost::phoenix::construct< binary_op< op_and > >( boost::spirit::_1, boost::spirit::_2 ) ] | not_   [ qi::_val = boost::spirit::_1 ];
                not_ = ( "not" > simple        ) [ qi::_val = boost::phoenix::construct<  unary_op< op_not > >( boost::spirit::_1                    ) ] | simple [ qi::_val = boost::spirit::_1 ];

                simple = ( ( '(' > expr_ > ')' ) | var_ );
                var_ = qi::lexeme[ +(qi::alnum | qi::char_("=;_,./:|+-")) ];

                BOOST_SPIRIT_DEBUG_NODE( not_ );
                BOOST_SPIRIT_DEBUG_NODE( and_ );
                BOOST_SPIRIT_DEBUG_NODE( xor_ );
                BOOST_SPIRIT_DEBUG_NODE( or_ );
                BOOST_SPIRIT_DEBUG_NODE( simple );
                BOOST_SPIRIT_DEBUG_NODE( expr_ );
                BOOST_SPIRIT_DEBUG_NODE( var_ );
            }

            private:
                boost::spirit::qi::rule< It, std::string(), Skipper > var_;
                boost::spirit::qi::rule< It, expr(), Skipper > not_, and_, xor_, or_, simple, expr_;
        };

        expr parse( const std::string & s );

    } // namespace logical

    namespace brackets
    {
        struct bracket;

        typedef boost::variant< std::string
                              , boost::recursive_wrapper< bracket >
                              > element;

        typedef boost::container::vector< element > sequence;

        struct bracket
        {
           explicit bracket( const sequence & f = sequence() ) : s_( f ) {}
           sequence s_;
        };

        struct printer : boost::static_visitor< void >
        {
            printer( std::ostream & os ) : _os(os) {}
            std::ostream & _os;

            void operator()( const std::string & s ) const { _os << s; }
            void operator()( const bracket & b ) const { _os << '('; ( *this )( b.s_ ); _os << ')'; }
            void operator()( const element & e ) const { boost::apply_visitor( *this, e ); }
            void operator()( const sequence & s ) const { for ( const auto& e : s ) { boost::apply_visitor( *this, e ); } }
        };

        inline std::ostream & operator<<( std::ostream & os, const element & l ) {
            boost::apply_visitor( printer( os ), l );
            return os;
        }

        inline std::ostream & operator<<( std::ostream & os, const sequence & s ) {
            for ( const auto & e : s ) { os << e; }
            return os;
        }

        template< typename It, typename Skipper = boost::spirit::qi::space_type >
        struct parser : boost::spirit::qi::grammar< It, sequence(), Skipper >
        {
            parser() : parser::base_type( sequence_ )
            {
                namespace qi = boost::spirit::qi;

                sequence_ = element_ >> *element_;
                element_ = ( bracket_ | var_ );
                bracket_ = '(' >> sequence_ >> ')';
                var_ = qi::lexeme[ +(qi::alnum | qi::char_("=;_,./:|+-")) ];

                BOOST_SPIRIT_DEBUG_NODE( sequence_ );
                BOOST_SPIRIT_DEBUG_NODE( element_ );
                BOOST_SPIRIT_DEBUG_NODE( bracket_ );
                BOOST_SPIRIT_DEBUG_NODE( var_ );
            }

            private:
                boost::spirit::qi::rule< It, std::string(), Skipper > var_;
                boost::spirit::qi::rule< It, bracket(), Skipper > bracket_;
                boost::spirit::qi::rule< It, element(), Skipper > element_;
                boost::spirit::qi::rule< It, sequence(), Skipper > sequence_;
        };

    } // namespace brackets

    expr parse( const std::string & s );

} // namespace bitwise

} } // namespace snark { namespace cv_mat {
