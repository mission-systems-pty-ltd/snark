// Copyright (c) 2011 The University of Sydney
// All rights reserved.

#include "../../../../../imaging/cv_mat/filters/ratio.h"
#include <gtest/gtest.h>
#include <boost/assign.hpp>
#include <iostream>

using namespace snark::cv_mat;
using boost::spirit::ascii::space;
typedef std::string::const_iterator iterator_type;

namespace {

template< typename T >
bool process( const ratios::parser< iterator_type, T > & parser
            , const std::string & input
            , T & result )
{
    iterator_type begin = input.begin();
    iterator_type end = input.end();
    bool status = phrase_parse( begin, end, parser, space, result );
    return status && ( begin == end );
}

void verify( const ratios::combination & c, const std::vector< double > & expected )
{
    EXPECT_EQ( expected.size(), ratios::channel::number_of_channels ); // self-check
    for ( int ci = ratios::channel::constant; ci != ratios::channel::number_of_channels ; ++ci )
    {
        EXPECT_EQ( c.terms[ci].value, expected[ci] );
    }
}

void verify( const ratios::ratio & r, const std::vector< double > & expected_numerator, const std::vector< double > & expected_denominator )
{
    verify( r.numerator, expected_numerator );
    verify( r.denominator, expected_denominator );
}

} // namespace anonymous

TEST( ratios, term )
{
    ratios::rules< iterator_type > rules;
    ratios::parser< iterator_type, ratios::term > parser( rules.term_ );
    ratios::term v;

    {
        std::string input = "3b";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, 3.0 );
        EXPECT_EQ( v.c, ratios::channel::blue );
    }

    {
        std::string input = "-2* g";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, -2.0 );
        EXPECT_EQ( v.c, ratios::channel::green );
    }

    {
        std::string input = "r";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, 1.0 );
        EXPECT_EQ( v.c, ratios::channel::red );
    }

    {
        std::string input = "-2";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, -2.0 );
        EXPECT_EQ( v.c, ratios::channel::constant );
    }

    {
        std::string input = "- 2";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, -2.0 );
        EXPECT_EQ( v.c, ratios::channel::constant );
    }

    {
        std::string input = "2";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, 2.0 );
        EXPECT_EQ( v.c, ratios::channel::constant );
    }

    {
        std::string input = "+2";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, 2.0 );
        EXPECT_EQ( v.c, ratios::channel::constant );
    }

    {
        std::string input = "2h";
        EXPECT_FALSE( process( parser, input, v ) );
    }

    {
        std::string input = "2 + g";
        EXPECT_FALSE( process( parser, input, v ) );
    }
}

TEST( ratios, combination )
{
    ratios::rules< iterator_type > rules;
    ratios::parser< iterator_type, ratios::combination > parser( rules.combination_ );
    ratios::combination v;

    {
        std::string input = "3b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(0.0)(0.0)(3.0)(0.0) );
        EXPECT_EQ( v.to_string(), "3b" );
    }
    {
        std::string input = "b*3";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(0.0)(0.0)(3.0)(0.0) );
        EXPECT_EQ( v.to_string(), "3b" );
    }

    {
        std::string input = "-3.5b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(0.0)(0.0)(-3.5)(0.0) );
        EXPECT_EQ( v.to_string(), "-3.5b" );
    }

    {
        std::string input = "1 + r - 3*a";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(1.0)(1.0)(0.0)(0.0)(-3.0) );
        EXPECT_EQ( v.to_string(), "1 + r - 3a" );
    }

    {
        std::string input = "g - a + b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(0.0)(1.0)(1.0)(-1.0) );
        EXPECT_EQ( v.to_string(), "g + b - a" );
    }

    {
        std::string input = "g - a + b/2";
        EXPECT_FALSE( process( parser, input, v ) );
    }

    {
        std::string input = ".0g";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "0" );
    }

    {
        std::string input = "2 + g";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(2.0)(0.0)(1.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "2 + g" );
    }

    {
        std::string input = "g + 2";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(2.0)(0.0)(1.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "2 + g" );
    }

    {
        std::string input = "r + 2r - g + 3.1g";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(3.0)(2.1)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "3r + 2.1g" );
    }
}

TEST( ratios, ratio )
{
    ratios::rules< iterator_type > rules;
    ratios::parser< iterator_type, ratios::ratio > parser( rules.ratio_ );
    ratios::ratio v;

    {
        v = ratios::ratio( boost::assign::list_of(0.0)(1.0)(0.0)(0.0)(0.0), boost::assign::list_of(2.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "r / 2" );
    }

    {
        v = ratios::ratio( boost::assign::list_of(0.0)(1.0)(-1.0)(0.0)(0.0), boost::assign::list_of(2.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "( r - g ) / 2" );
    }

    {
        v = ratios::ratio( boost::assign::list_of(0.0)(1.0)(0.0)(-3.0)(0.0), boost::assign::list_of(1.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "r - 3b" );
    }

    {
        std::string input = "r - b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(-1.0)(0.0), boost::assign::list_of(1.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "r - b" );
    }

    {
        std::string input = "(r - b)";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(-1.0)(0.0), boost::assign::list_of(1.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "r - b" );
    }

    {
        std::string input = "(r - b)/(2.1g + a)";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(-1.0)(0.0), boost::assign::list_of(0.0)(0.0)(2.1)(0.0)(1.0) );
        EXPECT_EQ( v.to_string(), "( r - b ) / ( 2.1g + a )" );
    }

    {
        std::string input = "r/(2.1g + a)";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(0.0)(0.0), boost::assign::list_of(0.0)(0.0)(2.1)(0.0)(1.0) );
        EXPECT_EQ( v.to_string(), "r / ( 2.1g + a )" );
    }

    {
        std::string input = "r / 2.1 * g";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(0.0)(0.0), boost::assign::list_of(0.0)(0.0)(2.1)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "r / 2.1g" );
    }

    {
        std::string input = "( r + b ) / 0.1*g";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(1.0)(0.0), boost::assign::list_of(0.0)(0.0)(0.1)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "( r + b ) / 0.1g" );
    }

    {
        std::string input = "( r + b ) / 2";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(1.0)(0.0), boost::assign::list_of(2.0)(0.0)(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "( r + b ) / 2" );
    }

    {
        std::string input = "( r + b ) / ( g )";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(0.0)(1.0)(0.0)(1.0)(0.0), boost::assign::list_of(0.0)(0.0)(1.0)(0.0)(0.0) );
        EXPECT_EQ( v.to_string(), "( r + b ) / g" );
    }

    {
        std::string input = "( 10 + r ) / b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(10.0)(1.0)(0.0)(0.0)(0.0), boost::assign::list_of(0.0)(0.0)(0.0)(1.0)(0.0) );
        EXPECT_EQ( v.to_string(), "( 10 + r ) / b" );
    }

    {
        std::string input = "( r + 10 ) / b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(10.0)(1.0)(0.0)(0.0)(0.0), boost::assign::list_of(0.0)(0.0)(0.0)(1.0)(0.0) );
        EXPECT_EQ( v.to_string(), "( 10 + r ) / b" );
    }

    {
        std::string input = "( r + 10 - r ) / ( b + 1.5b )";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of(10.0)(0.0)(0.0)(0.0)(0.0), boost::assign::list_of(0.0)(0.0)(0.0)(2.5)(0.0) );
        EXPECT_EQ( v.to_string(), "10 / 2.5b" );
    }
}

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
