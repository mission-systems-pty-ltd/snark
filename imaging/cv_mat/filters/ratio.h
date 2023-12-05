// Copyright (c) 2011 The University of Sydney
// All rights reserved.

#pragma once

#include <iostream>
#include <string>
#include <boost/bind.hpp> // #include <boost/bind/bind.hpp> // somehow, boost::spirit trips over it
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix.hpp>

namespace snark { namespace cv_mat { namespace ratios {
    
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phoenix = boost::phoenix;

struct channel
{
    enum channel_types { constant = 0, red, green, blue, alpha, number_of_channels };
    channel_types c;
    channel( channel_types c_ = number_of_channels ) : c( c_ ) {}
    channel & operator=( channel_types c_ ) { c = c_; return *this; }
    operator channel_types() const { return c; }
};

std::ostream & operator<<( std::ostream & o, const channel & c );

struct term
{
    double value;
    channel c;
    term( double v_ = 0.0, channel c_ = channel::number_of_channels ) : value( v_ ), c( c_ ) {}
};

inline term abs( const term & t ) { return term( std::abs( t.value ), t.c ); }

std::ostream & operator<<( std::ostream & o, const term & t );

struct combination
{
    const static double epsilon;

    explicit combination();
    combination( const term & t );
    void update( const term & t );
    size_t non_zero_term_count() const;
    bool unity() const;
    std::string to_string( ) const;
    void print( std::ostream & o ) const;

    std::vector< term > terms;
};

inline std::ostream & operator<<( std::ostream & o, const combination & c ) { c.print( o ); return o; }

struct ratio
{
    combination numerator;
    combination denominator;

    explicit ratio() { denominator.terms[0].value = 1.0; }
    explicit ratio( const std::vector< double > & n, const std::vector< double > & d );

    static size_t number_of_channels() { return channel::number_of_channels; }
    std::string to_string( ) const;
    static std::string describe_syntax( size_t offset = 0 );
};

inline std::ostream& operator<<( std::ostream & o, const ratio & r ) { o << r.numerator << "/" << r.denominator; return o; }

template< typename Iterator >
struct rules
{
    rules();

    qi::rule< Iterator, ratios::channel(), ascii::space_type > channel_;
    qi::rule< Iterator, ratios::term(), ascii::space_type > term_;
    qi::rule< Iterator, ratios::combination(), ascii::space_type > combination_;
    qi::rule< Iterator, ratios::ratio(), ascii::space_type > ratio_;
};

template< typename Iterator, typename What >
struct parser : qi::grammar< Iterator, What(), ascii::space_type >
{
    parser( const qi::rule< Iterator, What(), ascii::space_type > & start ) : parser::base_type( start ) {}
};

typedef std::vector< std::pair< double, double > > coefficients;

} } }  // namespace snark { namespace cv_mat { namespace ratios {
