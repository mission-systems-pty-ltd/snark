// Copyright (c) 2011 The University of Sydney
// All rights reserved.

#pragma once

#include "ratio.h"

namespace snark{ namespace cv_mat {

namespace ratios
{
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;
    namespace phoenix = boost::phoenix;

    template< typename Iterator >
    rules< Iterator >::rules()
    {
        using qi::double_;
        using qi::_1;
        using qi::lit;
        using phoenix::bind;
        using qi::_val;
        using qi::eps;

        channel_ = eps[ _val = ratios::channel() ] >> ( lit('r')[ _val = channel::red ] | lit('g')[ _val = channel::green ] | lit('b')[ _val = channel::blue ] | lit('a')[ _val = channel::alpha ] );
        term_ = eps[ _val = ratios::term( 1.0, channel::constant ) ] >>
            (
                    double_[ bind( &term::value, _val ) = _1 ] >> -lit('*') >> channel_[ bind( &term::c, _val ) = _1 ]
                |   channel_[ bind( &term::c, _val ) = _1 ] >> lit('*') >> double_[ bind( &term::value, _val ) = _1 ]
                |   channel_[ bind( &term::c, _val ) = _1 ]
                |   lit('+')[ bind( &term::value, _val ) = 1 ] >> -( double_[ bind( &term::value, _val ) *= _1 ] >> -lit('*') ) >> channel_[ bind( &term::c, _val ) = _1 ]
                |   lit('-')[ bind( &term::value, _val ) = -1 ] >> -( double_[ bind( &term::value, _val ) *= _1 ] >> -lit('*') ) >> channel_[ bind( &term::c, _val ) = _1 ]
                |   lit('+')[ bind( &term::value, _val ) = 1 ] >> double_[ bind( &term::value, _val ) *= _1 ]
                |   lit('-')[ bind( &term::value, _val ) = -1 ] >> double_[ bind( &term::value, _val ) *= _1 ]
                |   double_[ bind( &term::value, _val ) = _1 ]
            );
        combination_ = eps[ _val = ratios::combination() ] >> +( term_[ bind( &combination::update, _val, _1 ) ] );
        ratio_ = eps[ _val = ratios::ratio() ] >>
            (
                    lit('(') >> combination_[ bind( &ratio::numerator, _val ) = _1 ] >> lit(')') >> lit('/') >> lit('(') >> combination_[ bind( &ratio::denominator, _val ) = _1 ] >> lit(')')
                |   lit('(') >> combination_[ bind( &ratio::numerator, _val ) = _1 ] >> lit(')') >> lit('/') >> term_[ bind( &ratio::denominator, _val ) = _1 ]
                |   term_[ bind( &ratio::numerator, _val ) = _1 ] >> lit('/') >> lit('(') >> combination_[ bind( &ratio::denominator, _val ) = _1 ] >> lit(')')
                |   term_[ bind( &ratio::numerator, _val ) = _1 ] >> lit('/') >> term_[ bind( &ratio::denominator, _val ) = _1 ]
                |   lit('(') >> combination_[ bind( &ratio::numerator, _val ) = _1 ] >> lit(')')
                |   combination_[ bind( &ratio::numerator, _val ) = _1 ]
            );
    }

} // namespace ratios

} }  // namespace snark { namespace cv_mat {
