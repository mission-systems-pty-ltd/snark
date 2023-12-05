// Copyright (c) 2011 The University of Sydney

#include <comma/base/exception.h>
#include "ratio_parser.h"

namespace snark { namespace cv_mat {

namespace ratios
{
    std::ostream & operator<<( std::ostream & o, const channel & c )
    {
        switch( c.c )
        {
            case( channel::red ):      o << "r"; break;
            case( channel::green ):    o << "g"; break;
            case( channel::blue ):     o << "b"; break;
            case( channel::alpha ):    o << "a"; break;
            default:                             break;
        }
        return o;
    }

    std::ostream & operator<<( std::ostream & o, const term & t )
    {
        switch( t.c )
        {
            case( channel::number_of_channels ): break;
            case( channel::constant ): o << t.value; break;
            default:
                if ( t.value == 1.0 ) { o << t.c; }
                else if ( t.value == -1.0 ) { o << "-" << t.c; }
                else { o << t.value << t.c; }
                break;
        }
        return o;
    }

    combination::combination()
    {
        terms.reserve( channel::number_of_channels );
        for ( int ci = channel::constant; ci != channel::number_of_channels ; ++ci )
        {
            channel::channel_types c = static_cast< channel::channel_types >( ci );
            terms.push_back( term( 0.0, c ) );
        }
    }

    combination::combination( const term & t )
    {
        terms.reserve( channel::number_of_channels );
        for ( int ci = channel::constant; ci != channel::number_of_channels ; ++ci )
        {
            channel::channel_types c = static_cast< channel::channel_types >( ci );
            terms.push_back( term( 0.0, c ) );
        }
        terms[ t.c ] = t;
    }

    void combination::update( const term & t )
    {
        std::vector< term >::iterator i = terms.begin() + ( t.c - channel::constant );
        i->value += t.value;
        if ( std::abs( i->value ) < epsilon ) { i->value = 0.0; }
    }

    size_t combination::non_zero_term_count() const
    {
        // no c++-11
        struct counter
        {
            static size_t non_zero( size_t count, const term& t ) { return t.value != 0.0 ? count + 1 : count; }
        };
        return std::accumulate( terms.begin(), terms.end(), 0, counter::non_zero );
    }

    bool combination::unity() const { return terms[0].value == 1.0 && non_zero_term_count() == 1; }

    std::string combination::to_string( ) const
    {
        std::ostringstream o;
        size_t ncount = non_zero_term_count();
        bool previous = false;
        for ( std::vector< term >::const_iterator i = terms.begin(); i != terms.end(); ++i )
        {
            if ( i->value != 0.0 || ( i == terms.begin() && ncount == 0 ) ) // constant term is always the first; optionally output zero
            {
                if ( previous ) { o << ( i->value > 0.0 ? " + " : " - " ) << abs(*i); } else { o << *i; }
                previous = true;
            }
        }
        return o.str();
    }

    void combination::print( std::ostream & o ) const
    {
        for ( std::vector< term >::const_iterator i = terms.begin(); i != terms.end(); ) { o << i->value; o << ( ++i == terms.end() ? "" : "," ); }
    }

    const double combination::epsilon = 1.0e-10;

    ratio::ratio( const std::vector< double > & n, const std::vector< double > & d )
    {
        COMMA_ASSERT( n.size() == channel::number_of_channels, "expected numerator vector of size " << channel::number_of_channels << ", got " << n.size() );
        COMMA_ASSERT( d.size() == channel::number_of_channels, "expected denominator vector of size " << channel::number_of_channels << ", got " << d.size() );
        for ( int ci = channel::constant; ci != channel::number_of_channels ; ++ci )
        {
            numerator.terms[ci].value = n[ci];
            denominator.terms[ci].value = d[ci];
        }
    }

    std::string ratio::to_string( ) const
    {
        bool divide = !denominator.unity();
        bool nbrackets = ( numerator.non_zero_term_count() > 1 ) && divide;
        bool dbrackets = ( denominator.non_zero_term_count() > 1 );
        std::string rv = ( nbrackets ? "( " : "" ) + numerator.to_string() + ( nbrackets ? " )" : "" );
        if( divide ) { rv += std::string(" / ") + ( dbrackets ? "( " : "" ) + denominator.to_string() + ( dbrackets ? " )" : "" ); }
        return rv;
    }

    std::string ratio::describe_syntax( size_t offset )
    {
        std::ostringstream o; 
        std::string w( offset, ' ' );
        o << w << "input syntax for the ratio and linear-combination operations:\n";
        w = std::string( offset + 4, ' ' );
        o << w << "the input string specifies a ratio of linear combinations of signal channels in algebraic form\n";
        o << w << "\n";
        o << w << "one of the following notations can be used:\n";
        o << w << "        ( linear combination ) / ( linear combination )\n";
        o << w << "    here 'linear combination' is in the form Ar + Bg + Cb + Da + F, where A, B, C, D, F\n";
        o << w << "    are floating-point constants and r, g, b, a are symbolic channel names; the multiplication\n";
        o << w << "    sign '*' is allowed between the constant and channel name; something like 3r, 3*r, r*3 are allowed and mean the same\n";
        o << w << "    the F term gives a fixed offset to be added to the output\n";
        o << w << "        term / ( linear combination )\n";
        o << w << "    here 'term' is either A or Ac, where A is a constant and c is one of the channel names\n";
        o << w << "    for a single term, surrounding brackets are optional, 'term' is the same as '(term)'\n";
        o << w << "        ( linear combination ) / term\n";
        o << w << "        term / term\n";
        o << w << "        linear combination\n";
        o << w << "    here the linear combination becomes the ratio numerator, while the denominator is set to 1\n";
        o << w << "    brackets are optional around a numerator-only linear combination\n";
        o << w << "\n";
        o << w << "    the input is parsed into two lists of coefficients for the numerator and denominator\n";
        o << w << "\n";
        o << w << "examples (outputs are shown after --> arrows as ratios of comma-separated lists of coefficients,\n";
        o << w << "for the numerator and denominator)\n";
        o << w << "    ratios are composed of linear combinations in the natural way\n";
        o << w << "        ( a +2*b - 3r ) / ( r + b )  -->  0,-3,0,2,1 / 0,1,0,1,0\n";
        o << w << "        a / ( r + b )                -->  0,0,0,0,1 / 0,1,0,1,0\n";
        o << w << "        (3a + 4*b) / g               -->  0,0,0,4,3 / 0,0,1,0,0\n";
        o << w << "        3a / g                       -->  0,0,0,0,3 / 0,0,1,0,0\n";
        o << w << "    terms and signs between them may be separated by spaces\n";
        o << w << "    brackets are optional for single terms and are mandatory for multi-term linear combinations\n";
        o << w << "    with one exception, numerator-only input\n";
        o << w << "        3a + b / g                   -->  error!\n";
        o << w << "        3a + b                       -->  0,0,0,1,3 / 1,0,0,0,0\n";
        o << w << "    multiplication signs are optional and multipliers of 1 or -1 can be omitted\n";
        o << w << "        ( 2*b - a ) / 2              -->  0,0,0,2,-1 / 2,0,0,0,0\n";
        o << w << "    leading '+' sign is ignored; arbitrary floating-point notation is supported\n";
        o << w << "        +2*b - 1.3e-4r               -->  0,-1.3e-4,0,2,0 / 1,0,0,0,0\n";
        o << w << "    order of channel terms does not matter\n";
        o << w << "        1 + r - g + b                -->  1,1,-1,1,0 / 1,0,0,0,0\n";
        o << w << "        b - g + 1 + r                -->  1,1,-1,1,0 / 1,0,0,0,0\n";
        o << w << "\n";
        o << w << "for the linear-combination operation, only the numerator part is expected without brackets\n";
        o << w << "examples (outputs are shown after --> arrows as ratios of comma-separated lists of coefficients)\n";
        o << w << "    -r +2g - b                       -->  0,-1,2,-1,0\n";
        o << w << "\n";
        o << w << "ratios and linear combinations are cumulative, i.e., repeated terms for the same channels\n";
        o << w << "are accumulated:\n";
        o << w << "    -r +2r - r                       -->  0,0,0,0,0\n";
        o << w << "\n";
        o << w << "multiple ratio or linear combination expressions may be specified separated by commas;\n";
        o << w << "each of the expressions is applied to calculate one of the output image channels, the order\n";
        o << w << "of expressions corresponding to the order of channels; expressions are parsed as explained above;\n";
        o << w << "it is possible to use either ratio or linear combination syntax for different expressions;\n";
        o << w << "if an expression is left undefined (empty), such as in '-r+g,,r+g', the empty string is interpreted\n";
        o << w << "as a channel pass-through; e.g., in case of the string above, the output channel 1 (g) is copied\n";
        o << w << "verbatim from the input image.\n";
        return o.str();
    }

    // explicit instantiations
    template class rules< std::string::const_iterator >;
    template class rules< std::string::iterator >;

} // namespace ratios

} }  // namespace snark { namespace cv_mat {
