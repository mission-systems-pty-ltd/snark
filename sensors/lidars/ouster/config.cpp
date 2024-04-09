// Copyright (c) 2024 Mission Systems Pty Ltd

#include "config.h"
#include <comma/base/exception.h>

namespace snark { namespace ouster { namespace lidar { namespace config {

time_standard_t::time_standard_t( const std::string& s )
{
    if( s == "UTC" || s == "utc" ) { value = utc; }
    else if( s == "TAI" || s == "tai" ) { value = tai; }
    else { COMMA_THROW( comma::exception, "time standard '" << s << "' not supported"); }
}

time_standard_t::operator std::string() const
{
    switch( value )
    {
        case( utc ): return "UTC"; break;
        case( tai ): return "TAI"; break;
        default: return "unknown";      // quieten compiler warning
    }
}

} } } } // namespace snark { namespace ouster { namespace lidar { namespace config {
