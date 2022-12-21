// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#pragma once

#include "camera.h"

namespace comma { namespace visiting {

template <>
struct traits< snark::vimba::ptp_status >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::vimba::ptp_status& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "use_ptp", p.use_ptp );
        std::vector<char> buf(20,'\0');
        std::memcpy(&buf[0],p.value.data(),std::min(p.value.size()+1,buf.size()));
        v.apply( "value", buf );
    }
};

} } // namespace comma { namespace visiting {

