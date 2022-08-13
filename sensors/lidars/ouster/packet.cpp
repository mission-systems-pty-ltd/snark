// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020,2022 Mission Systems Pty Ltd

#include "packet.h"
#include <comma/application/verbose.h>
#include <comma/base/exception.h>

namespace snark { namespace ouster { namespace lidar {

beam_angle_lut_t get_beam_angle_lut( const config::beam_intrinsics_t& beam_intrinsics )
{
    beam_angle_lut_t beam_angle_lut;

    comma::verbose << "loading instrinsics for " << beam_intrinsics.beam_altitude_angles.size() << " beams" << std::endl;

    if( beam_intrinsics.beam_altitude_angles.size() != beam_intrinsics.beam_azimuth_angles.size() )
    {
        COMMA_THROW( comma::exception, "expected the same number of intrinsic altitude angles and azimuth angles. Got "
                                       << beam_intrinsics.beam_altitude_angles.size() << " altitude angles and "
                                       << beam_intrinsics.beam_azimuth_angles.size() << " azimuth angles" );
    }

    for( unsigned int i = 0; i < beam_intrinsics.beam_altitude_angles.size(); i++ )
    {
        beam_angle_lut.push_back( beam_angle_lut_entry( beam_intrinsics.beam_altitude_angles[i] * 2 * M_PI / 360.0
                                                      , beam_intrinsics.beam_azimuth_angles[i] * 2 * M_PI / 360.0 ));
    }
    return beam_angle_lut;
}

} } } // namespace snark { namespace ouster { namespace lidar
