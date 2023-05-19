// Copyright (c) 2011 The University of Sydney

#pragma once

namespace snark { namespace graphics { namespace qt3d {

struct camera_options
{
    bool   orthographic;
    double field_of_view;
    bool   z_is_up;

    camera_options( bool orthographic = false, double field_of_view = 45, bool z_is_up = false )
        : orthographic( orthographic )
        , field_of_view( field_of_view )
        , z_is_up( z_is_up )
    {
    }
};

} } } // namespace snark { namespace graphics { namespace qt3d {
