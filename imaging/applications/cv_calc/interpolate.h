// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>
#include "../../../imaging/cv_mat/serialization.h"

namespace snark { namespace cv_calc { namespace interpolate {

std::string options();

int run( const comma::command_line_options& options );

} } } // namespace snark { namespace cv_calc { namespace interpolate {
