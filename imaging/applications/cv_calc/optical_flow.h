// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>
#include "../../../imaging/cv_mat/serialization.h"

namespace snark { namespace cv_calc { namespace optical_flow {

namespace farneback {

std::string options();

int run( const comma::command_line_options& options );

} // namespace farneback {

} } } // namespace snark { namespace cv_calc { namespace optical_flow {
