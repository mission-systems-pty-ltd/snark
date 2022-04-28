// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>

namespace snark { namespace cv_calc { namespace equirectangular_map {

std::string options();

int run( const comma::command_line_options& options );

} } } // namespace snark { namespace cv_calc { namespace equirectangular_map {
