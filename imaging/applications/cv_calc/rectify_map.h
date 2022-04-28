// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>

namespace snark { namespace cv_calc { namespace rectify_map {

std::string options();

int run( const comma::command_line_options& options );

} } } // namespace snark { namespace cv_calc { namespace rectify_map {
