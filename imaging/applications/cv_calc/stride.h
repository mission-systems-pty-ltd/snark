// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>
#include "../../../imaging/cv_mat/serialization.h"

namespace snark { namespace cv_calc { namespace stride {

std::string options();

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options );

} } } // namespace snark { namespace cv_calc { namespace stride {
