// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>
#include "../../cv_mat/serialization.h"

namespace snark { namespace cv_calc { namespace enumerate {

std::string options();

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options, const snark::cv_mat::serialization::options& output_options ); // quick and dirty

} } } // namespace snark { namespace cv_calc { namespace enumerate {
