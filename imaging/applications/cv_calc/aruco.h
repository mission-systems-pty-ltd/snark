// Copyright (c) 2026 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>
#include "../../../imaging/cv_mat/serialization.h"

namespace snark { namespace cv_calc { namespace aruco {
    
namespace detection {

std::string options();

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options );

} // namespace detection {

namespace localization {

std::string options();

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options );

} // namespace detection {

} } } // namespace snark { namespace cv_calc { namespace aruco {
