// Copyright (c) 2018 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <comma/application/command_line_options.h>

namespace snark { namespace points_calc { namespace triangles {
    
namespace area {
    
struct traits
{
    static std::string input_fields();

    static std::string input_format();
        
    static std::string output_fields();

    static std::string output_format();

    static std::string usage();
        
    static int run( const comma::command_line_options& options );
};

} // namespace discretise {

namespace discretise {
    
struct traits
{
    static std::string input_fields();

    static std::string input_format();
        
    static std::string output_fields();

    static std::string output_format();

    static std::string usage();
        
    static int run( const comma::command_line_options& options );
};

} // namespace discretise {
    
} } } // namespace snark { namespace points_calc { namespace triangles {
