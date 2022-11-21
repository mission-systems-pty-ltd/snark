// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#ifndef SNARK_SENSORS_VIMBA_ERROR_H_
#define SNARK_SENSORS_VIMBA_ERROR_H_

#include <string>
#include <VimbaC/Include/VmbCommonTypes.h>

namespace snark { namespace vimba {

std::string error_code_to_string( VmbError_t error_code );
std::string error_msg( const std::string& prologue, VmbErrorType error );
void write_error( const std::string& prologue, VmbErrorType error );

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_ERROR_H_
