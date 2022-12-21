// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <comma/application/command_line_options.h>
#include "error.h"

namespace snark { namespace vimba {

// maps error codes to explanation strings for output
std::string error_code_to_string( VmbError_t error_code )
{
    std::string error_str = "Error code undefined";

    switch (error_code)
    {
    case VmbErrorSuccess:        error_str = "No error"; break;
    case VmbErrorInternalFault:  error_str = "Unexpected fault in Vimba API or driver"; break;
    case VmbErrorApiNotStarted:  error_str = "VmbStartup() was not called before the current command"; break;
    case VmbErrorNotFound:       error_str = "The designated instance (camera, feature etc.) cannot be found"; break;
    case VmbErrorBadHandle:      error_str = "The given handle is not valid"; break;
    case VmbErrorDeviceNotOpen:  error_str = "Device was not opened for usage"; break;
    case VmbErrorInvalidAccess:  error_str = "Operation is invalid with the current access mode"; break;
    case VmbErrorBadParameter:   error_str = "One of the parameters was invalid (usually an illegal pointer)"; break;
    case VmbErrorStructSize:     error_str = "The given struct size is not valid for this version of the API"; break;
    case VmbErrorMoreData:       error_str = "More data was returned in a string/list than space was provided"; break;
    case VmbErrorWrongType:      error_str = "The feature type for this access function was wrong"; break;
    case VmbErrorInvalidValue:   error_str = "The value was not valid; either out of bounds or not an increment of the minimum"; break;
    case VmbErrorTimeout:        error_str = "Timeout during wait"; break;
    case VmbErrorOther:          error_str = "Other error"; break;
    case VmbErrorResources:      error_str = "Resources not available (e.g memory)"; break;
    case VmbErrorInvalidCall:    error_str = "Call is invalid in the current context (e.g callback)"; break;
    case VmbErrorNoTL:
        {
            std::ostringstream error_oss;
            error_oss << "No transport layers were found. GENICAM_GENTL64_PATH is set to \""
                      << getenv( "GENICAM_GENTL64_PATH" ) << "\". Is it correct?";
            error_str = error_oss.str();
        }
        break;
    case VmbErrorNotImplemented: error_str = "API feature is not implemented"; break;
    case VmbErrorNotSupported:   error_str = "API feature is not supported"; break;
    case VmbErrorIncomplete:     error_str = "A multiple register read or write was partially completed"; break;
    default: break;
    }
    return error_str;
}

std::string error_msg( const std::string& prologue, VmbErrorType error )
{
    std::ostringstream msg;
    if( !prologue.empty() ) { msg << prologue << ", "; }
    msg << "Error " << error << ": " << error_code_to_string( error );
    return msg.str();
}

void write_error( const std::string& prologue, VmbErrorType error )
{
    comma::say() << error_msg( prologue, error ) << std::endl;
}

} } // namespace snark { namespace vimba {
