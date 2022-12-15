// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2022 Mission Systems Pty Ltd

#include "types.h"

namespace snark { namespace vimba {

const char* VmbInterfaceType_to_string( VmbInterfaceType type )
{
    switch( type )
    {
        case VmbInterfaceUnknown:  return "Unknown";      // Interface is not known to this version of the API
        case VmbInterfaceFirewire: return "Firewire";     // 1394
        case VmbInterfaceEthernet: return "Ethernet";     // GigE
        case VmbInterfaceUsb:      return "Usb";          // USB 3.0
        case VmbInterfaceCL:       return "CL";           // Camera Link
        case VmbInterfaceCSI2:     return "CSI2";         // CSI-2
        default:                   return "Unknown";
    }
}

} } // namespace snark { namespace vimba {
