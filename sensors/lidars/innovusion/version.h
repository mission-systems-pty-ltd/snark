// Copyright (c) 2025 Mission Systems Pty Ltd

// use the Innovusion_VERSION cmake variable as determined by find_package()
// to work out which version of the Innovusion library we should build against

#pragma once

// we would use the INNO_SDK_V_MAJOR etc values from the SDK but they're defined
// as strings which make them useless for comparisons

#define INNOVUSION_VERSION "3.103.1.20250715023009"
#define INNOVUSION_VERSION_MAJOR 3
#define INNOVUSION_VERSION_MINOR 103
#define INNOVUSION_VERSION_PATCH 1
#define INNOVUSION_VERSION_BUILDTIME 
