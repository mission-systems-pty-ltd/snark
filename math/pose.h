// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include "frames/transforms.h"
#include "position.h"

namespace snark { 

// todo! consolidate position and pose types
// this terrible type duplication was due too
// very unfortunate lack of communication
// when frame transforms were implemented
typedef frames::transform pose;

} // namespace snark {
