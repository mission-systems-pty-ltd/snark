#pragma once

// If these are not defined, ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH
// you must define them in your CMakeLists.txt
// ROS2 does not define the same macros as ros1 so we need to define them ourselves.
#ifndef ROS_VERSION_MAJOR
#error ROS_VERSION_MAJOR not defined. If ROS1, source setup.bash, if ros2, then add add_compile_definitions( ROS_VERSION_MAJOR=XX ),add_compile_definitions( ROS_VERSION_MINOR=XX ),add_compile_definitions( ROS_VERSION_PATCH=XX ) to your CMakeLists.txt. Where; ardent={2,1,0}, bouncy={2,2,0}, crystal={2,3,0} etc. 
#endif
#ifndef ROS_VERSION_MINOR
#error ROS_VERSION_MINOR not defined. If ROS1, source setup.bash, if ros2, then add add_compile_definitions( ROS_VERSION_MAJOR=XX ),add_compile_definitions( ROS_VERSION_MINOR=XX ),add_compile_definitions( ROS_VERSION_PATCH=XX ) to your CMakeLists.txt. Where; ardent={2,1,0}, bouncy={2,2,0}, crystal={2,3,0} etc. 
#endif
#ifndef ROS_VERSION_PATCH
#error ROS_VERSION_PATCH not defined. If ROS1, source setup.bash, if ros2, then add add_compile_definitions( ROS_VERSION_MAJOR=XX ),add_compile_definitions( ROS_VERSION_MINOR=XX ),add_compile_definitions( ROS_VERSION_PATCH=XX ) to your CMakeLists.txt. Where; ardent={2,1,0}, bouncy={2,2,0}, crystal={2,3,0} etc. 
#endif

// Copied from ROS1 common.h header files so we can use version comparisions between the two versions.
#define ROS_VERSION_COMBINED(major, minor, patch) (((major) << 20) | ((minor) << 10) | (patch))
#define ROS_VERSION ROS_VERSION_COMBINED(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH)

#define ROS_VERSION_GE(major1, minor1, patch1, major2, minor2, patch2) (ROS_VERSION_COMBINED(major1, minor1, patch1) >= ROS_VERSION_COMBINED(major2, minor2, patch2))
#define ROS_VERSION_MINIMUM(major, minor, patch) ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, major, minor, patch)
