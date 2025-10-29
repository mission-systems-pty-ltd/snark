# The basic structure here is:
# application -> SDK wrapper -> SDK
#
# The upstream SDK comes in two similar but incompatible versions, with major
# version numbers of 1 and 3. So we build two wrappers and two versions of the
# application.
#
# So the directory structure is:
# innovusion
#   ├─ v1  (wrapper of SDK 1)
#   ├─ v3  (wrapper of SDK 3)
#   └─ applications
#         └─ innovusion-cat.cpp  (builds innovusion-cat1 and innovusion-cat3)
#
# CMakeLists.txt for v1 and v3 is the same but we need to have separate
# directories to allow CMake to see them as distinct targets when building the
# applications.

set( PROJECT "innovusion" )

if( WIN32 )
    message( FATAL_ERROR "not implemented (but could be)" )
endif()

foreach( innovusion_build_version IN LISTS snark_build_sensors_innovusion_sdk_versions )
    add_subdirectory(v${innovusion_build_version})
endforeach()

add_subdirectory( applications )
