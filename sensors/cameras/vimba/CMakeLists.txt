set( PROJECT "vimba" )
set( TARGET_NAME snark_${PROJECT} )

# ------------------------------------------------------------------------------
# Version handling for the Vimba SDK
#
# The Vimba SDK changes from time to time but we only have access to some
# releases. At time of writing: 1.4, 2.0, 4.2, 6.0, 6.1.
#
# The C++ code does conditional compilation based on the version (passed in
# integer format, e.g. 060100), via the VIMBA_SDK_VERSION compile parameter.
#
# Given that we only have access to some releases, the conditionals use only
# those versions, although it might be that other (lower) versions also have the
# required functionality.
# ------------------------------------------------------------------------------

file( RELATIVE_PATH project_path ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

if( WIN32 )
    message( FATAL_ERROR "snark/${project_path}: not implemented for windows" )
endif()

if( NOT IS_64_BIT )
    message( FATAL_ERROR "snark/${project_path}: only implemented for 64 bit systems" )
endif()

file( GLOB source *.cpp )
file( GLOB includes *.h )

source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( ${TARGET_NAME} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/cameras/${PROJECT} )
install( TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

if( NOT DEFINED vimba_sdk_root_previous )
    set( vimba_sdk_root_previous "unset" CACHE PATH "previous VIMBA_SDK_ROOT value" )
    mark_as_advanced( FORCE vimba_sdk_root_previous )
endif()

set( VIMBA_SDK_ROOT "/opt/vimba" CACHE PATH "Vimba SDK root directory" )
if( NOT EXISTS ${VIMBA_SDK_ROOT} )
    message( FATAL_ERROR "\nPlease install the Vimba SDK at ${VIMBA_SDK_ROOT} "
        "or set VIMBA_SDK_ROOT to point to the SDK installation directory.\n"
        "The SDK is available from https://www.alliedvision.com/en/products/software.html.\n"
        "The code in this project is currently developed and tested against version 6.1 of the SDK although it has also been tested against versions 4.2, 2.0 and 1.4."
        )
endif()

# If we change VIMBA_SDK_ROOT, reset the dependent options
if( NOT ${VIMBA_SDK_ROOT} STREQUAL ${vimba_sdk_root_previous} )
    unset( VIMBA_SDK_INCLUDE_DIR CACHE )
    unset( VIMBA_SDK_LIB_DIR CACHE )
    unset( VIMBA_GENICAM_GENTL64_PATH CACHE )
    set( vimba_sdk_root_previous ${VIMBA_SDK_ROOT} CACHE PATH "previous VIMBA_SDK_ROOT value" FORCE )
endif()

set( VIMBA_SDK_INCLUDE_DIR "${VIMBA_SDK_ROOT}" CACHE PATH "Path to Vimba SDK include directory" )
include_directories( ${VIMBA_SDK_INCLUDE_DIR} )

# Set up Vimba SDK shared libraries (selected by architecture)
set( VIMBA_SDK_x86_64_LIB_DIR "x86_64bit" )
set( VIMBA_SDK_aarch64_LIB_DIR "arm_64bit" )
set( VIMBA_SDK_LIB_DIR "${VIMBA_SDK_ROOT}/VimbaCPP/DynamicLib/${VIMBA_SDK_${CMAKE_SYSTEM_PROCESSOR}_LIB_DIR}" CACHE PATH "Path to Vimba SDK shared libraries" )
link_directories( ${VIMBA_SDK_LIB_DIR} )

set( vimba_conf ${CMAKE_CURRENT_BINARY_DIR}/vimba.conf )
add_custom_command( OUTPUT vimba.conf
                    COMMAND echo ${VIMBA_SDK_LIB_DIR} > ${vimba_conf}
                    COMMENT "generating vimba.conf with VIMBA_SDK_LIB_DIR=${VIMBA_SDK_LIB_DIR}"
                    VERBATIM )
add_custom_target( create_vimba_conf DEPENDS vimba.conf )
add_dependencies( ${TARGET_NAME} create_vimba_conf )
install( FILES ${vimba_conf} DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"${project_path}: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )

# Try to determine the SDK version. This will work for later SDK's (6.0+ at least)
# which are the ones we care about.
string( REGEX REPLACE ".*/[Vv]imba_([0-9]+)_([0-9]+)$" "\\1.\\2" VIMBA_SDK_VERSION ${VIMBA_SDK_ROOT} )
message( STATUS "VIMBA_SDK_VERSION=${VIMBA_SDK_VERSION}" )

# ---------------------------------------------------------------
# Turn dotted version string (e.g. 6.1) in to integerized version
# string (e.g. 060100) so that we can use it in C++ code for
# "if" macros
function( pad_string output str padchar len )
    string( LENGTH "${str}" _strlen )
    math( EXPR _padlen "${len} - ${_strlen}" )
    if( _padlen GREATER 0 )
        unset( _pad )
        foreach( _i RANGE 1 ${_strlen})
            string( APPEND _pad ${padchar} )
        endforeach()
        string( PREPEND str ${_pad} )
    endif()
    set( ${output} "${str}" PARENT_SCOPE )
endfunction()

# dotted version: a.b.c (e.g. "6.1")
# int version: xyz (e.g. "060100")
function( dotted_version_to_int_version int_version dotted_version )
    string( REGEX REPLACE "\\." ";" _version_list ${dotted_version} )
    list( LENGTH _version_list _element_count )
    if( _element_count EQUAL 1 )
        list( APPEND _version_list "0;0" )
    endif()
    if( _element_count EQUAL 2 )
        list( APPEND _version_list "0" )
    endif()

    set( _int_version "" )
    foreach( _element ${_version_list} )
        pad_string( _padded_element ${_element} "0" 2 )
        string( APPEND _int_version ${_padded_element} )
    endforeach()
    set( ${int_version} "${_int_version}" PARENT_SCOPE )
endfunction()
# ---------------------------------------------------------------

dotted_version_to_int_version( VIMBA_SDK_VERSION_INT ${VIMBA_SDK_VERSION} )
message( STATUS "VIMBA_SDK_VERSION_INT=${VIMBA_SDK_VERSION_INT}" )

set_source_files_properties( ${TARGET_NAME} PROPERTIES COMPILE_OPTIONS "-DVIMBA_SDK_VERSION=${VIMBA_SDK_VERSION_INT}" )

# Set up an environment variable default
#
# For Vimba SDK 6.0+ (and possibly earlier), this is not required as the same
# data is written to /etc/profile.d/<vimba-profile> and loaded on shell start
if( VIMBA_SDK_VERSION VERSION_LESS 6.0 )
    set( VIMBA_GENICAM_GENTL64_PATH "${VIMBA_SDK_ROOT}/VimbaGigETL/CTI/x86_64bit" CACHE PATH "Path to GenTL provider library" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVIMBA_GENICAM_GENTL64_PATH=\"${VIMBA_GENICAM_GENTL64_PATH}\"" )
endif()

add_subdirectory( applications )
