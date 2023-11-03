SET( PROJECT "Basler" )
SET( TARGET_NAME snark_${PROJECT} )

if( WIN32 )
    message( FATAL_ERROR "not implemented" )
endif( WIN32 )

# SET( BASLER_PYLON_DIR /opt/pylon5 CACHE PATH "basler pylon library installation directory" )
# SET( BASLER_PYLON_MAJOR_VERSION 5 CACHE STRING "major version of basler pylon libraries" )
# SET( BASLER_PYLON_MINOR_VERSION 0 CACHE STRING "minor version of basler pylon libraries" )
SET( BASLER_PYLON_DIR /opt/pylon CACHE PATH "basler pylon library installation directory" )
SET( BASLER_PYLON_MAJOR_VERSION 7 CACHE STRING "major version of basler pylon libraries" )
SET( BASLER_PYLON_MINOR_VERSION 4 CACHE STRING "minor version of basler pylon libraries" )
if( ${BASLER_PYLON_MAJOR_VERSION} LESS 5 )
    SET( BASLER_PYLON_GENICAM_DIR ${BASLER_PYLON_DIR}/genicam CACHE PATH "basler genicam directory" )
endif()

# voodoo, see pylon sample makefiles
#INCLUDE_DIRECTORIES( BEFORE SYSTEM ${BASLER_PYLON_DIR}/include )
# if building JAI is enabled, SYSTEM (that translates into c++ -isystem) makes compiler
# pick up JAI GenApi headers in some cases, which obviously does not compile
INCLUDE_DIRECTORIES( ${BASLER_PYLON_DIR}/include )

if( ${BASLER_PYLON_MAJOR_VERSION} LESS_EQUAL 5 )
    LINK_DIRECTORIES( ${BASLER_PYLON_DIR}/lib64 )
else()
    LINK_DIRECTORIES( ${BASLER_PYLON_DIR}/lib )
endif()
if( ${BASLER_PYLON_MAJOR_VERSION} LESS 5 )
    INCLUDE_DIRECTORIES( BEFORE ${BASLER_PYLON_GENICAM_DIR}/library/CPP/include )
    LINK_DIRECTORIES( ${BASLER_PYLON_DIR}/lib )
    LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux64_x64 )
    LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux64_x64/GenApi/Generic )
    LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux32_i86 )
    LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux32_i86/GenApi/Generic )
endif()

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GIGE -Wl,-E" )

file( GLOB source *.cpp )
file( GLOB includes *.h )
source_group( ${TARGET_NAME} FILES ${source} ${includes} )
add_subdirectory( applications )
