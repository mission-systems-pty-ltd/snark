cmake_minimum_required( VERSION 3.1 )
if( POLICY CMP0048 )
    cmake_policy( SET CMP0048 NEW )
    project( snark VERSION 1.0.10 LANGUAGES CXX )
else( POLICY CMP0048 )
    project( snark )
    set( CMAKE_PROJECT_VERSION_MAJOR 1 )
    set( CMAKE_PROJECT_VERSION_MINOR 0 )
    set( CMAKE_PROJECT_VERSION_PATCH 10 )
endif( POLICY CMP0048 )

set( snark_version_major "${CMAKE_PROJECT_VERSION_MAJOR}" ) # quick and dirty
set( snark_version_minor "${CMAKE_PROJECT_VERSION_MINOR}" ) # quick and dirty
set( snark_version_patch "${CMAKE_PROJECT_VERSION_PATCH}" ) # quick and dirty
set( snark_version "${CMAKE_PROJECT_VERSION}" ) # quick and dirty

ENABLE_TESTING()
INCLUDE( CMakeDependentOption )

IF (UNIX AND NOT APPLE)
  # May also fail on BSD as it assumes GNU date rather than BSD date command
	# Check if Git is installed
	find_package(Git)
	IF(GIT_FOUND)
		# Get the date of the last commit (for use by CPack)
		EXECUTE_PROCESS(
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND git log -n1 --format=%ci
		COMMAND xargs -0 date +%Y%m%dT%H%M -d
		OUTPUT_VARIABLE COMMITDATE
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
		# And also the hash.
		EXECUTE_PROCESS(
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND git rev-parse HEAD
		OUTPUT_VARIABLE COMMITHASH
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	ELSE()
		SET ( COMMITDATE 1 )
		SET ( COMMITHASH "abc" )
	ENDIF()
ELSE() # Probably windows; gets hardcoded patch version
	SET ( COMMITDATE 1 )
	SET ( COMMITHASH "abc" )
ENDIF()

# ---------------------------------------------------------------------------
# set compiler and linker flags and variables

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING "options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE )
endif( NOT CMAKE_BUILD_TYPE )

set( comma_DIR "/usr/local/CMakeFiles" CACHE STRING "location of comma cmake files" )
set( _comma_DIR "${comma_DIR}" )
find_package( comma PATHS ${comma_DIR} )
if( comma_DIR STREQUAL "comma_DIR-NOTFOUND" )
    message( FATAL_ERROR "comma not found in ${_comma_DIR}" )
endif( comma_DIR STREQUAL "comma_DIR-NOTFOUND" )
message( "found comma ${comma_version}" )

if( UNIX )
    include( ${comma_DIR}/check.c++.standard.cmake REQUIRED )
endif()

IF(WIN32)
  SET(CMAKE_CXX_WARNING_LEVEL 4)
  IF(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    STRING(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  ELSE(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  ENDIF(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
  ADD_DEFINITIONS(/wd4512)
  ADD_DEFINITIONS(/wd4127)
  ADD_DEFINITIONS(/wd4996)
  ADD_DEFINITIONS(/wd4800)
  ADD_DEFINITIONS(/wd4244)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"NOMINMAX\"")
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
  ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN) #used to fix winsock redefinition error
ENDIF(WIN32)

IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  SET( IS_64_BIT TRUE )
ELSE( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  SET( IS_64_BIT FALSE )
ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )

IF(CMAKE_BUILD_TOOL MATCHES "make")
  IF(NOT CMAKE_CXX_FLAGS MATCHES "-Wall")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  ENDIF(NOT CMAKE_CXX_FLAGS MATCHES "-Wall")
  IF(NOT CMAKE_C_FLAGS MATCHES "-Wall")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  ENDIF(NOT CMAKE_C_FLAGS MATCHES "-Wall")

  IF( IS_64_BIT )
    IF(NOT CMAKE_CXX_FLAGS MATCHES "-fPIC")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    ENDIF(NOT CMAKE_CXX_FLAGS MATCHES "-fPIC")
    IF(NOT CMAKE_C_FLAGS MATCHES "-fPIC")
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    ENDIF(NOT CMAKE_C_FLAGS MATCHES "-fPIC")
  ENDIF( IS_64_BIT )
ENDIF(CMAKE_BUILD_TOOL MATCHES "make")

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

SET( SOURCE_CODE_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )
SET( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib )

set( snark_INSTALL_BIN_DIR "bin" CACHE STRING "installation directory for binaries" )
set( snark_INSTALL_INCLUDE_DIR "include/snark" CACHE STRING "installation directory for include directories" )
set( snark_INSTALL_LIB_DIR "lib" CACHE STRING "installation directory for libraries" )
set( snark_INSTALL_PACKAGE_DIR "CMakeFiles" CACHE STRING "installation directory for cmake files" )

SET( snark_LIBRARY_PROPERTIES ${snark_LIBRARY_PROPERTIES} DEBUG_POSTFIX "-d" SOVERSION ${CMAKE_PROJECT_VERSION} LINK_FLAGS_RELEASE -s )

#INCLUDE_DIRECTORIES( ${SOURCE_CODE_BASE_DIR}/../ )
#INCLUDE_DIRECTORIES( ${snark_INSTALL_INCLUDE_DIR}/../ )

IF( APPLE )
    SET( CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_LIB_DIR} )
ENDIF( APPLE )

OPTION( USE_CUDA "use cuda in (some) applications" OFF )
if( USE_CUDA )
    find_package( CUDA REQUIRED )
    include( FindCUDA )
    list( APPEND CUDA_NVCC_FLAGS --compiler-options -fno-strict-aliasing -lineinfo -use_fast_math -Xptxas -dlcm=cg )
#    list( APPEND CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20 ) # todo: make sms configurable?
#    list( APPEND CUDA_NVCC_FLAGS -gencode arch=compute_30,code=sm_30 )
#    list( APPEND CUDA_NVCC_FLAGS -gencode arch=compute_35,code=sm_35 )
#    list( APPEND CUDA_NVCC_FLAGS -gencode arch=compute_37,code=sm_37 )
#    list( APPEND CUDA_NVCC_FLAGS -gencode arch=compute_50,code=sm_50 )
#    list( APPEND CUDA_NVCC_FLAGS -gencode arch=compute_52,code=sm_52 )
    #message( "CUDA_NVCC_FLAGS: ${CUDA_NVCC_FLAGS}" )
    list( APPEND CUDA_NVCC_FLAGS -D_MWAITXINTRIN_H_INCLUDED -D__STRICT_ANSI__ -D_FORCE_INLINES )
    set( CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE ON ) # todo: what is this?
    set( CUDA_SEPARABLE_COMPILATION ON ) # todo: what is this?
    set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSNARK_USE_CUDA" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSNARK_USE_CUDA" )
endif( USE_CUDA )

find_package( ZLIB )

OPTION( BUILD_TESTS "build unit tests" OFF )
SET( snark_BUILD_TESTS ${BUILD_TESTS} )

OPTION( BUILD_APPLICATIONS "build applications" ON )
SET( snark_BUILD_APPLICATIONS ${BUILD_APPLICATIONS} )

option( BUILD_SHARED_LIBS "build with shared libraries" ON )
set( snark_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS} )

if( BUILD_SHARED_LIBS )
    foreach( library ${comma_ALL_EXTERNAL_LIBRARIES} )
        if( ${library} MATCHES ".a$" )
            get_filename_component( library_name ${library} NAME )
            message( WARNING "You have BUILD_SHARED_LIBS=ON but comma specifies a static lib for ${library_name}. This may cause linker issues. To resolve, build comma with BUILD_SHARED_LIBS=ON" )
        endif()
    endforeach()
endif()

IF( NOT BUILD_SHARED_LIBS )
    SET( Boost_USE_STATIC_LIBS ON )
ENDIF( NOT BUILD_SHARED_LIBS )

option( snark_BUILD_PYTHON_PACKAGES "install snark python packages" OFF )
CMAKE_DEPENDENT_OPTION( snark_BUILD_PYTHON_BINDINGS "build C++/Python bindings" OFF "snark_BUILD_PYTHON_PACKAGES" ON )

IF( snark_BUILD_PYTHON_PACKAGES AND snark_BUILD_PYTHON_BINDINGS )
    IF( NOT BUILD_SHARED_LIBS )
        MESSAGE( WARNING "Linking C++/Python bindings with static libraries is requested. If linking fails, recompile required libraries with -fPIC or use shared libraries. Note that this feature has only been tested with shared libraries." )
    ENDIF( NOT BUILD_SHARED_LIBS )
    #SET( Python_ADDITIONAL_VERSIONS 2.7 )
    FIND_PACKAGE( PythonLibs REQUIRED )
    INCLUDE_DIRECTORIES( ${PYTHON_INCLUDE_DIRS} )
    FIND_PACKAGE( Boost COMPONENTS REQUIRED iostreams thread filesystem system serialization date_time program_options regex python python3 )
ELSE( snark_BUILD_PYTHON_PACKAGES AND snark_BUILD_PYTHON_BINDINGS )
    FIND_PACKAGE( Boost COMPONENTS REQUIRED iostreams thread filesystem system serialization date_time program_options regex )
ENDIF( snark_BUILD_PYTHON_PACKAGES AND snark_BUILD_PYTHON_BINDINGS )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${Boost_LIBRARY_DIRS} )

IF( ${CXX_STANDARD_TO_USE} MATCHES "11" )
    # Work around a boost bug in Boost 1.56 and below when the library is compiled
    # against one C++ version and the application is compiled against another.
    # It will manifest itself as a linker error.
    # ref: https://svn.boost.org/trac/boost/ticket/6124
    if( Boost_MAJOR_VERSION EQUAL 1 )
        if( Boost_MINOR_VERSION LESS 51 )
            set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_NO_SCOPED_ENUMS" )
        elseif( Boost_MINOR_VERSION LESS 57 )
            set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_NO_CXX11_SCOPED_ENUMS" )
        endif()
    endif()
endif()

IF( WIN32 )
    # improve later
    #FIND_PATH( Eigen_INCLUDE_DIR NAMES Eigen/CMakeLists.txt PATHS $ENV{EIGEN_ROOT} )
    FIND_PATH( Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS $ENV{EIGEN_ROOT} )
ELSEIF( APPLE )
    # actually the following line will probably work everywhere, not just APPLE (but have not tested on Windows yet)
    FIND_PATH( Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS $ENV{EIGEN_ROOT} PATH_SUFFIXES eigen3 )
ELSE( WIN32 )
    #FIND_PATH( Eigen_INCLUDE_DIR Eigen/Core PATHS $ENV{EIGEN_ROOT} /usr/include/eigen3 /usr/local/include/eigen3 )
    FIND_PATH( Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS $ENV{EIGEN_ROOT} /usr/include/eigen3 /usr/local/include/eigen3 )
ENDIF( WIN32 )
IF( NOT EXISTS ${Eigen_INCLUDE_DIR} )
    MESSAGE( "could not find Eigen - should be the path containing the Eigen 3 directory and /Eigen/Eigen, /Eigen/Core, etc..." )
ELSE( NOT EXISTS ${Eigen_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${Eigen_INCLUDE_DIR} )
ENDIF( NOT EXISTS ${Eigen_INCLUDE_DIR} )

IF( snark_BUILD_TESTS )

    # todo: maybe else clause is actually the right way to do things
    IF( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" OR ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
        FIND_PATH( GTest_SOURCE_DIR NAMES CMakeLists.txt HINTS /usr/src/googletest/googletest DOC "source code of gtest" ) # FIND_PATH( GTest_SOURCE_DIR NAMES CMakeLists.txt HINTS /usr/src/gtest DOC "source code of gtest" )
        ADD_SUBDIRECTORY( ${GTest_SOURCE_DIR} gtest )
        INCLUDE_DIRECTORIES( src ${GTest_SOURCE_DIR}/include ${GTest_SOURCE_DIR} )
        SET( GTEST_BOTH_LIBRARIES gtest gtest_main )
    ELSE( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" OR ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
        FIND_PACKAGE( GTest )
        INCLUDE_DIRECTORIES( ${GTEST_INCLUDE_DIRS} )
    ENDIF( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" OR ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )

#TT: Install tests to dir. This allows them to be packed by CPack.
#Note: if your source tree is dirty (eg you've already run tests there and have output/ directories)
#those will also be installed too.

OPTION( INSTALL_TESTS "Install the tests into the nominated directory" OFF )
IF( INSTALL_TESTS )
    SET ( snark_CPP_TESTS_INSTALL_DIR bin CACHE PATH "Install directory for cpp unit tests" )
    SET ( snark_INSTALL_SNARK_TESTS_DIR share/test )

# Get the list of test directories (using glob).
FILE ( GLOB_RECURSE all_files
		# RELATIVE is tricky. When you use it, you need to specify the path you expect the output to be RELATIVE to. (Yes, that path could be considerably non-relative to the file that is found.)
		RELATIVE ${CMAKE_SOURCE_DIR}
		* )

SET ( test_dirs )
FOREACH ( i ${all_files} )
	if ( ${i} MATCHES "[\\/]test[\\/]" )
		LIST (APPEND test_dirs ${i} )
	endif ()
ENDFOREACH (i)

# Add those directories
	FOREACH (file ${test_dirs} )
			# Determine the parent directories for a given file so to prefix the install location (otherwise they just all get installed in the destination root)
			get_filename_component( dest_dir ${file} DIRECTORY )
			# See that slash there? 		 vvv It's really important.
			INSTALL ( 	DIRECTORY ${dest_dir}/
						DESTINATION ${snark_INSTALL_SNARK_TESTS_DIR}/${CMAKE_PROJECT_NAME}/unit/${dest_dir}
						USE_SOURCE_PERMISSIONS )
	ENDFOREACH (file)

ENDIF( INSTALL_TESTS )
ENDIF( snark_BUILD_TESTS )

OPTION( snark_build_actuators "build actuators" ON )
OPTION( snark_build_batteries "build batteries" OFF )
OPTION( snark_build_control "build control" ON )
OPTION( snark_build_geodesy_solar "build geodesy solar" OFF )
OPTION( snark_build_graph "build graph" ON )
OPTION( snark_build_graphics "build graphics" ON )
OPTION( snark_build_imaging "build imaging" ON )
CMAKE_DEPENDENT_OPTION( snark_build_imaging_opencv_contrib "build opencv contrib functions" ON "snark_build_imaging" ON )
CMAKE_DEPENDENT_OPTION( snark_build_imaging_stereo "build imaging stereo" OFF "snark_build_imaging" ON )
OPTION( snark_build_io "build io" OFF )
OPTION( snark_build_math_geometry "build math geometry" ON )
option( snark_build_math_fft "build math-fft" OFF )
OPTION( snark_build_navigation "build navigation" ON )
OPTION( snark_build_point_cloud "build point cloud" ON )
OPTION( snark_build_point_cloud_pcl "build points cloud pcl" OFF )
OPTION( snark_build_python "build python packages" OFF )
OPTION( snark_build_sensors "build sensors" ON )
OPTION( snark_build_terrain "build terrain" ON )
OPTION( snark_build_web "build web" OFF )
option( snark_build_ros "build ros" OFF )


CMAKE_DEPENDENT_OPTION( ADD_PYTHON_PACKAGES_TO_RPM "add snark python packages to RPM" OFF "snark_build_python" ON )

CMAKE_DEPENDENT_OPTION( snark_build_sensors_asd "build asd components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_basler "build basler components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_cepton "build cepton components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_dc1394 "build dc1394 components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_flycapture "build flycapture components for pointgrey" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_echodyne "build echodyne components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_gige "build gige components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_gobi "build gobi components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_ipx "build ipx components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_hokuyo "build hokuyo components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_innovusion "build innovusion components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_jai "build jai components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_kinect "build kinect components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_ouster "build ouster components" OFF "snark_build_sensors" OFF )
cmake_dependent_option( snark_build_sensors_roboteye "build RobotEye components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_realsense "build realsense components" OFF "snark_build_sensors" OFF )
cmake_dependent_option( snark_build_sensors_realsense2 "build realsense2 components" OFF "snark_build_sensors" OFF )
cmake_dependent_option( snark_build_sensors_robosense "build robosense components" OFF "snark_build_sensors" ON )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_sick "build sick components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_velodyne "build velodyne components" OFF "snark_build_sensors" ON )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_vimba "build vimba components" OFF "snark_build_sensors" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_actuators_quickset "build quickset components" OFF "snark_build_actuators" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_actuators_wheels "build wheels components" OFF "snark_build_actuators" ON )
CMAKE_DEPENDENT_OPTION( snark_build_actuators_universal_robots "build universal_robots components" OFF "snark_build_actuators" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_web_feeds "snark: build web feeds" ON "snark_build_web" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_batteries_braille "snark: build braille battery" OFF "snark_build_batteries" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_batteries_ocean "snark: build ocean battery" OFF "snark_build_batteries" OFF )

SET( snark_system_manager "auto" CACHE STRING "the system and service manager of the operating system, options are: auto init systemd" )
IF( snark_system_manager STREQUAL "auto" )
    EXECUTE_PROCESS( COMMAND cat /proc/1/comm
                     OUTPUT_VARIABLE snark_system_manager
                     OUTPUT_STRIP_TRAILING_WHITESPACE
                     ERROR_QUIET )
    MESSAGE( STATUS "auto detected system manager: ${snark_system_manager}" )
ENDIF( snark_system_manager STREQUAL "auto" )

IF( snark_build_imaging )
    OPTION( CUDA_USE_STATIC_CUDA_RUNTIME "use static cuda runtime, enabling seems to cause lots of linking grief with some installations" OFF )
    IF( WIN32 )
        SET( OpenCV_STATIC OFF )
    ENDIF( WIN32 )
    set( OpenCV_REQUIRED_VERSION "" CACHE STRING "opencv version" )
    find_package( OpenCV ${OpenCV_REQUIRED_VERSION} REQUIRED )
    #message( "OpenCV_LIBS: ${OpenCV_LIBS}" )
    #message( "OpenCV_LIB_PATH: ${OpenCV_LIB_PATH}" )
    #message( "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}" )
    include_directories( BEFORE ${OpenCV_INCLUDE_DIRS} )
    if( ${OpenCV_VERSION_MAJOR} GREATER 2 )
        if( snark_build_imaging_stereo )
            message( "imaging/stereo with OpenCV ${OpenCV_VERSION_MAJOR} not supported yet; will not build imaging/stereo" )
        endif( snark_build_imaging_stereo )
        set( snark_build_imaging_stereo OFF CACHE BOOL "" FORCE )
    endif( ${OpenCV_VERSION_MAJOR} GREATER 2 )
ENDIF( snark_build_imaging )

if( snark_build_graphics )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_graphics_qt3d-d
                             optimized snark_graphics_qt3d )
endif( snark_build_graphics )

if( snark_build_math_geometry )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_math_geometry-d
                             optimized snark_math_geometry )
endif( snark_build_math_geometry )

if( snark_build_point_cloud )
    if( NOT snark_build_math_geometry )
        message( FATAL_ERROR  "please turn on snark_build_math_geometry, since snark_build_point_cloud depends on it" )
    endif( NOT snark_build_math_geometry )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_point_cloud-d
                             optimized snark_point_cloud )
endif( snark_build_point_cloud )

if( snark_build_imaging )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_imaging-d
                             optimized snark_imaging )
endif( snark_build_imaging )

# todo: uncomment, once implemented
# if( snark_build_navigation )
#     set( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_navigation_trimble-d
#                              optimized snark_navigation_trimble )
# endif( snark_build_navigation )

IF( snark_build_sensors_echodyne )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_echodyne-d
                             optimized snark_echodyne )
ENDIF( snark_build_sensors_echodyne )

IF( snark_build_sensors_velodyne )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_velodyne-d
                             optimized snark_velodyne )
ENDIF( snark_build_sensors_velodyne )

IF( snark_build_sensors_hokuyo )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_hokuyo-d
                             optimized snark_hokuyo )
ENDIF( snark_build_sensors_hokuyo )

IF( snark_build_sensors_asd )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_asd-d
                             optimized snark_asd )
ENDIF( snark_build_sensors_asd )

if( snark_build_sensors_kinect )
  find_path( LIBUSB_INCLUDE_DIR  NAMES libusb.h  PATH_SUFFIXES "include" "libusb" "libusb-1.0")

  # todo: find usb library
  #find_library( LIBUSB_LIBRARY NAMES usb PATH_SUFFIXES "lib" "lib32" "lib64" "x86_64-linux-gnu" )
  find_library( LIBUSB_LIBRARY NAMES usb HINTS /usr/lib/x86_64-linux-gnu )


  find_package( OpenCL REQUIRED )
  find_package( freenect2 REQUIRED )
endif( snark_build_sensors_kinect )

SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_math-d
                         optimized snark_math
                         debug snark_math_spherical_geometry-d
                         optimized snark_math_spherical_geometry
                         debug snark_timing-d
                         optimized snark_timing )
IF( snark_build_actuators_quickset )
  SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_quickset_ptcr-d
                         optimized snark_quickset_ptcr )
ENDIF( snark_build_actuators_quickset )

IF( snark_build_actuators_wheels )
  SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_wheels-d
                         optimized snark_wheels )
ENDIF( snark_build_actuators_wheels )

if( snark_build_batteries_ocean )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_ocean-d
                             optimized snark_ocean )
endif( snark_build_batteries_ocean )

if( snark_build_control )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_control-d
                             optimized snark_control )
endif( snark_build_control )

IF(WIN32)
    SET( snark_ALL_EXTERNAL_LIBRARIES Ws2_32.lib )
ENDIF(WIN32)

include( CMakeFiles/FindTBB.cmake REQUIRED )
include_directories( ${TBB_INCLUDE_DIRS} )
link_directories( ${TBB_LIBRARY_DIRS} )
find_file( TBB_VERSION_H version.h HINTS ${TBB_INCLUDE_DIRS}/tbb )
if( NOT TBB_VERSION_H STREQUAL "TBB_VERSION_H-NOTFOUND" )
    add_definitions( -D TBB_VERSION_H_EXISTS )
endif( NOT TBB_VERSION_H STREQUAL "TBB_VERSION_H-NOTFOUND" )

SET( CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -DTBB_USE_EXCEPTIONS" )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTBB_USE_EXCEPTIONS" )

IF(UNIX AND NOT QNXNTO AND NOT APPLE)
    SET( snark_ALL_EXTERNAL_LIBRARIES rt pthread )
ENDIF(UNIX AND NOT QNXNTO AND NOT APPLE)

SET( snark_ALL_EXTERNAL_LIBRARIES ${snark_ALL_EXTERNAL_LIBRARIES} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} ${Boost_LIBRARIES} )
SET( snark_ALL_EXTERNAL_LIBRARIES ${snark_ALL_EXTERNAL_LIBRARIES} )
SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} ${snark_ALL_LIBRARIES} ${ZLIB_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ) # as gcc parses library list only once
SET( snark_EXTERNAL_INCLUDES ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIR} )

CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

FILE( GLOB snark_PACKAGE_CMAKEFILES ${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/*.cmake )

INCLUDE( ${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/generate_${PROJECT_NAME}_config.cmake )

CONFIGURE_FILE( ${PROJECT_SOURCE_DIR}/CMakeFiles/use_${PROJECT_NAME}.cmake.in
                ${PROJECT_BINARY_DIR}/CMakeFiles/use_${PROJECT_NAME}.cmake COPYONLY IMMEDIATE )

SET( snark_PACKAGE_CMAKEFILES ${snark_PACKAGE_CMAKEFILES} ${PROJECT_BINARY_DIR}/CMakeFiles/use_${PROJECT_NAME}.cmake )
INSTALL( FILES ${snark_PACKAGE_CMAKEFILES}
         DESTINATION ${snark_INSTALL_PACKAGE_DIR} )

if( snark_build_actuators )
    add_subdirectory( actuators )
endif()

IF( snark_build_graphics )
    add_subdirectory( graphics )
ENDIF( snark_build_graphics )

IF( snark_build_point_cloud )
    add_subdirectory( point_cloud )
ENDIF( snark_build_point_cloud )

add_subdirectory( doc )

IF( snark_build_imaging )
    add_subdirectory( tbb )
    add_subdirectory( imaging )
ENDIF( snark_build_imaging )

if( snark_build_navigation )
    add_subdirectory( navigation )
endif( snark_build_navigation )

add_subdirectory( math )

IF( snark_build_sensors )
    add_subdirectory( sensors )
ENDIF( snark_build_sensors )

IF( snark_build_batteries )
    add_subdirectory( batteries )
ENDIF( snark_build_batteries )

IF( snark_build_graph )
    add_subdirectory( graph )
ENDIF( snark_build_graph )

add_subdirectory( audio )
add_subdirectory( control )
add_subdirectory( geodesy )
add_subdirectory( machine_learning )
add_subdirectory( render )
add_subdirectory( timing )
add_subdirectory( visiting )

IF( snark_build_terrain )
    add_subdirectory( terrain )
ENDIF( snark_build_terrain )

add_subdirectory( etc/bash_completion.d )

add_subdirectory( python )

if( snark_build_io )
    add_subdirectory( io )
endif( snark_build_io )

IF( snark_build_web )
    add_subdirectory( web )
ENDIF( snark_build_web )

if(snark_build_ros)
    add_subdirectory(ros)
endif()

# Testing
macro( configure_testing build_option module_dir )
    if( ${build_option} )
        file( REMOVE ${CMAKE_SOURCE_DIR}/${module_dir}/disabled )
    else()
        if( NOT EXISTS ${CMAKE_SOURCE_DIR}/${module_dir}/disabled )
            file( WRITE ${CMAKE_SOURCE_DIR}/${module_dir}/disabled "tests disabled by build configuration\n" )
        endif()
    endif()
endmacro()

configure_testing( snark_build_geodesy_solar geodesy/solar/test )
configure_testing( snark_build_navigation navigation )
configure_testing( snark_build_ros ros/applications/test )
configure_testing( snark_build_sensors_echodyne sensors/radars/echodyne )
configure_testing( snark_build_sensors_ouster sensors/lidars/ouster )

add_subdirectory( CMakeFiles/post_install ) # uber-quick and dirty

set( CPACK_PACKAGE_NAME "snark" )
set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "cli utilities for robotics and computer vision" )
set( CPACK_PACKAGE_VENDOR "orthographic" )
set( CPACK_DEBIAN_PACKAGE_DESCRIPTION "command line utilities for for robotics and computer vision" )
set( CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://gitlab.com/orthographic/snark/-/wikis/home" )
set( CPACK_DEBIAN_PACKAGE_MAINTAINER "vsevolod vlaskine <vsevolod.vlaskine@gmail.com>" )
#set( CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_SOURCE_DIR}/system/package/debian/changelog" )
set( CPACK_PACKAGE_CONTACT "vsevolod.vlaskine@gmail.com" )
set( CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/COPYING" )
set( CPACK_GENERATOR "DEB" CACHE STRING "cpack generator: PRM or DEB" )
set( CPACK_PACKAGE_VERSION_MAJOR ${snark_version_major} )
set( CPACK_PACKAGE_VERSION_MINOR ${snark_version_minor} )
set( CPACK_PACKAGE_VERSION_PATCH ${snark_version_patch} )
SET(CPACK_RPM_PACKAGE_REQUIRES "snark python numpy python-opencv")
#set( CPACK_DEBIAN_PACKAGE_DEPENDS "snark, " )
set( CPACK_DEBIAN_PACKAGE_SHLIBDEPS "snark, libeigen3-dev, libfftw3-dev, libtbb-dev, zlib1g-dev, libbz2-dev, libopencv-dev, libopencv-highgui-dev, libpcap-dev, qtbase5-dev, qt5-default, qt3d5-dev, libassimp-dev, python-opencv, libqt5charts5-dev, libboost-all-dev (>=1.65), libc" )
set( CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON )
set( CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS_POLICY ON )

# if the user builds shared, make sure that the RPM package (that might be generated) runs ldconfig when it's installed.
if( BUILD_SHARED_LIBS )
    #set( CPACK_RPM_SPEC_MORE_DEFINE "%posttrans -p /sbin/ldconfig" )
    set( CPACK_RPM_POST_INSTALL_SCRIPT_FILE "%posttrans -p /sbin/ldconfig" )
    #set( SHLIBS_FILE "${CMAKE_CURRENT_BINARY_DIR}/shlibs" )
    # witchcraft from here: https://github.com/roboception/rcdiscover/blob/master/cmake/package_debian.cmake
    set( TRIGGERS_FILE "${CMAKE_CURRENT_BINARY_DIR}/triggers" )
    file( WRITE "${TRIGGERS_FILE}" "activate-noawait ldconfig\n" )
    #file( WRITE "${SHLIBS_FILE}" "" )
    #foreach ( libname ${sharedlibs} )
    #    get_target_property( so_abiversion ${libname} SOVERSION )
    #    if(NOT ${so_abiversion})
    #      set(so_abiversion ${abiversion})
    #      message(STATUS "SOVERSION of shared lib \"${libname}\" not set explicitly. Using <Major.Minor> of latest tag: ${so_abiversion}")
    #      set_target_properties(${libname} PROPERTIES SOVERSION ${so_abiversion})
    #    endif()
    #    file(APPEND "${SHLIBS_FILE}" "lib${libname} ${so_abiversion} ${CPACK_PACKAGE_NAME}\n")
    #endforeach (libname)
    #execute_process( COMMAND chmod 644 "${SHLIBS_FILE}" "${TRIGGERS_FILE}" )
    execute_process( COMMAND chmod 644 "${TRIGGERS_FILE}" )
    if( CPACK_DEB_COMPONENT_INSTALL )
        set( CPACK_DEBIAN_BIN_PACKAGE_CONTROL_EXTRA "${CPACK_DEBIAN_BIN_PACKAGE_CONTROL_EXTRA};${SHLIBS_FILE};${TRIGGERS_FILE}" )
    else ( CPACK_DEB_COMPONENT_INSTALL )
        set( CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA};${SHLIBS_FILE};${TRIGGERS_FILE}" )
    endif ( CPACK_DEB_COMPONENT_INSTALL )
endif( BUILD_SHARED_LIBS )

IF( snark_build_python AND ADD_PYTHON_PACKAGES_TO_RPM )
    SET(CPACK_RPM_PACKAGE_REQUIRES "comma python numpy python-opencv")
    SET(CPACK_RPM_POST_INSTALL_SCRIPT_FILE "${CMAKE_SOURCE_DIR}/python/snark-rpm-post-install.sh" )
    SET(CPACK_RPM_PRE_INSTALL_SCRIPT_FILE "${CMAKE_SOURCE_DIR}/python/snark-rpm-pre-install.sh" )
	# Prevents /tmp from permission change when installing RPM package
	SET(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION "/tmp" )
ELSE( snark_build_python AND ADD_PYTHON_PACKAGES_TO_RPM )
    SET(CPACK_RPM_PACKAGE_REQUIRES "comma")
ENDIF( snark_build_python AND ADD_PYTHON_PACKAGES_TO_RPM )

if( WIN32 )
    if( snark_build_imaging )
        file( GLOB OpenCV_DLLS ${OpenCV_LIB_PATH}/../bin/*.dll ) #from opencv version 2.4
    endif( snark_build_imaging )
    if( snark_build_graphics )
        get_filename_component( QT_BIN_DIR ${QT_QMAKE_EXECUTABLE} PATH )
        file( GLOB QT_DLLS ${QT_BIN_DIR}/*.dll )
    endif( snark_build_graphics )
    set( snark_PACKAGE_DLLS ${TBB_DLL_DIR}/tbb.dll ${OpenCV_DLLS} ${QT_DLLS} )
    install(FILES ${snark_PACKAGE_DLLS} DESTINATION bin )
    #this line finds the appropriate compiler dlls, e.g. Visual Studio DLLS in windows. Maybe it would find other dlls if a different build tool were used.
    include( InstallRequiredSystemLibraries )
endif( WIN32 )

include( CPack )
