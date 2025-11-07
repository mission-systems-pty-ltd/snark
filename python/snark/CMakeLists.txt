MACRO( disable_python_testing )
    EXECUTE_PROCESS( COMMAND touch ${CMAKE_CURRENT_SOURCE_DIR}/disabled )
    MESSAGE( STATUS "disabled file has been dropped in ${CMAKE_CURRENT_SOURCE_DIR}: testing python modules and applications is disabled" )
ENDMACRO( disable_python_testing )

MACRO( check_python_module python module )
    EXECUTE_PROCESS( COMMAND ${python} -c "import ${module}; print( hasattr( ${module}, '__version__' ) and ${module}.__version__ or 'unknown' );"
        RESULT_VARIABLE MODULE_EXIT_CODE
        OUTPUT_VARIABLE MODULE_VERSION_STRING
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "found ${python} ${module} module version ${MODULE_VERSION_STRING}" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( FATAL_ERROR "${python} ${module} module not found; install it or disable BUILD_PYTHON_PACKAGES" )
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( check_python_module )

MACRO( set_python_build_lib_directory )
    EXECUTE_PROCESS( COMMAND ${PYTHON3} -c "from distutils.dist import Distribution as D; from distutils.command.build import build as B; b = B(D()); b.finalize_options(); print( b.build_lib.split('/',1)[-1] );"
        RESULT_VARIABLE MODULE_EXIT_CODE
        OUTPUT_VARIABLE PYTHON_BUILD_LIB_DIR
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "python build lib directory: ${PYTHON_BUILD_LIB_DIR}" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( WARNING "failed to get python build lib directory" )
        MESSAGE( STATUS "snark/python modules and scripts will not be installed" )
        disable_python_testing()
        RETURN()
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( set_python_build_lib_directory )

find_program( PYTHON3 "python3" )

check_python_module( ${PYTHON3} comma )
check_python_module( ${PYTHON3} dataclasses_json )
check_python_module( ${PYTHON3} numpy )
check_python_module( ${PYTHON3} cv2 )
if( snark_build_ros )
    # Note that ROS 1 Melodic (18.04) uses Python 2. ROS 1 Noetic (20.04) uses Python 3
    # ROS 2 uses Python 3 across the board
    if( snark_build_ros_version_1 )
        if( EXISTS "/opt/ros/noetic" )
            # noetic uses Python3
            check_python_module( ${PYTHON3} rosbag )
            check_python_module( ${PYTHON3} rospy )
            check_python_module( ${PYTHON3} rostopic )
        else()
            find_program( PYTHON2 "python2" )
            check_python_module( ${PYTHON2} rosbag )
            check_python_module( ${PYTHON2} rospy )
            check_python_module( ${PYTHON2} rostopic )
        endif()
    else() # version 2
        check_python_module( ${PYTHON3} rclpy )
    endif( snark_build_ros_version_1 )
    # check_python_module( ros-kinetic-rospy-message-converter ) # todo: uncomment? or it is too exotic to stipulate?
endif( snark_build_ros )

SET( PYTHON_PACKAGE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "install prefix for python packages, leave empty for python default prefix" )

SET( INSTALL_PREFIX_OPTION "" )
IF( PYTHON_PACKAGE_INSTALL_PREFIX )
    SET( INSTALL_PREFIX_OPTION "--prefix=${PYTHON_PACKAGE_INSTALL_PREFIX}" )
    IF( NOT WIN32 )
        SET( INSTALL_PREFIX_OPTION "${INSTALL_PREFIX_OPTION} --exec-prefix=${PYTHON_PACKAGE_INSTALL_PREFIX}" )
    ENDIF( NOT WIN32 )
ENDIF( PYTHON_PACKAGE_INSTALL_PREFIX )

set( BUILD_BASE "${CMAKE_CURRENT_BINARY_DIR}/build" )

set_python_build_lib_directory()

set( SETUP_PY "${CMAKE_CURRENT_SOURCE_DIR}/setup.py" ) # todo? use pip3 instead?

# TODO: (maybe) fix for ROS 1 Melodic (18.04) installation
#       on Melodic snark/ros needs to be installed in the Python 2 packages
#       currently the below only supports Python 3 installation

if( snark_build_python_use_pip )
    # although installing with python3 setup.py (else clause) prints deprecation message,
    # pip3 is a bit of black box, e.g. it hangs for a while sometimes, maybe waiting for some
    # internet response - it would be good to find a way/option to run it purely locally
    # hence, currently, building with pip is set to OFF by default
    find_program( PIP3 "pip3" )
    install( CODE "EXECUTE_PROCESS(
             COMMAND ${PIP3} install ${CMAKE_CURRENT_SOURCE_DIR} --no-index --no-deps --install-option --prefix=${CMAKE_INSTALL_PREFIX}
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )" )
else()
    add_custom_target( python_build_snark ALL ${PYTHON3} ${SETUP_PY} build --build-base ${BUILD_BASE}
	               WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
    install( CODE "EXECUTE_PROCESS(
             COMMAND ${PYTHON3} ${SETUP_PY} install --prefix=${CMAKE_INSTALL_PREFIX} --force
             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/build/lib )" )
endif()

if( ADD_PYTHON_PACKAGES_TO_RPM )
    set( PYTHON_TMP "/tmp/snark/python_modules_for_rpm_install" )
    install( DIRECTORY ${BUILD_BASE}/${PYTHON_BUILD_LIB_DIR}/ DESTINATION ${PYTHON_TMP} FILES_MATCHING PATTERN "*" )
    install( FILES ${SETUP_PY} DESTINATION ${PYTHON_TMP} )
endif( ADD_PYTHON_PACKAGES_TO_RPM )
