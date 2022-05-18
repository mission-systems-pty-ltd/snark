set( snark_python_test_disabled_file "${CMAKE_CURRENT_SOURCE_DIR}/disabled" )
macro( enable_python_testing )
    if( EXISTS "${snark_python_test_disabled_file}" )
        execute_process( COMMAND rm -f "${snark_python_test_disabled_file}" )
        message( STATUS "${snark_python_test_disabled_file} has been removed: testing is enabled" )
    endif()
endmacro()

macro( disable_python_testing )
    if( NOT EXISTS "${snark_python_test_disabled_file}" )
        execute_process( COMMAND touch "${snark_python_test_disabled_file}" )
        message( STATUS "${snark_python_test_disabled_file} added: testing is disabled" )
    endif()
endmacro()

MACRO( check_python_module module )
    EXECUTE_PROCESS( COMMAND ${PYTHON} -c "import ${module}; print( hasattr( ${module}, '__version__' ) and ${module}.__version__ or 'unknown' );"
        RESULT_VARIABLE MODULE_EXIT_CODE
        OUTPUT_VARIABLE MODULE_VERSION_STRING
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "found python ${module} module version ${MODULE_VERSION_STRING}" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( WARNING "python ${module} module not found; install it or disable BUILD_PYTHON_PACKAGES" )
        MESSAGE( STATUS "snark/python modules and scripts will not be installed" )
        disable_python_testing()
        RETURN()
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( check_python_module )

MACRO( set_python_build_lib_directory )
    EXECUTE_PROCESS( COMMAND ${PYTHON} -c "from distutils.dist import Distribution as D; from distutils.command.build import build as B; b = B(D()); b.finalize_options(); print( b.build_lib.split('/',1)[-1] );"
        RESULT_VARIABLE MODULE_EXIT_CODE
        OUTPUT_VARIABLE PYTHON_BUILD_LIB_DIR
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "python build lib directory: ${PYTHON_BUILD_LIB_DIR}" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( WARNING "failed to get python build lib directory" )
        MESSAGE( STATUS "comma/python modules and scripts will not be installed" )
        disable_python_testing()
        RETURN()
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( set_python_build_lib_directory )


IF( NOT snark_build_python )
    disable_python_testing()
    RETURN()
ENDIF( NOT snark_build_python )

FIND_PROGRAM( PYTHON "python3" )

IF( NOT PYTHON )
    MESSAGE( WARNING "python3 not found" )
    MESSAGE( STATUS "snark/python modules and scripts will not be installed" )
    disable_python_testing()
    RETURN()
ENDIF( NOT PYTHON )

check_python_module( comma )
check_python_module( numpy )
check_python_module( cv2 )
if( snark_build_ros )
    check_python_module( rosbag )
    check_python_module( rospy )
    check_python_module( rostopic )
    # check_python_module( ros-kinetic-rospy-message-converter ) # todo: uncomment? or it is too exotic to stipulate?
endif( snark_build_ros )

enable_python_testing()

SET( PYTHON_PACKAGE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "install prefix for python packages, leave empty for python default prefix" )

SET( INSTALL_PREFIX_OPTION "" )
IF( PYTHON_PACKAGE_INSTALL_PREFIX )
    SET( INSTALL_PREFIX_OPTION "--prefix=${PYTHON_PACKAGE_INSTALL_PREFIX}" )
    IF( NOT WIN32 )
        SET( INSTALL_PREFIX_OPTION "${INSTALL_PREFIX_OPTION} --exec-prefix=${PYTHON_PACKAGE_INSTALL_PREFIX}" )
    ENDIF( NOT WIN32 )
ENDIF( PYTHON_PACKAGE_INSTALL_PREFIX )

SET( SETUP_PY "${CMAKE_CURRENT_SOURCE_DIR}/setup.py" )

SET( BUILD_BASE "${CMAKE_CURRENT_BINARY_DIR}/build" )

set_python_build_lib_directory()

ADD_SUBDIRECTORY( snark )

ADD_CUSTOM_TARGET( python_build ALL
                ${PYTHON} ${SETUP_PY} build --build-base ${BUILD_BASE}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

INSTALL( CODE "EXECUTE_PROCESS(
                    COMMAND ${PYTHON} ${SETUP_PY} build --build-base ${BUILD_BASE} install ${INSTALL_PREFIX_OPTION} --force
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )" )

SET_DIRECTORY_PROPERTIES( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES build )

IF( ADD_PYTHON_PACKAGES_TO_RPM )
    SET( PYTHON_TMP "/tmp/snark/python_modules_for_rpm_install" )
    INSTALL( DIRECTORY ${BUILD_BASE}/${PYTHON_BUILD_LIB_DIR}/ DESTINATION ${PYTHON_TMP} FILES_MATCHING PATTERN "*" )
    INSTALL( FILES ${SETUP_PY} DESTINATION ${PYTHON_TMP} )
ENDIF( ADD_PYTHON_PACKAGES_TO_RPM )
