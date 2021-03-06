SET( PROJECT "graphics" )
SET( TARGET_NAME snark_${PROJECT} )

IF( WIN32 OR APPLE )
    FIND_PACKAGE( OpenGL )
ELSE( WIN32 OR APPLE )
    FIND_LIBRARY( OPENGL_LIBRARY NAMES GL REQUIRED )
ENDIF( WIN32 OR APPLE )

# We use qmake to determine the Qt version
#
# If you have multiple Qt installations you can choose between them by setting
# QT_QMAKE_EXECUTABLE, either via ccmake or cmake -DQT_QMAKE_EXECUTABLE=...

if( NOT DEFINED QT_QMAKE_EXECUTABLE )
    find_program( QT_QMAKE_EXECUTABLE NAMES qmake )
endif()

if( NOT EXISTS ${QT_QMAKE_EXECUTABLE} )
    message( FATAL_ERROR "QT_QMAKE_EXECUTABLE=\"${QT_QMAKE_EXECUTABLE}\" does not exist" )
endif()

# Use qmake to find Qt version and path
execute_process( COMMAND ${QT_QMAKE_EXECUTABLE} -v COMMAND grep "Qt version"
                 OUTPUT_VARIABLE snark_qt_qmake_version_string
                 OUTPUT_STRIP_TRAILING_WHITESPACE )

if( snark_qt_qmake_version_string STREQUAL "" )
    message( FATAL_ERROR "\n${QT_QMAKE_EXECUTABLE} did not return a Qt version\nConsider installing qt4-default or qt5-default and qt3d5-dev packages\nTest with \"${QT_QMAKE_EXECUTABLE} -v\"" )
endif()

message( STATUS "${snark_qt_qmake_version_string}" )

string( REGEX REPLACE ".*Qt version ([0-9]+\\.[0-9]+\\.[0-9]+).*" "\\1"
        snark_qt_version ${snark_qt_qmake_version_string} )
string( REGEX REPLACE ".* (.*)" "\\1"
        snark_qt_lib_path ${snark_qt_qmake_version_string} )

if( NOT DEFINED snark_qt_version OR snark_qt_version EQUAL "")
    message( FATAL_ERROR "Please set QT_QMAKE_EXECUTABLE to the path to qmake")
endif()

if( snark_qt_version MATCHES "^[0-9]+\\.[0-9]+" )
    string( REPLACE "." ";" qt_version_list ${snark_qt_version} )
    list( GET qt_version_list 0 QT_VERSION_MAJOR )
    list( GET qt_version_list 1 QT_VERSION_MINOR )
elseif( snark_qt_version MATCHES "^[0-9]+$" )
    set( QT_VERSION_MAJOR ${snark_qt_version} )
    set( QT_VERSION_MINOR "0" )
else( snark_qt_version MATCHES "^[0-9]+\\.[0-9]+" )
    message( FATAL_ERROR "Malformed QT version: ${snark_qt_version}" )
endif( snark_qt_version MATCHES "^[0-9]+\\.[0-9]+" )

if( QT_VERSION_MAJOR EQUAL 4 )
    set( Qt3D_VERSION 1 )
    find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGl QtXml REQUIRED )
    include( ${QT_USE_FILE} )
    # If the user cleared the value, look for it again
    if( Qt3D_LIB STREQUAL "" )
        unset( Qt3D_LIB CACHE )
    endif()
    find_library( Qt3D_LIB Qt3D )
    set( snark_disable_qt3dv2_warning OFF CACHE INTERNAL "" )
elseif( QT_VERSION_MAJOR EQUAL 5 )
    # If Qt is installed in a non-standard location (e.g. /opt/) find_package will
    # need some help. Check for a tell-tale directory and set Qt5_DIR if found.
    if( IS_DIRECTORY "${snark_qt_lib_path}/cmake/Qt5" )
        set( Qt5_DIR "${snark_qt_lib_path}/cmake/Qt5" )
        # In this case "make install" will fail unless rpath is set.
        # It would be preferable to set LD_LIBRARY_PATH at runtime.
        set( CMAKE_INSTALL_RPATH "${snark_qt_lib_path}" )
    endif()
    find_package( Qt5 COMPONENTS Core Gui OpenGL Xml REQUIRED )
    include_directories( ${Qt5Widgets_INCLUDE_DIRS} )
    if( QT_VERSION_MINOR LESS 5 )
        set( Qt3D_VERSION 1 )
        find_package( Qt5 COMPONENTS 3D REQUIRED )
        set( snark_disable_qt3dv2_warning OFF CACHE INTERNAL "" )
    else()
        # 3D libraries changed significantly with Qt5.5
        set( Qt3D_VERSION 2 )
        if( NOT snark_disable_qt3dv2_warning )
            message( WARNING "\nQt ${snark_qt_version} includes Qt3D version 2.\nview-points support for Qt3D version 2 is experimental and incomplete\nConsider using Qt4 (by setting QT_QMAKE_EXECUTABLE to the path to the Qt4 qmake, typically /usr/bin/qmake)" )
            set( snark_disable_qt3dv2_warning ON CACHE INTERNAL "" )
        endif()
        find_package( Qt5 COMPONENTS 3DCore REQUIRED )
    endif()
else()
    message( FATAL_ERROR "Qt version 4 or 5 required" )
endif( QT_VERSION_MAJOR EQUAL 4 )

add_definitions( -DQt3D_VERSION=${Qt3D_VERSION} )

SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( snark_BUILD_APPLICATIONS )

ADD_SUBDIRECTORY( qt3d )
if( QT_VERSION_MAJOR GREATER 5 OR ( QT_VERSION_MAJOR EQUAL 5 AND QT_VERSION_MINOR GREATER 4 ))
    add_subdirectory( qt5.5 )
endif()
