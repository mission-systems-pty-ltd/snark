set( PROJECT "innovusion" )
set( TARGET_NAME snark_${PROJECT} )

if( WIN32 )
    message( FATAL_ERROR "not implemented (but could be)" )
endif()

find_package( Innovusion REQUIRED )

message( STATUS "found Innovusion library version ${Innovusion_VERSION}" )
message( STATUS "found Innovusion library major version ${Innovusion_VERSION_MAJOR}" )

set( Innovusion_VERSION_DIR "v${Innovusion_VERSION_MAJOR}" )

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
                ${CMAKE_CURRENT_SOURCE_DIR}/version.h
                @ONLY )

file( GLOB source ${Innovusion_VERSION_DIR}/*.cpp )
file( GLOB includes ${Innovusion_VERSION_DIR}/*.h )

source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( ${TARGET_NAME} snark_math tbb ${snark_ALL_EXTERNAL_LIBRARIES} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${Innovusion_VERSION_DIR}/${PROJECT} )
install(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory( applications )
