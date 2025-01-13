set( PROJECT "signals" )
set( TARGET_NAME snark_${PROJECT} )

SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )
# file( GLOB source ${dir}/*.cpp)
# file( GLOB includes ${dir}/*.h)
file( GLOB pipeline_includes ${dir}/pipeline/*.h )
file( GLOB pipeline_source ${dir}/pipeline/*.cpp )
source_group( ${PROJECT} FILES ${source} ${includes} ${pipeline_source} ${pipeline_includes} )
add_library( ${TARGET_NAME} ${source} ${includes} ${pipeline_source} ${pipeline_includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( ${TARGET_NAME} ${comma_ALL_LIBRARIES} fftw3 )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
install( FILES ${impl_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/impl )
install( TARGETS ${TARGET_NAME}
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime       # .exe, .dll
         LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime       # .so, mod.dll
         ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development ) # .a, .lib

if( BUILD_APPLICATIONS )
    add_subdirectory( applications )
endif( BUILD_APPLICATIONS )
