set( PROJECT "io" )
set( TARGET_NAME snark_${PROJECT} )
set( DIR ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

file( GLOB source ${DIR}/*.cpp )
file( GLOB includes ${DIR}/*.h )

source_group( ${PROJECT} FILES ${source} ${includes} ${impl_includes} )
add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( ${TARGET_NAME} ${comma_ALL_LIBRARIES} tbb )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
install( TARGETS ${TARGET_NAME}
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime
         LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime
         ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development )

if( BUILD_APPLICATIONS )
    add_subdirectory( applications )
    add_subdirectory( event/applications )
endif( BUILD_APPLICATIONS )

# if( BUILD_TESTS )
#     add_subdirectory( test )
# endif( BUILD_TESTS )
