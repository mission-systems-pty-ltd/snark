set( dir ${SOURCE_CODE_BASE_DIR}/geodesy/applications )
file( GLOB source   ${dir}/*.cpp)
file( GLOB includes ${dir}/*.h)
source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_executable( geo-calc geo-calc.cpp )
target_link_libraries( geo-calc ${snark_ALL_EXTERNAL_LIBRARIES} snark_geodesy )
set_target_properties( geo-calc PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS geo-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS geotiff-cat DESTINATION ${snark_INSTALL_BIN_DIR} )
install( PROGRAMS netcdf-cat DESTINATION ${snark_INSTALL_BIN_DIR} )
