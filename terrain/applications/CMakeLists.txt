add_executable( dem-to-csv dem-to-csv.cpp )
target_link_libraries( dem-to-csv snark_terrain ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( dem-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS dem-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS terrain-elevation DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} )
