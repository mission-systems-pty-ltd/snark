ADD_EXECUTABLE( csv-to-svg csv-to-svg.cpp )
TARGET_LINK_LIBRARIES( csv-to-svg snark_render snark_svg ${comma_ALL_LIBRARIES} )
set_target_properties( csv-to-svg PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS csv-to-svg RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
