source_group( realsense2-util FILES realsense2-util.cpp )

add_executable( realsense2-util realsense2-util.cpp )
target_link_libraries( realsense2-util snark_imaging ${snark_ALL_EXTERNAL_LIBRARIES} realsense2 )

set_target_properties( realsense2-util PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS realsense2-util RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
