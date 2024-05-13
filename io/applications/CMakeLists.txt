add_executable( video-cat video-cat.cpp )
target_link_libraries( video-cat snark_io ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )
set_target_properties( video-cat PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS video-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
