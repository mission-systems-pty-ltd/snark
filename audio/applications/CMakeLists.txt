add_executable( audio-sample audio-sample.cpp )
target_link_libraries( audio-sample ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )
set_target_properties( audio-sample PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS audio-sample RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
