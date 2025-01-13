add_executable( signals-cat signals-cat.cpp )
target_link_libraries( signals-cat ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} fftw3 tbb )
set_target_properties( signals-cat PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS signals-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )


