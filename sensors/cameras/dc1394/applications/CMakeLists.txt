add_executable( fire-cat fire-cat.cpp )
target_link_libraries( fire-cat ${comma_ALL_LIBRARIES} snark_dc1394 snark_imaging tbb ${Boost_PROGRAM_OPTIONS_LIBRARY} )
set_target_properties( fire-cat PROPERTIES LINK_FLAGS_RELEASE -s )

add_executable( point-grey point-grey.cpp )
target_link_libraries( point-grey ${comma_ALL_LIBRARIES} snark_dc1394 snark_imaging ${Boost_PROGRAM_OPTIONS_LIBRARY} )
set_target_properties( point-grey PROPERTIES LINK_FLAGS_RELEASE -s )

install( TARGETS fire-cat point-grey RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
