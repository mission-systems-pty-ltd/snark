install( PROGRAMS echodyne-emulate DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} )

source_group( echodyne-cat FILES echodyne-cat.cpp )
add_executable( echodyne-cat echodyne-cat.cpp )
set_target_properties( echodyne-cat PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
set_target_properties( echodyne-cat PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( echodyne-cat snark_echodyne BNET::bnet_interface_dynamic BNET::bnet_reader )

source_group( echodyne-command FILES echodyne-command.cpp )
add_executable( echodyne-command echodyne-command.cpp )
set_target_properties( echodyne-command PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
set_target_properties( echodyne-command PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( echodyne-command snark_echodyne BNET::bnet_interface_dynamic BNET::bnet_reader )

install( TARGETS echodyne-cat echodyne-command
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
