ADD_EXECUTABLE( control-calc control-calc.cpp )
TARGET_LINK_LIBRARIES( control-calc ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} snark_control )
set_target_properties( control-calc PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS control-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( control-command control-command.cpp )
TARGET_LINK_LIBRARIES( control-command ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} snark_control )
set_target_properties( control-command PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS control-command RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( control-error control-error.cpp )
TARGET_LINK_LIBRARIES( control-error ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} snark_control )
set_target_properties( control-error PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS control-error RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( control-speed control-speed.cpp )
TARGET_LINK_LIBRARIES( control-speed ${comma_ALL_LIBRARIES} )
set_target_properties( control-speed PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS control-speed RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

INSTALL( PROGRAMS control-follow-waypoints DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( control-console control-console.cpp )
target_link_libraries( control-console snark_math snark_timing ${comma_ALL_LIBRARIES} )
set_target_properties( control-console PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS control-console RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
