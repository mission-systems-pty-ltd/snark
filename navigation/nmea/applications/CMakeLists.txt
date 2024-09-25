add_executable( nmea-to-csv nmea-to-csv.cpp )
target_link_libraries( nmea-to-csv snark_navigation_nmea ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( nmea-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS nmea-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( nmea-from-csv nmea-from-csv.cpp )
target_link_libraries( nmea-from-csv snark_navigation_nmea ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( nmea-from-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS nmea-from-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( nmea-checksum nmea-checksum.cpp )
target_link_libraries( nmea-checksum snark_navigation_nmea ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( nmea-checksum PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS nmea-checksum RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
