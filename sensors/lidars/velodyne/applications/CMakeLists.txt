SOURCE_GROUP( velodyne-stream-example FILES velodyne-stream-example.cpp )
ADD_EXECUTABLE( velodyne-stream-example velodyne-stream-example.cpp )
set_target_properties( velodyne-stream-example PROPERTIES LINK_FLAGS_RELEASE -s )
TARGET_LINK_LIBRARIES( velodyne-stream-example snark_velodyne snark_timing ${snark_ALL_EXTERNAL_LIBRARIES} )

SOURCE_GROUP( velodyne-to-csv FILES velodyne-to-csv.cpp )
ADD_EXECUTABLE( velodyne-to-csv velodyne-to-csv.cpp )
set_target_properties( velodyne-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
TARGET_LINK_LIBRARIES( velodyne-to-csv snark_velodyne snark_timing ${snark_ALL_EXTERNAL_LIBRARIES} )

SOURCE_GROUP( velodyne-thin FILES velodyne-thin.cpp )
ADD_EXECUTABLE( velodyne-thin velodyne-thin.cpp )
set_target_properties( velodyne-thin PROPERTIES LINK_FLAGS_RELEASE -s )
TARGET_LINK_LIBRARIES( velodyne-thin snark_velodyne snark_math ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( TARGETS velodyne-to-csv velodyne-thin RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
