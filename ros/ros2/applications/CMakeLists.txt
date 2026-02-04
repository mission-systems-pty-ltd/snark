# Ros 2 points.
find_package( std_msgs REQUIRED )
find_package( sensor_msgs REQUIRED )
add_executable( ros2-points ros2-points.cpp ${detail_source} )
target_compile_definitions( ros2-points PRIVATE SNARK_BUILD_ROS_2 )
target_include_directories( ros2-points PRIVATE
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${ros2_core_include_dirs}
    ${ros2_extra_include_dirs}
)
set( all_ros2_libs ${ros2_core_libs} ${rosbag2_cpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${std_msgs_LIBRARIES} )

target_link_libraries( ros2-points ${all_ros2_libs} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS ros2-points
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS ros2-to-csv ros2-from-csv
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
