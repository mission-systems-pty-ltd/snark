set( dir ${SOURCE_CODE_BASE_DIR}/ros/applications )

link_directories( "${ros_path}/lib" )
include_directories( "${ros_path}/include" )

file( GLOB detail_source ${dir}/detail/*.cpp )

if( snark_build_ros_version_1 )
    find_package( OpenCV REQUIRED )

    add_executable( ros-array ros-array.cpp ${detail_source} )
    target_link_libraries( ros-array ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} )
    set_target_properties( ros-array PROPERTIES LINK_FLAGS_RELEASE -s )

    add_executable( ros-image ros-image.cpp ${detail_source} )
    target_include_directories( ros-image PRIVATE ${OpenCV_INCLUDE_DIRS} )
    target_link_libraries( ros-image ${ros_libs} snark_imaging ${snark_ALL_EXTERNAL_LIBRARIES} )
    set_target_properties( ros-image PROPERTIES LINK_FLAGS_RELEASE -s )

    add_executable( ros-points ros-points.cpp ${detail_source} )
    target_include_directories(ros-points PRIVATE ${OpenCV_INCLUDE_DIRS} )
    target_link_libraries( ros-points ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} )
    set_target_properties( ros-points PROPERTIES LINK_FLAGS_RELEASE -s )
    target_compile_definitions(ros-points PRIVATE SNARK_BUILD_ROS_1)

    install( TARGETS ros-array ros-image ros-points
            RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

    install( PROGRAMS ros-from-csv ros-to-csv
            DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
endif( snark_build_ros_version_1 )

if( snark_build_ros_version_2 )
    # Ros 2 points.
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    add_executable(ros2-points ros2-points.cpp ${detail_source})
    target_compile_definitions(ros2-points PRIVATE SNARK_BUILD_ROS_2)
    target_include_directories(ros2-points PRIVATE
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
            
endif( snark_build_ros_version_2 )
