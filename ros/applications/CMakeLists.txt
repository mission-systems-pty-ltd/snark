set( dir ${SOURCE_CODE_BASE_DIR}/ros/applications )

link_directories( "${ros_path}/lib" )
include_directories( "${ros_path}/include" )

file( GLOB detail_source ${dir}/detail/*.cpp )

add_executable( ros-array ros-array.cpp ${detail_source} )
target_link_libraries( ros-array ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( ros-array PROPERTIES LINK_FLAGS_RELEASE -s )

add_executable( ros-image ros-image.cpp ${detail_source} )
target_link_libraries( ros-image ${ros_libs} snark_imaging ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( ros-image PROPERTIES LINK_FLAGS_RELEASE -s )

add_executable( ros-points ros-points.cpp ${detail_source} )
target_link_libraries( ros-points ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( ros-points PROPERTIES LINK_FLAGS_RELEASE -s )

install( TARGETS ros-array ros-image ros-points
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS ros-from-csv ros-to-csv
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

if( snark_build_ros2 )
    # Ros 2 points.
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    add_executable(ros2-points ros2-points.cpp ${detail_source})
    target_include_directories(ros2-points PRIVATE 
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${ros2_core_include_dirs}
        ${ros2_extra_include_dirs}
    )
    # Sorting creatly reduces linker time
    set( all_ros2_libs ${ros2_core_libs} ${rosbag2_cpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${std_msgs_LIBRARIES} )
    # ROS2 appears to include shared libraries a million times over and over... It slows down the linker so much its painful 
    # sorting the list and removing duplicates seems to help a lot - I'm just worried that it's fragile :/
    sort_and_remove_duplicates(all_ros2_libs all_ros2_libs_sorted)

    target_link_libraries( ros2-points ${all_ros2_libs_sorted} ${snark_ALL_EXTERNAL_LIBRARIES} )
    install( TARGETS ros2-points
            DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
endif( snark_build_ros2 )
