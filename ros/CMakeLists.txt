if( snark_build_ros_version_1 )
    add_subdirectory( ros1 )
endif()

if( snark_build_ros_version_2 )
    add_subdirectory( ros2 )
endif()
