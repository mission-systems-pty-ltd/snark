set( dir ${SOURCE_CODE_BASE_DIR}/ros/applications )

## Setup for ROS1
if(EXISTS "/opt/ros/groovy")
    set(ros_path "/opt/ros/groovy" CACHE STRING "Path to ROS Groovy" FORCE)
elseif(EXISTS "/opt/ros/hydro")
    set(ros_path "/opt/ros/hydro" CACHE STRING "Path to ROS Hydro" FORCE)
elseif(EXISTS "/opt/ros/indigo")
    set(ros_path "/opt/ros/indigo" CACHE STRING "Path to ROS Indigo" FORCE)
elseif(EXISTS "/opt/ros/jade")
    set(ros_path "/opt/ros/jade" CACHE STRING "Path to ROS Jade" FORCE)
elseif(EXISTS "/opt/ros/kinetic")
    set(ros_path "/opt/ros/kinetic" CACHE STRING "Path to ROS Kinetic" FORCE)
elseif(EXISTS "/opt/ros/lunar")
    set(ros_path "/opt/ros/lunar" CACHE STRING "Path to ROS Lunar" FORCE)
elseif(EXISTS "/opt/ros/melodic")
    set(ros_path "/opt/ros/melodic" CACHE STRING "Path to ROS Melodic" FORCE)
elseif(EXISTS "/opt/ros/noetic")
    set(ros_path "/opt/ros/noetic" CACHE STRING "Path to ROS Noetic" FORCE)
else()
    message( "ros not found; please specify ros path as something like /opt/ros/melodic on your computer or set snark_build_ros to OFF" )
    set( ros_path "NOT FOUND" CACHE STRING "path to ros, something like /opt/ros/melodic" FORCE )
endif()

set( ros_libs roscpp roscpp_serialization rosbag rosbag_storage rostime rosconsole console_bridge cpp_common )


## Setup for ROS2
cmake_minimum_required(VERSION 3.10) # 3.10 is required for rclcpp packages.
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_generator_cpp REQUIRED )
find_package(rosidl_typesupport_interface REQUIRED )
find_package(rcl_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set( ros2_core_include_dirs ${rclcpp_INCLUDE_DIRS}  ${rmw_INCLUDE_DIRS}  ${rosidl_generator_cpp_INCLUDE_DIRS}  ${rosidl_typesupport_interface_INCLUDE_DIRS}  ${rcl_interfaces_INCLUDE_DIRS} )
set( ros2_extra_include_dirs ${rosbag2_cpp_INCLUDE_DIRS} )
set( ros2_core_libs ${rclcpp_LIBRARIES} ${rmw_LIBRARIES} ${rosbag2_cpp_LIBRARIES} ${rosidl_generator_cpp_LIBRARIES} ${rosidl_typesupport_interface_LIBRARIES} ${rcl_interfaces_LIBRARIES} )
set( ros2_core_libs
    ${rmw_LIBRARIES}
    ${rosbag2_cpp_LIBRARIES}
    ${rosidl_generator_cpp_LIBRARIES}
    ${rosidl_typesupport_interface_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rcl_interfaces_LIBRARIES} 
)

# ROS2 appears to include shared libraries a million times over and over... It slows down the linker so much its painful 
# sorting the list and removing duplicates seems to help a lot - I'm just worried that it's fragile :/
function(sort_and_remove_duplicates input_list_name output_list_name)
    # Dereference the variable name to get its content
    set(input_list "${${input_list_name}}")
    list(REMOVE_DUPLICATES input_list)
    list(SORT input_list)
    # Set the result to the output variable
    set(${output_list_name} ${input_list} PARENT_SCOPE)
endfunction()


set( core_ros2_libs
    ${rclcpp_LIBRARIES} 
    ${rcl_interfaces_LIBRARIES} 
    ${rmw_LIBRARIES} 
    ${rosidl_generator_cpp_LIBRARIES} 
    ${rosidl_typesupport_interface_LIBRARIES} 
    ${type_description_interfaces_LIBRARIES} 
)

sort_and_remove_duplicates( ros2_core_libs ros2_core_libs_sorted )

# TODO: ros2_extra_libs
message("ROS 2 version: $ENV{ROS_DISTRO}")

# ROS2 does not define ROS_VERSION, so we define it here to make the macros in the c++ code 
# make more sense & to allow us to have some form of backwards compatibility.
if ( $ENV{ROS_DISTRO} STREQUAL "ardent" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=1 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "bouncy" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=2 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "crystal" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=3 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "dashing" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=4 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "eloquent" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=5 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "foxy" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=6 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "galactic" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=7 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "humble" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=8 )
    add_definitions( -DROS_VERSION_PATCH=0 )
elseif ( $ENV{ROS_DISTRO} STREQUAL "iron" )
    add_definitions( -DROS_VERSION_MAJOR=2 )
    add_definitions( -DROS_VERSION_MINOR=9 )
    add_definitions( -DROS_VERSION_PATCH=0 )
endif()

install( DIRECTORY messages DESTINATION ${snark_INSTALL_INCLUDE_DIR}/ros )

if( snark_BUILD_APPLICATIONS )
    add_subdirectory( applications )
endif( snark_BUILD_APPLICATIONS )
