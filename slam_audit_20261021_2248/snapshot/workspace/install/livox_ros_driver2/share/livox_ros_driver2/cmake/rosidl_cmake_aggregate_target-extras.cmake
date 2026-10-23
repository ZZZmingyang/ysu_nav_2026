# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target livox_ros_driver2::livox_ros_driver2
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${livox_ros_driver2_TARGETS}.
if(livox_ros_driver2_TARGETS AND NOT TARGET livox_ros_driver2::livox_ros_driver2)
  add_library(livox_ros_driver2::livox_ros_driver2 INTERFACE IMPORTED)
  set_target_properties(livox_ros_driver2::livox_ros_driver2 PROPERTIES
    INTERFACE_LINK_LIBRARIES "${livox_ros_driver2_TARGETS}")
endif()
