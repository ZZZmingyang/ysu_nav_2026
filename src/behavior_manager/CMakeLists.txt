cmake_minimum_required(VERSION 3.8)
project(behavior_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(behavior_manager_node
  src/behavior_manager_node.cpp)

add_executable(gimbal_scan_server
  src/servers/gimbal_scan_server.cpp
)

target_include_directories(behavior_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(gimbal_scan_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(behavior_manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(gimbal_scan_server PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  behavior_manager_node
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "behaviortree_cpp"
)

ament_target_dependencies(
  gimbal_scan_server
  rclcpp
  sensor_msgs
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS behavior_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gimbal_scan_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
