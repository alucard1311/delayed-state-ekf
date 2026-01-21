cmake_minimum_required(VERSION 3.8)
project(usbl_navigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Delayed-state EKF library
add_library(delayed_state_ekf
  src/ekf/delayed_state_ekf.cpp
)
target_include_directories(delayed_state_ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(delayed_state_ekf
  rclcpp
)

# Truth generator node
add_executable(truth_generator_node
  src/nodes/truth_generator_node.cpp
  src/nodes/truth_generator_main.cpp
)
target_include_directories(truth_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(truth_generator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# IMU simulator node
add_executable(imu_simulator_node
  src/nodes/imu_simulator_node.cpp
  src/nodes/imu_simulator_main.cpp
)
target_include_directories(imu_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(imu_simulator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# DVL simulator node
add_executable(dvl_simulator_node
  src/nodes/dvl_simulator_node.cpp
  src/nodes/dvl_simulator_main.cpp
)
target_include_directories(dvl_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(dvl_simulator_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# USBL simulator node
add_executable(usbl_simulator_node
  src/nodes/usbl_simulator_node.cpp
  src/nodes/usbl_simulator_main.cpp
)
target_include_directories(usbl_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(usbl_simulator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Navigation node
add_executable(navigation_node
  src/nodes/navigation_node.cpp
  src/nodes/navigation_main.cpp
)
target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(navigation_node
  delayed_state_ekf
)
ament_target_dependencies(navigation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

# Install targets
install(TARGETS
  delayed_state_ekf
  navigation_node
  truth_generator_node
  imu_simulator_node
  dvl_simulator_node
  usbl_simulator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install config and launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
