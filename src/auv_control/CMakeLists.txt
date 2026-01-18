cmake_minimum_required(VERSION 3.8)
project(auv_control)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(auv_control_lib
  src/pid.cpp
  src/control_node.cpp
)
ament_target_dependencies(auv_control_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# Create executable
add_executable(control_node src/main.cpp)
target_link_libraries(control_node auv_control_lib)
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# Install targets
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  auv_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
