cmake_minimum_required(VERSION 3.8)
project(POBSControl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gather all source files (mirrors wildcard in makefile)
file(GLOB SOURCES src/Code/*.cpp)

add_executable(POBSControl ${SOURCES})

target_include_directories(POBSControl PRIVATE
  src/Headers
  src/Libs
  src/Libs/eigen-5.0.0
)

ament_target_dependencies(POBSControl
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

# Precompile the two heaviest headers (Eigen + nlohmann json) once and reuse
# across all translation units. Without this, each .cpp independently compiles
# all of Eigen/Dense which is large enough to OOM WSL during a parallel build.
target_precompile_headers(POBSControl PRIVATE
  <rclcpp/rclcpp.hpp>
  <nav_msgs/msg/odometry.hpp>
  <geometry_msgs/msg/pose_stamped.hpp>
  <Eigen/Dense>
  "src/Libs/json.hpp"
)

install(TARGETS POBSControl
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()