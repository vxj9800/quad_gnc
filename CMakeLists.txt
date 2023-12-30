cmake_minimum_required(VERSION 3.8)

# Create project and make it's name available to the executables.
# This will allow the nodes to figure out the name of the package they are in.
project(quad_gnc C CXX)
add_compile_definitions(ROS_PACKAGE_NAME="${CMAKE_PROJECT_NAME}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(quad_sim_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add sources to the target
add_executable(quad_gnc
  src/guidance.cpp
  src/navigation.cpp
  src/control.cpp
  src/quad_gnc.cpp
)
ament_target_dependencies(quad_gnc rclcpp sensor_msgs quad_sim_interfaces)
target_include_directories(quad_gnc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  quad_gnc
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
