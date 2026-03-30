cmake_minimum_required(VERSION 3.8)
project(quadruped_locomotion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

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

# Add the include directories so .cpp files can find <Eigen/Dense>
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Pushup Node
add_executable(pushup_node src/pushup_node.cpp src/ik_solver.cpp)
ament_target_dependencies(pushup_node rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs)
target_link_libraries(pushup_node Eigen3::Eigen)

# Trot Node
add_executable(trot_node src/trot_node.cpp src/ik_solver.cpp)
ament_target_dependencies(trot_node rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs)
target_link_libraries(trot_node Eigen3::Eigen)

# Closed-Loop Balance Node
add_executable(balance_node src/balance_node.cpp src/ik_solver.cpp)
ament_target_dependencies(balance_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  trajectory_msgs
)
target_link_libraries(balance_node Eigen3::Eigen)

# Standalone test executable
add_executable(ik_test src/ik_test.cpp src/ik_solver.cpp)
target_link_libraries(ik_test Eigen3::Eigen)

# Install the executables
install(TARGETS
  pushup_node
  trot_node
  balance_node
  ik_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
