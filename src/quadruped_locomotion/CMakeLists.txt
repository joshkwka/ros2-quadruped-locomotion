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

# 1. Find the Eigen3 package
find_package(Eigen3 REQUIRED)

# 2. Add the include directories so your .cpp files can find <Eigen/Dense>
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 3. Create your node (we'll call the file ik_node.cpp for now)
add_executable(ik_node 
  src/ik_node.cpp 
  src/ik_solver.cpp
)
ament_target_dependencies(ik_node rclcpp std_msgs geometry_msgs sensor_msgs)

# 4. Specifically link Eigen to your executable
target_link_libraries(ik_node Eigen3::Eigen)

# 5. Install the executable so 'ros2 run' can find it
install(TARGETS
  ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
