cmake_minimum_required(VERSION 3.8)
project(my_moveit_app)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# 2. add executable  
add_executable(ur_manipulator_node src/moveit_test_UR.cpp)
add_executable(ur_collision_check_node src/collision_test.cpp)

# 3. link Library

#target_link_libraries(ur_manipulator_node 
#  moveit_ros_planning_interface::moveit_ros_planning_interface
#  rclcpp::rclcpp
#  geometry_msgs::geometry_msgs
#)

ament_target_dependencies(ur_manipulator_node
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
)

ament_target_dependencies(ur_collision_check_node
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
)

# 4. Install
install(TARGETS
  ur_manipulator_node
  ur_collision_check_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
