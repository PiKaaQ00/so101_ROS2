cmake_minimum_required(VERSION 3.10...3.27)
project(so101_direct_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(direct_follower_readonly_node
  src/direct_follower_readonly_node.cpp
)

add_executable(single_joint_write_test_node
  src/single_joint_write_test_node.cpp
)

add_executable(moveit_trajectory_check_node
  src/moveit_trajectory_check_node.cpp
)

add_executable(follow_joint_trajectory_server_node
  src/follow_joint_trajectory_server_node.cpp
)

target_compile_features(direct_follower_readonly_node PUBLIC cxx_std_17)
target_compile_definitions(direct_follower_readonly_node PRIVATE _DEFAULT_SOURCE)
target_compile_features(single_joint_write_test_node PUBLIC cxx_std_17)
target_compile_definitions(single_joint_write_test_node PRIVATE _DEFAULT_SOURCE)
target_compile_features(moveit_trajectory_check_node PUBLIC cxx_std_17)
target_compile_features(follow_joint_trajectory_server_node PUBLIC cxx_std_17)
target_compile_definitions(follow_joint_trajectory_server_node PRIVATE _DEFAULT_SOURCE)

ament_target_dependencies(direct_follower_readonly_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(single_joint_write_test_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(moveit_trajectory_check_node
  moveit_msgs
  rclcpp
)

ament_target_dependencies(follow_joint_trajectory_server_node
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

install(TARGETS
  direct_follower_readonly_node
  single_joint_write_test_node
  moveit_trajectory_check_node
  follow_joint_trajectory_server_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
