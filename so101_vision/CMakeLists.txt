cmake_minimum_required(VERSION 3.10...3.27)
project(so101_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(wrist_camera_tf_node
  src/wrist_camera_tf_node.cpp
)

target_compile_features(wrist_camera_tf_node PUBLIC cxx_std_17)

ament_target_dependencies(wrist_camera_tf_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  wrist_camera_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
