cmake_minimum_required(VERSION 3.8)
project(assignment3_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 链接海康 MVS SDK
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

add_executable(mvs_camera_node src/mvs_camera_node.cpp)
ament_target_dependencies(mvs_camera_node rclcpp sensor_msgs)
target_link_libraries(mvs_camera_node libMvCameraControl.so pthread)

install(TARGETS mvs_camera_node DESTINATION lib/${PROJECT_NAME})
ament_package()