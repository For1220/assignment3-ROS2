cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE include)
# 链接海康 MVS SDK
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

# 添加依赖
find_package(sensor_msgs REQUIRED)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge)

# 链接海康核心库
target_link_libraries( camera_node ${OpenCV_LIBS} /opt/MVS/lib/64/libMvCameraControl.so pthread)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

# TODO(student): Declare and link the dependencies used by your implementation,
# including the vendor SDK. The starter does not discover or link MVS.

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
