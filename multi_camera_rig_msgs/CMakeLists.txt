cmake_minimum_required(VERSION 3.8)
project(multi_camera_rig_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InstanceSegmentation2D.msg"
  "msg/InstanceSegmentation2DArray.msg"
  DEPENDENCIES std_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
