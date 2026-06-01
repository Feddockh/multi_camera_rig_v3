cmake_minimum_required(VERSION 3.8)
project(multi_camera_rig_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(multi_camera_rig_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# QoS Republisher Node
add_executable(qos_republisher_node src/qos_republisher_node.cpp)
ament_target_dependencies(qos_republisher_node rclcpp multi_camera_rig_common)

# Image Saver Node
add_executable(image_saver_node src/image_saver_node.cpp)
ament_target_dependencies(image_saver_node
  rclcpp sensor_msgs cv_bridge OpenCV
)

# Install executables
install(TARGETS
  qos_republisher_node
  image_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
