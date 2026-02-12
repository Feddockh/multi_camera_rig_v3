cmake_minimum_required(VERSION 3.8)
project(firefly-ros2-wrapper-bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------
# ROS dependencies
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spinnaker_camera_driver REQUIRED)
find_package(spinnaker_synchronized_camera_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------
# Existing nodes
# ------------------------
add_executable(stereo_disparity_node src/stereo_disparity_node.cpp)
ament_target_dependencies(stereo_disparity_node
  rclcpp sensor_msgs stereo_msgs cv_bridge message_filters OpenCV
)

add_executable(point_cloud_node src/point_cloud_node.cpp)
ament_target_dependencies(point_cloud_node
  rclcpp sensor_msgs stereo_msgs cv_bridge message_filters image_geometry OpenCV
)

add_executable(flash_simulator_node src/flash_simulator_node.cpp)
ament_target_dependencies(flash_simulator_node
  rclcpp sensor_msgs cv_bridge message_filters OpenCV
)

add_executable(depth_cleaner_node src/depth_cleaner_node.cpp)
ament_target_dependencies(depth_cleaner_node
  rclcpp sensor_msgs cv_bridge OpenCV
)

add_executable(qos_republisher_node src/qos_republisher_node.cpp)
ament_target_dependencies(qos_republisher_node rclcpp)

add_executable(aruco_detection_node src/aruco_detection_node.cpp)
ament_target_dependencies(aruco_detection_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs cv_bridge tf2_ros tf2_geometry_msgs OpenCV
)
target_include_directories(aruco_detection_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(image_saver_node src/image_saver_node.cpp)
ament_target_dependencies(image_saver_node
  rclcpp sensor_msgs cv_bridge OpenCV
)

add_executable(joy_trigger_node src/joy_trigger_node.cpp)
ament_target_dependencies(joy_trigger_node
  rclcpp sensor_msgs std_srvs
)

# ------------------------
# Install
# ------------------------
install(TARGETS
  stereo_disparity_node
  point_cloud_node
  flash_simulator_node
  depth_cleaner_node
  qos_republisher_node
  aruco_detection_node
  image_saver_node
  joy_trigger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY calibs configs launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
