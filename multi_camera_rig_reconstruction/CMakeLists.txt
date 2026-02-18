cmake_minimum_required(VERSION 3.8)
project(multi_camera_rig_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Workaround for GCC 11+ and OpenCV 4.x FLANN compatibility issue
if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11.0)
  add_compile_options(-Wno-deprecated-declarations)
  add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
endif()

# ------------------------
# ROS dependencies
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(multi_camera_rig_common REQUIRED)
find_package(multi_camera_rig_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# ------------------------
# Shared libraries
# ------------------------

# StereoRectifyScale library
add_library(stereo_rectify_scale src/stereo_rectify_scale.cpp)
ament_target_dependencies(stereo_rectify_scale sensor_msgs OpenCV rclcpp)

# SemanticPointCloud library
add_library(semantic_pointcloud src/semantic_pointcloud.cpp)
ament_target_dependencies(semantic_pointcloud sensor_msgs vision_msgs cv_bridge OpenCV rclcpp multi_camera_rig_msgs)

# FoundationStereoMatcher library
add_library(foundation_stereo_matcher src/foundation_stereo_matcher.cpp)
target_link_libraries(foundation_stereo_matcher
  multi_camera_rig_common::trt_runner  # Brings TensorRT/CUDA transitively
  ${OpenCV_LIBS}
)
ament_target_dependencies(foundation_stereo_matcher
  multi_camera_rig_common
  OpenCV
  rclcpp
)

# ------------------------
# Executables
# ------------------------

# Foundation stereo matcher node
add_executable(foundation_stereo_matcher_node src/foundation_stereo_matcher_node.cpp)
target_compile_features(foundation_stereo_matcher_node PUBLIC cxx_std_17)
target_link_libraries(foundation_stereo_matcher_node foundation_stereo_matcher)
ament_target_dependencies(foundation_stereo_matcher_node
  multi_camera_rig_common
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

# Stereo rectify + scale node
add_executable(stereo_rectify_scale_node src/stereo_rectify_scale_node.cpp)
target_link_libraries(stereo_rectify_scale_node stereo_rectify_scale)
ament_target_dependencies(stereo_rectify_scale_node
  multi_camera_rig_common
  rclcpp sensor_msgs cv_bridge message_filters OpenCV
)

# Semantic pointcloud node
add_executable(semantic_pointcloud_node src/semantic_pointcloud_node.cpp)
target_link_libraries(semantic_pointcloud_node semantic_pointcloud)
ament_target_dependencies(semantic_pointcloud_node
  multi_camera_rig_common
  multi_camera_rig_msgs
  rclcpp sensor_msgs vision_msgs message_filters
)

# ------------------------
# Install
# ------------------------
install(TARGETS
  foundation_stereo_matcher_node
  stereo_rectify_scale_node
  semantic_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()