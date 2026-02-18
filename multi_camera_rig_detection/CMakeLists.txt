cmake_minimum_required(VERSION 3.8)
project(multi_camera_rig_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(multi_camera_rig_common REQUIRED)
find_package(multi_camera_rig_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)

# YOLO Detector Library (supports YOLOv8, YOLOv11, segmentation, etc.)
add_library(yolo_detector SHARED
  src/yolo_detector.cpp
)

target_compile_features(yolo_detector PUBLIC cxx_std_17)

target_include_directories(yolo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(yolo_detector
  multi_camera_rig_common::trt_runner  # This brings TensorRT transitively
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolo_detector
  multi_camera_rig_common
  OpenCV
)

# YOLO Detection Node
add_executable(yolo_trt_node src/yolo_trt_node.cpp)
target_compile_features(yolo_trt_node PUBLIC cxx_std_17)
target_include_directories(yolo_trt_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(yolo_trt_node
  yolo_detector
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolo_trt_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  vision_msgs
  multi_camera_rig_common
  multi_camera_rig_msgs
)

# Test executable (standalone TensorRT tester, no ROS dependencies)
add_executable(test_yolo_trt testing/test_yolo_trt.cpp)

target_compile_features(test_yolo_trt PUBLIC cxx_std_17)

target_link_libraries(test_yolo_trt
  multi_camera_rig_common::trt_runner
  ${OpenCV_LIBS}
)

ament_target_dependencies(test_yolo_trt
  multi_camera_rig_common
  OpenCV
)

# Install
install(TARGETS
  yolo_detector
  yolo_trt_node
  test_yolo_trt
  EXPORT ${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()