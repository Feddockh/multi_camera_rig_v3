cmake_minimum_required(VERSION 3.8)
project(firefly-ros2-wrapper-reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------
# ROS dependencies
# ------------------------
find_package(ament_cmake REQUIRED)
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
ament_target_dependencies(semantic_pointcloud sensor_msgs vision_msgs cv_bridge OpenCV rclcpp)

# ------------------------
# TensorRT + CUDA (for foundation stereo)
# ------------------------
find_package(CUDAToolkit QUIET)

# CUDA "targets" dirs
set(CUDA_TARGETS_LIB_DIR "/usr/local/cuda/targets/x86_64-linux/lib")
set(CUDA_TARGETS_INC_DIR "/usr/local/cuda/targets/x86_64-linux/include")

# TensorRT headers
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS
    ${CUDA_TARGETS_INC_DIR}
    /usr/include
    /usr/include/x86_64-linux-gnu
    /usr/local/include
    /usr/include/aarch64-linux-gnu
)

# TensorRT libs
find_library(TENSORRT_LIB nvinfer
  PATHS
    ${CUDA_TARGETS_LIB_DIR}
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/lib/aarch64-linux-gnu
)

find_library(TENSORRT_PLUGIN_LIB nvinfer_plugin
  PATHS
    ${CUDA_TARGETS_LIB_DIR}
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/lib/aarch64-linux-gnu
)

# CUDA runtime
if(CUDAToolkit_FOUND)
  set(CUDART_LIB CUDA::cudart)
else()
  find_library(CUDART_LIB cudart
    PATHS
      ${CUDA_TARGETS_LIB_DIR}
      /usr/local/cuda/lib64
      /usr/lib/x86_64-linux-gnu
      /usr/lib
  )
endif()

set(HAVE_TRT OFF)
if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIB AND TENSORRT_PLUGIN_LIB AND CUDART_LIB)
  set(HAVE_TRT ON)
endif()

if(HAVE_TRT)
  message(STATUS "TensorRT found. Building TensorRT-based components")
  message(STATUS "  TENSORRT_INCLUDE_DIR: ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "  TENSORRT_LIB:         ${TENSORRT_LIB}")
  message(STATUS "  TENSORRT_PLUGIN_LIB:  ${TENSORRT_PLUGIN_LIB}")
  message(STATUS "  CUDART_LIB:           ${CUDART_LIB}")

  # TrtRunner library
  add_library(trt_runner src/trt_runner.cpp)
  target_include_directories(trt_runner PUBLIC ${TENSORRT_INCLUDE_DIR})
  if(CUDAToolkit_FOUND)
    target_include_directories(trt_runner PUBLIC ${CUDAToolkit_INCLUDE_DIRS})
    target_link_libraries(trt_runner
      ${TENSORRT_LIB}
      ${TENSORRT_PLUGIN_LIB}
      CUDA::cudart
    )
  else()
    target_link_libraries(trt_runner
      ${TENSORRT_LIB}
      ${TENSORRT_PLUGIN_LIB}
      ${CUDART_LIB}
    )
  endif()

  # FoundationStereoMatcher library
  add_library(foundation_stereo_matcher src/foundation_stereo_matcher.cpp)
  target_link_libraries(foundation_stereo_matcher
    trt_runner
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(foundation_stereo_matcher OpenCV rclcpp)

  # Foundation stereo matcher node (refactored)
  add_executable(foundation_stereo_matcher_node src/foundation_stereo_matcher_node.cpp)
  target_compile_features(foundation_stereo_matcher_node PUBLIC cxx_std_17)
  
  set_target_properties(foundation_stereo_matcher_node PROPERTIES
    BUILD_RPATH "${CUDA_TARGETS_LIB_DIR}"
    INSTALL_RPATH "${CUDA_TARGETS_LIB_DIR}"
  )

  target_link_libraries(foundation_stereo_matcher_node
    foundation_stereo_matcher
  )
  ament_target_dependencies(foundation_stereo_matcher_node
    rclcpp
    sensor_msgs
    cv_bridge
    message_filters
  )

else()
  message(WARNING
    "TensorRT/CUDA not found. Skipping TensorRT-based components.\n"
    "Need NvInfer.h + libnvinfer + libnvinfer_plugin + cudart."
  )
endif()

# ------------------------
# Stereo rectify + scale node (refactored)
# ------------------------
add_executable(stereo_rectify_scale_node src/stereo_rectify_scale_node.cpp)
target_link_libraries(stereo_rectify_scale_node
  stereo_rectify_scale
)
ament_target_dependencies(stereo_rectify_scale_node
  rclcpp sensor_msgs cv_bridge message_filters OpenCV
)

# ------------------------
# Semantic pointcloud node (refactored)
# ------------------------
add_executable(semantic_pointcloud_node src/semantic_pointcloud_node.cpp)
target_link_libraries(semantic_pointcloud_node
  semantic_pointcloud
)
ament_target_dependencies(semantic_pointcloud_node
  rclcpp sensor_msgs vision_msgs message_filters
)

# ------------------------
# Install
# ------------------------
install(TARGETS
  stereo_rectify_scale_node
  semantic_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

if(HAVE_TRT)
  install(TARGETS
    foundation_stereo_matcher_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
