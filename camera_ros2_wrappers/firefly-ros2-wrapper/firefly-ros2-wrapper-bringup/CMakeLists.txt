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
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
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

# ------------------------
# TensorRT + CUDA (NEW)
# ------------------------
find_package(CUDAToolkit QUIET)

# CUDA "targets" dirs (your ldconfig output shows libs here)
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
  message(STATUS "TensorRT found. Building foundation_stereo_points_node")
  message(STATUS "  TENSORRT_INCLUDE_DIR: ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "  TENSORRT_LIB:         ${TENSORRT_LIB}")
  message(STATUS "  TENSORRT_PLUGIN_LIB:  ${TENSORRT_PLUGIN_LIB}")
  message(STATUS "  CUDART_LIB:           ${CUDART_LIB}")

  add_executable(foundation_stereo_points_node src/foundation_stereo_points_node.cpp)

  # Good defaults for performance
  target_compile_features(foundation_stereo_points_node PUBLIC cxx_std_17)

  # Ensure the runtime can locate TensorRT libs without extra env vars
  set_target_properties(foundation_stereo_points_node PROPERTIES
    BUILD_RPATH "${CUDA_TARGETS_LIB_DIR}"
    INSTALL_RPATH "${CUDA_TARGETS_LIB_DIR}"
  )

  target_include_directories(foundation_stereo_points_node PRIVATE
    ${TENSORRT_INCLUDE_DIR}
  )

  # If you use any CUDA runtime headers directly, this helps when CUDAToolkit is found
  if(CUDAToolkit_FOUND)
    target_include_directories(foundation_stereo_points_node PRIVATE
      ${CUDAToolkit_INCLUDE_DIRS}
    )
  endif()

  # Link order matters sometimes with TRT plugins
  if(CUDAToolkit_FOUND)
    target_link_libraries(foundation_stereo_points_node
      ${OpenCV_LIBS}
      ${TENSORRT_LIB}
      ${TENSORRT_PLUGIN_LIB}
      CUDA::cudart
    )
  else()
    target_link_libraries(foundation_stereo_points_node
      ${OpenCV_LIBS}
      ${TENSORRT_LIB}
      ${TENSORRT_PLUGIN_LIB}
      ${CUDART_LIB}
    )
  endif()

  ament_target_dependencies(foundation_stereo_points_node
    rclcpp
    sensor_msgs
    cv_bridge
    message_filters
    OpenCV
  )

else()
  message(WARNING
    "TensorRT/CUDA not found. Skipping build of foundation_stereo_points_node.\n"
    "Need NvInfer.h + libnvinfer + libnvinfer_plugin + cudart."
  )
endif()

# ------------------------
# Install
# ------------------------
install(TARGETS
  stereo_disparity_node
  point_cloud_node
  flash_simulator_node
  depth_cleaner_node
  qos_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(HAVE_TRT)
  install(TARGETS
    foundation_stereo_points_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install Python scripts
install(PROGRAMS
  scripts/fs_disparity_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fs_disparity_node
)

install(
  DIRECTORY calibs configs launch params rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
