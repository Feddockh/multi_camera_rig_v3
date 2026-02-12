cmake_minimum_required(VERSION 3.8)
project(firefly-ros2-wrapper-detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------
# ROS dependencies
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)

# ------------------------
# Existing nodes
# ------------------------
# add_executable(stereo_disparity_node src/stereo_disparity_node.cpp)
# ament_target_dependencies(stereo_disparity_node
#   rclcpp sensor_msgs stereo_msgs cv_bridge message_filters OpenCV
# )

# ------------------------
# TensorRT + CUDA
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
  message(STATUS "TensorRT found. Building yolov8_trt_node")
  message(STATUS "  TENSORRT_INCLUDE_DIR: ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "  TENSORRT_LIB:         ${TENSORRT_LIB}")
  message(STATUS "  TENSORRT_PLUGIN_LIB:  ${TENSORRT_PLUGIN_LIB}")
  message(STATUS "  CUDART_LIB:           ${CUDART_LIB}")

  # ------------------------
  # TRT Runner Library
  # ------------------------
  add_library(trt_runner SHARED
    src/trt_runner.cpp
  )

  target_compile_features(trt_runner PUBLIC cxx_std_17)

  target_include_directories(trt_runner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
  )

  if(CUDAToolkit_FOUND)
    target_include_directories(trt_runner PUBLIC
      ${CUDAToolkit_INCLUDE_DIRS}
    )
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

  set_target_properties(trt_runner PROPERTIES
    BUILD_RPATH "${CUDA_TARGETS_LIB_DIR}"
    INSTALL_RPATH "${CUDA_TARGETS_LIB_DIR}"
  )

  # ------------------------
  # Detection Utils Library
  # ------------------------
  add_library(detection_utils SHARED
    src/detection_utils.cpp
  )

  target_compile_features(detection_utils PUBLIC cxx_std_17)

  target_include_directories(detection_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(detection_utils
    ${OpenCV_LIBS}
  )

  ament_target_dependencies(detection_utils
    OpenCV
  )

  # ------------------------
  # Yolov8 Detector Library
  # ------------------------
  add_library(yolov8_detector SHARED
    src/yolov8_detector.cpp
  )

  target_compile_features(yolov8_detector PUBLIC cxx_std_17)

  target_include_directories(yolov8_detector PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(yolov8_detector
    trt_runner
    detection_utils
    ${OpenCV_LIBS}
  )

  ament_target_dependencies(yolov8_detector
    OpenCV
  )

  # ------------------------
  # YOLO Detection Node
  # ------------------------
  add_executable(yolov8_trt_node src/yolov8_trt_node.cpp)

  target_compile_features(yolov8_trt_node PUBLIC cxx_std_17)

  target_include_directories(yolov8_trt_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(yolov8_trt_node
    yolov8_detector
    ${OpenCV_LIBS}
  )

  ament_target_dependencies(yolov8_trt_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    vision_msgs
  )

  set_target_properties(yolov8_trt_node PROPERTIES
    BUILD_RPATH "${CUDA_TARGETS_LIB_DIR}"
    INSTALL_RPATH "${CUDA_TARGETS_LIB_DIR}"
  )

else()
  message(WARNING
    "TensorRT/CUDA not found. Skipping build of yolov8_trt_node.\n"
    "Need NvInfer.h + libnvinfer + libnvinfer_plugin + cudart."
  )
endif()

# ------------------------
# Install
# ------------------------
# install(TARGETS
#   stereo_disparity_node
#   DESTINATION lib/${PROJECT_NAME}
# )

if(HAVE_TRT)
  install(TARGETS
    trt_runner
    detection_utils
    yolov8_detector
    yolov8_trt_node
    EXPORT ${PROJECT_NAME}_Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
