cmake_minimum_required(VERSION 3.8)
project(multi_camera_rig_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# CUDA & TensorRT for TRT runner (optional)
find_package(CUDA)

# Find TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include/x86_64-linux-gnu)
find_library(TENSORRT_LIB nvinfer
  PATHS
    /usr/local/cuda/targets/x86_64-linux/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/lib/aarch64-linux-gnu
)
find_library(TENSORRT_PLUGIN_LIB nvonnxparser
  PATHS
    /usr/local/cuda/targets/x86_64-linux/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/lib/aarch64-linux-gnu
)
set(TensorRT_LIBRARIES ${TENSORRT_LIB} ${TENSORRT_PLUGIN_LIB})

if(CUDA_FOUND AND TENSORRT_LIB AND TENSORRT_PLUGIN_LIB)
  message(STATUS "CUDA and TensorRT found — building trt_runner")

  # TensorRT runner library
  add_library(trt_runner src/trt_runner.cpp)
  set_target_properties(trt_runner PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(trt_runner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CUDA_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIRS}
  )
  ament_target_dependencies(trt_runner PUBLIC rclcpp)
  target_link_libraries(trt_runner PRIVATE
    ${CUDA_LIBRARIES}
    ${TensorRT_LIBRARIES}
  )

  # Install library
  install(TARGETS trt_runner
    EXPORT trt_runnerTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # Export targets for downstream packages
  ament_export_targets(trt_runnerTargets HAS_LIBRARY_TARGET)
else()
  message(WARNING "CUDA or TensorRT NOT found — skipping trt_runner. Detection and reconstruction inference nodes will not be built.")
endif()

# Install headers (always)
install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
