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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------
# Spinnaker SDK check (required for real hardware only)
# ------------------------
find_library(SPINNAKER_LIB Spinnaker
  PATHS
    /usr/lib
    /usr/local/lib
    /opt/spinnaker/lib
)
if(NOT SPINNAKER_LIB)
  message(WARNING
    "Spinnaker SDK not found — the real hardware camera pipeline "
    "(spinnaker_synchronized_camera_driver) will not be available at runtime. "
    "Simulation (use_gazebo:=true) still works without it. "
    "See multi_camera_rig_cameras/firefly-ros2-wrapper/README.md for installation instructions."
  )
else()
  message(STATUS "Found Spinnaker SDK: ${SPINNAKER_LIB}")
endif()

# ------------------------
# Existing nodes
# ------------------------
add_executable(flash_simulator_node src/flash_simulator_node.cpp)
ament_target_dependencies(flash_simulator_node
  rclcpp sensor_msgs cv_bridge message_filters OpenCV
)

# ------------------------
# Install
# ------------------------
install(TARGETS
  flash_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY calibs configs launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
