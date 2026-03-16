cmake_minimum_required(VERSION 3.8)
project(ximea-ros2-wrapper-bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)

# Find Ximea API
find_path(XIMEA_INCLUDE_DIR NAMES m3api/xiApi.h PATHS /opt/XIMEA/include)
find_library(XIMEA_LIBRARY NAMES m3api PATHS /opt/XIMEA/lib)

if(NOT XIMEA_INCLUDE_DIR OR NOT XIMEA_LIBRARY)
  message(WARNING "Ximea API not found — skipping ximea_camera_node. Install the Ximea SDK to enable it.")
else()
  message(STATUS "Found Ximea API: ${XIMEA_LIBRARY}")

  # Add ximea camera node
  add_executable(ximea_camera_node src/ximea_camera_node.cpp)
  ament_target_dependencies(ximea_camera_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    cv_bridge
    camera_info_manager
    OpenCV)
  target_include_directories(ximea_camera_node PRIVATE
    ${XIMEA_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(ximea_camera_node ${XIMEA_LIBRARY})

  install(TARGETS
    ximea_camera_node
    DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY
  calibs
  launch
  configs
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
