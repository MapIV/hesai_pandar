cmake_minimum_required(VERSION 3.5)
project(pandar_pointcloud)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pandar_cloud SHARED
  src/pandar_cloud.cpp
  src/lib/calibration.cpp
  src/lib/decoder/pandar40_decoder.cpp
  src/lib/decoder/pandar_qt_decoder.cpp
  src/lib/decoder/pandar_qt128_decoder.cpp
  src/lib/decoder/pandar_xt32_decoder.cpp
  src/lib/decoder/pandar_xt16_decoder.cpp
  src/lib/decoder/pandar64_decoder.cpp
)

target_include_directories(pandar_cloud
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(pandar_cloud
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
)


rclcpp_components_register_node(pandar_cloud
  PLUGIN "pandar_pointcloud::PandarCloud"
  EXECUTABLE pandar_cloud_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

