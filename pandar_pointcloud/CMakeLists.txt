cmake_minimum_required(VERSION 3.0.2)
project(pandar_pointcloud)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nodelet
  pluginlib
  pcl_conversions
  pcl_ros
  sensor_msgs
  pandar_msgs
  pandar_api
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pandar_msgs
    pandar_api
  LIBRARIES
    pandar_cloud
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
add_library(pandar_cloud
  src/pandar_cloud.cpp
  src/lib/calibration.cpp
  src/lib/decoder/pandar40_decoder.cpp
  src/lib/decoder/pandar_qt_decoder.cpp
  src/lib/decoder/pandar_xt_decoder.cpp
  src/lib/decoder/pandar_xtm_decoder.cpp
  src/lib/decoder/pandar64_decoder.cpp
  src/lib/decoder/pandar_qt128_decoder
)
target_link_libraries(pandar_cloud
  ${catkin_LIBRARIES}
)

## add node
add_executable(pandar_cloud_node
  src/cloud_node.cpp
)

target_link_libraries(pandar_cloud_node
  pandar_cloud
  ${catkin_LIBRARIES}
)

## add nodelet
add_library(pandar_cloud_nodelet
  src/cloud_nodelet.cpp
  src/pandar_cloud.cpp
)

target_link_libraries(pandar_cloud_nodelet
  pandar_cloud
  ${catkin_LIBRARIES}
)


# Install
## executables and libraries
install(
  TARGETS
    pandar_cloud_node
    pandar_cloud_nodelet
    pandar_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## files
install(
  FILES
    nodelet_pandar_pointcloud.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## project namespaced headers
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
