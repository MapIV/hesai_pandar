cmake_minimum_required(VERSION 3.0.2)
project(pandar_driver)

add_compile_options(-std=c++14)

include(cmake/FindPCAP.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  roslib
  nodelet
  pluginlib
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
    pandar_input
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
add_library(pandar_input
  src/lib/socket_input.cpp
  src/lib/pcap_input.cpp
)
target_link_libraries(pandar_input
  pcap
  ${catkin_LIBRARIES}
)

## add node
add_executable(pandar_driver_node
  src/driver/node.cpp
  src/driver/pandar_driver.cpp
)

target_link_libraries(pandar_driver_node
  pandar_input  
  ${catkin_LIBRARIES}
)

## add nodelet
add_library(pandar_driver_nodelet
  src/driver/nodelet.cpp
  src/driver/pandar_driver.cpp
)

target_link_libraries(pandar_driver_nodelet
  pandar_input
  ${catkin_LIBRARIES}
)

## pcap
add_executable(pcap2rosbag_converter
  src/driver/pcap2rosbag_converter.cpp
)
target_include_directories(pcap2rosbag_converter PRIVATE
  ${PCAP_INCLUDE_DIR}
)
target_link_libraries(pcap2rosbag_converter
  ${PCAP_LIBRARY}
  ${catkin_LIBRARIES}
)


# Install
## executables and libraries
install(
  TARGETS
    pandar_driver_node
    pandar_driver_nodelet
    pandar_input
    pcap2rosbag_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## files
install(
  FILES
    nodelet_pandar_driver.xml
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
