cmake_minimum_required(VERSION 3.0.2)
project(safer_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  safer_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs geometry_msgs
    cv_bridge image_transport pcl_ros safer_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# C++ hazard detector node
add_executable(hazard_detector_node
  src/hazard_detector_node.cpp
  src/depth_processor.cpp
  src/ultrasonic_filter.cpp
)

target_link_libraries(hazard_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(hazard_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Python scripts
catkin_install_python(PROGRAMS
  scripts/hazard_classifier.py
  scripts/pedestrian_density_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hazard_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
