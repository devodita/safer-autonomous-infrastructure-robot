cmake_minimum_required(VERSION 3.0.2)
project(safer_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  move_base_msgs
  safer_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/patrol_controller.py
  scripts/mpc_trajectory_tracker.py
  scripts/costmap_hazard_layer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
