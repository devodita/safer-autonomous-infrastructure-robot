cmake_minimum_required(VERSION 3.0.2)
project(safer_comms)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  safer_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mqtt_alert_bridge.py
  scripts/dashboard_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
