cmake_minimum_required(VERSION 3.0.2)
project(safer_actuation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  safer_msgs
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/actuation_manager_node.py
  scripts/motor_controller.py
  scripts/brake_controller.py
  scripts/alert_peripheral_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files and config
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
