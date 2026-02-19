cmake_minimum_required(VERSION 3.0.2)
project(safer_decision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  safer_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/risk_scorer.py
  scripts/mode_fsm.py
  scripts/risk_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
