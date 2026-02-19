cmake_minimum_required(VERSION 3.0.2)
project(safer_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  actionlib_msgs
)

add_message_files(FILES
  HazardDetection.msg
  RiskAssessment.msg
  ModeCommand.msg
  ModeStatus.msg
  ActuatorCommand.msg
  AlertPayload.msg
)

add_service_files(FILES
  ClassifyHazard.srv
  GetRiskScore.srv
  TriggerMitigation.srv
)

add_action_files(FILES
  MitigateHazard.action
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    actionlib_msgs
)
