cmake_minimum_required(VERSION 3.8)
project(rover_x1_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObservationHeader.msg"
  "msg/ObstacleProfile.msg"
  "msg/LaneObservation.msg"
  "msg/StopLineObservation.msg"
  "msg/TrafficLightObservation.msg"
  "msg/TaskIdentity.msg"
  "msg/MotionResult.msg"
  "msg/VoiceEvent.msg"
  "msg/VoiceResult.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
