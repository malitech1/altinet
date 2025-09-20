cmake_minimum_required(VERSION 3.8)
project(altinet_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Event.msg"
  "msg/PersonDetection.msg"
  "msg/PersonDetections.msg"
  "msg/PersonTrack.msg"
  "msg/PersonTracks.msg"
  "msg/RoomPresence.msg"
)

set(srv_files
  "srv/ManualLightOverride.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
