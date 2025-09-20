cmake_minimum_required(VERSION 3.8)
project(altinet)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/camera_node
    scripts/camera_viewer_node
    scripts/detector_node
    scripts/tracker_node
    scripts/event_manager_node
    scripts/lighting_control_node
    scripts/ros2_django_bridge_node
    scripts/visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY altinet/launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY altinet/config
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
