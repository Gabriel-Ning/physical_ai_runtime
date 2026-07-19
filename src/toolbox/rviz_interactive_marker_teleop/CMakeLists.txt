cmake_minimum_required(VERSION 3.16)
project(rviz_interactive_marker_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/target_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
