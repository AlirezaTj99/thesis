cmake_minimum_required(VERSION 3.5)
project(go2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params maps rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go2_viz.py
  scripts/scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
