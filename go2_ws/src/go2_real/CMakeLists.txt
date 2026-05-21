cmake_minimum_required(VERSION 3.5)
project(go2_real)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch params rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/go2_bridge.py
  scripts/obstacle_repulsion.py
  scripts/go2_goal.py
  scripts/go2_goal.sh
  scripts/start_go2_real.sh
  scripts/start_go2_repulsion.sh
  scripts/setup_sdk.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
