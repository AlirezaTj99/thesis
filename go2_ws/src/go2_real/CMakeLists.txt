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
  scripts/velocity_combiner.py
  scripts/repulsion_gate.py
  scripts/stair_cloud_persist.py
  scripts/go2_goal.py
  scripts/go2_goal.sh
  scripts/run_goal.sh
  scripts/monitor_robot.py
  scripts/map_saver_node.py
  scripts/run_monitor_robot.sh
  scripts/start_go2_real.sh
  scripts/start_go2_repulsion.sh
  scripts/start_go2_mapping.sh
  scripts/setup_sdk.sh
  scripts/height_filter_map.py
  scripts/go2_panel.py
  scripts/body_filter.py
  scripts/stair_controller.py
  scripts/stair_perception.py
  scripts/map_repulsion.py
  scripts/auto_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
