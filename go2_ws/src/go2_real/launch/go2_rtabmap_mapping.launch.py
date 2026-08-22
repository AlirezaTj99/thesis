"""
go2_rtabmap_mapping.launch.py — 3D mapping with RTAB-Map

Drive the robot around all floors manually.  Ctrl+C when done —
RTAB-Map saves ~/maps/rtabmap.db automatically.

Usage:
  ros2 launch go2_real go2_rtabmap_mapping.launch.py

To start a brand-new map, delete the old DB first:
  rm ~/maps/rtabmap.db && ros2 launch go2_real go2_rtabmap_mapping.launch.py

Odometry pipeline (ter_go2 / Point-LIO method):
  /cloud_relay → icp_odometry (F2M, 30 iters) → /icp_odom → rtabmap
  go2_bridge /odom used only as the initial TF seed for icp_odometry.

TF tree produced:
  map → odom        (RTAB-Map, correcting icp_odom drift via loop closure)
  odom → base_link  (go2_bridge at 50 Hz)
  base_link → utlidar_lidar  (static)
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_description_dir = get_package_share_directory('go2_description')
    go2_real_dir        = get_package_share_directory('go2_real')

    urdf_xacro       = os.path.join(go2_description_dir, 'urdf', 'go2_real.urdf.xacro')
    rtab_params      = os.path.join(go2_real_dir, 'params', 'go2_rtabmap_params.yaml')
    icp_odom_params  = os.path.join(go2_real_dir, 'params', 'go2_icp_odom_params.yaml')
    rviz_cfg         = os.path.join(go2_real_dir, 'rviz', 'go2_mapping.rviz')

    robot_description = subprocess.check_output(['xacro', urdf_xacro], text=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_description,
        }],
    )

    utlidar_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='utlidar_lidar_tf',
        output='screen',
        arguments=['0.08', '0', '0.15', '0', '0', '0', 'base_link', 'utlidar_lidar'],
    )

    go2_bridge = Node(
        package='go2_real',
        executable='go2_bridge.py',
        name='go2_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # icp_odometry: refines go2_bridge LIO-SAM output using scan-to-local-map ICP.
    # Starts at 3 s so it is accumulating a local cloud map before rtabmap wakes.
    icp_odometry = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rtabmap_odom',
                executable='icp_odometry',
                name='icp_odometry',
                output='screen',
                parameters=[
                    icp_odom_params,
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('scan_cloud', '/cloud_relay'),
                    ('odom',       '/icp_odom'),    # publish refined odom here
                ],
            )
        ],
    )

    rtabmap = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[
                    rtab_params,
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('scan_cloud', '/cloud_relay'),
                    ('odom',       '/icp_odom'),    # use icp_odometry output
                    ('grid_map',   '/map'),
                ],
            )
        ],
    )

    rviz2 = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_mapping',
                output='screen',
                arguments=['-d', rviz_cfg],
                additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        robot_state_publisher,
        utlidar_lidar_tf,
        go2_bridge,
        icp_odometry,
        rtabmap,
        rviz2,
    ])
