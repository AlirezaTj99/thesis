import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_description_dir = get_package_share_directory('go2_description')
    go2_gazebo_dir     = get_package_share_directory('go2_gazebo')
    gazebo_ros_dir     = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir   = get_package_share_directory('nav2_bringup')

    urdf_xacro  = os.path.join(go2_description_dir, 'urdf', 'go2.urdf.xacro')
    world_file  = os.path.join(go2_gazebo_dir, 'worlds', 'go2_arena.world')
    nav2_params = os.path.join(go2_gazebo_dir, 'params', 'go2_nav2_params.yaml')
    rviz_config = os.path.join(go2_gazebo_dir, 'rviz', 'go2_nav2.rviz')

    robot_description = subprocess.check_output(['xacro', urdf_xacro], text=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart    = LaunchConfiguration('autostart',    default='true')

    # ── 1. Gazebo server ────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # ── 2. Gazebo client (GUI) ──────────────────────────────────────────────────
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        ),
    )

    # ── 3. Robot State Publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ── 4. Spawn Go2 (3 s — wait for gzserver) ─────────────────────────────────
    spawn_go2 = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_go2',
                output='screen',
                arguments=[
                    '-entity', 'go2',
                    '-topic', '/robot_description',
                    '-x', '-0.55', '-y', '-0.55', '-z', '0.01', '-Y', '0.0',
                ],
            )
        ],
    )

    # ── 5. Nav2 + SLAM (8 s delay) ─────────────────────────────────────────────
    # slam=True replaces AMCL+static-map with slam_toolbox.
    # slam_toolbox publishes map→odom TF as soon as the first laser scan arrives
    # (~1 s after robot spawns), so no manual initial-pose script is needed.
    # The robot builds its map live and saves it to disk automatically.
    nav2_bringup = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam':        'True',
                    'map':         '',
                    'use_sim_time': use_sim_time,
                    'params_file':  nav2_params,
                    'autostart':    autostart,
                    'default_bt_xml_filename':
                        '/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/'
                        'navigate_w_replanning_time.xml',
                }.items(),
            )
        ],
    )

    # ── 6. Scan filter (removes phantom self-detection returns in front arc) ────
    scan_filter = Node(
        package='go2_gazebo',
        executable='scan_filter.py',
        name='scan_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 7. Visualization node (goal circle + path arrows) ──────────────────────
    go2_viz = Node(
        package='go2_gazebo',
        executable='go2_viz.py',
        name='go2_viz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 8. RViz2 (9 s delay) ───────────────────────────────────────────────────
    rviz2 = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_go2,
        nav2_bringup,
        scan_filter,
        go2_viz,
        rviz2,
    ])
