import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_description_dir = get_package_share_directory('go2_description')
    go2_real_dir        = get_package_share_directory('go2_real')
    nav2_bringup_dir    = get_package_share_directory('nav2_bringup')

    urdf_xacro  = os.path.join(go2_description_dir, 'urdf', 'go2_real.urdf.xacro')
    nav2_params = os.path.join(go2_real_dir, 'params', 'go2_nav2_real_params.yaml')
    rviz_config = os.path.join(go2_real_dir, 'rviz', 'go2_nav2_real.rviz')

    robot_description = subprocess.check_output(['xacro', urdf_xacro], text=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart    = LaunchConfiguration('autostart',    default='true')

    # ── 1. Robot State Publisher ─────────────────────────────────────────────────
    # Publishes static TFs: base_link→laser_frame, base_link→leg_links.
    # Does NOT publish odom→base_link — that comes from go2_bridge.py.
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

    # ── 2. Go2 Bridge (cmd_vel → sport API only) ────────────────────────────────
    # The Go2 already publishes odom→base_link TF and /utlidar/cloud_base
    # via its internal DDS. This node only forwards Nav2 velocity commands.
    go2_bridge = Node(
        package='go2_real',
        executable='go2_bridge.py',
        name='go2_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 3. PointCloud2 → LaserScan ───────────────────────────────────────────────
    # The Go2 publishes /utlidar/cloud_base (PointCloud2, frame: base_link).
    # Convert to LaserScan for Nav2 and SLAM toolbox.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_relay'),
            ('scan',     '/scan'),
        ],
        parameters=[{
            'use_sim_time':        use_sim_time,
            'target_frame':        'base_link',
            'transform_tolerance': 0.1,
            'min_height':  -0.10,
            'max_height':   0.50,
            'angle_min':   -3.14159265,
            'angle_max':    3.14159265,
            'angle_increment': 0.01745329,
            'scan_time':    0.1,
            'range_min':    0.30,
            'range_max':   30.0,
            'use_inf':      True,
        }],
    )

    # ── 4. Nav2 + SLAM (4 s delay — wait for TF from robot) ─────────────────────
    nav2_bringup = TimerAction(
        period=4.0,
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

    # ── 5. Obstacle Repulsion (5 s delay — wait for /scan to be available) ─────────
    obstacle_repulsion = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='go2_real',
                executable='obstacle_repulsion.py',
                name='obstacle_repulsion',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    # ── 6. Visualization (goal circle + path arrows) ─────────────────────────────
    go2_viz = Node(
        package='go2_gazebo',
        executable='go2_viz.py',
        name='go2_viz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 7. RViz2 (5 s delay) ─────────────────────────────────────────────────────
    rviz2 = TimerAction(
        period=5.0,
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
        robot_state_publisher,
        go2_bridge,
        pointcloud_to_laserscan,
        nav2_bringup,
        obstacle_repulsion,
        go2_viz,
        rviz2,
    ])
