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

    urdf_xacro  = os.path.join(go2_description_dir, 'urdf', 'go2_real.urdf.xacro')
    slam_params = os.path.join(go2_real_dir, 'params', 'go2_slam_params.yaml')
    ekf_params  = os.path.join(go2_real_dir, 'params', 'go2_ekf.yaml')
    rviz_config = os.path.join(go2_real_dir, 'rviz', 'go2_mapping.rviz')

    robot_description = subprocess.check_output(['xacro', urdf_xacro], text=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Optional: path to a previously serialized pose graph (no extension).
    # When set, SLAM Toolbox loads the graph and continues mapping from it.
    map_file = LaunchConfiguration('map_file', default='')

    # ── 1. Robot State Publisher ─────────────────────────────────────────────────
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

    # ── 1b. utlidar_lidar alias TF ───────────────────────────────────────────────
    utlidar_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='utlidar_lidar_tf',
        output='screen',
        arguments=['0.08', '0', '0.15', '0', '0', '0', 'base_link', 'utlidar_lidar'],
    )

    # ── 2. Go2 Bridge ────────────────────────────────────────────────────────────
    # Relays odom→base_link TF and /cloud_relay. No Nav2 cmd_vel during mapping —
    # user drives manually with the physical controller.
    go2_bridge = Node(
        package='go2_real',
        executable='go2_bridge.py',
        name='go2_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 3. EKF — fuses /odom + /imu/data → publishes TF odom→base_link ─────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    # ── 4. PointCloud2 → LaserScan ───────────────────────────────────────────────
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

    # ── 5. SLAM Toolbox — online async mapping (4 s delay, wait for TF) ─────────
    # Using a direct Node (not IncludeLaunchDescription) so we can pass map_file_name
    # dynamically for lifelong mapping (continues from the previous session's pose graph).
    slam_toolbox = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {
                        'use_sim_time':       use_sim_time,
                        'map_file_name':      map_file,
                        'map_start_at_dock':  False,
                    },
                ],
            )
        ],
    )

    # ── 6. Map Saver — subscribes to /map, saves .pgm+.yaml on Ctrl+C ──────────
    # Runs inside the launch stack so it shares the same DDS domain — no new
    # CycloneDDS node needed at shutdown time (avoids interface init failures).
    map_saver = Node(
        package='go2_real',
        executable='map_saver_node.py',
        name='map_saver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 7. RViz2 — top-down map view (5 s delay) ─────────────────────────────────
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
        utlidar_lidar_tf,
        go2_bridge,
        ekf_node,
        pointcloud_to_laserscan,
        slam_toolbox,
        map_saver,
        rviz2,
    ])
