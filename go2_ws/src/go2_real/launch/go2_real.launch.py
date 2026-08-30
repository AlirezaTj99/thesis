import os
import subprocess
import yaml

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
    nav2_params          = os.path.join(go2_real_dir, 'params', 'go2_nav2_real_params.yaml')
    rtab_params          = os.path.join(go2_real_dir, 'params', 'go2_rtabmap_params.yaml')
    rtab_loc_params      = os.path.join(go2_real_dir, 'params', 'go2_rtabmap_localization_params.yaml')
    icp_odom_params      = os.path.join(go2_real_dir, 'params', 'go2_icp_odom_params.yaml')
    rviz_config     = os.path.join(go2_real_dir, 'rviz', 'go2_nav2_real.rviz')

    robot_description = subprocess.check_output(['xacro', urdf_xacro], text=True)

    # Read desired_linear_vel from Nav2 params — used to set max repulsion velocity
    with open(nav2_params) as f:
        _params = yaml.safe_load(f)
    _desired_linear_vel = _params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']
    _max_repulsion_vel  = round(1.2 * _desired_linear_vel, 4)

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

    # ── 1b. utlidar_lidar alias TF ───────────────────────────────────────────────
    # The robot SDK publishes /cloud_relay with frame_id='utlidar_lidar', but the
    # URDF names the lidar frame 'laser_frame'. Publish an alias so RViz can
    # render the 3D point cloud with Fixed Frame=map.
    utlidar_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='utlidar_lidar_tf',
        output='screen',
        arguments=['0.08', '0', '0.15', '0', '0', '0', 'base_link', 'utlidar_lidar'],
    )

    # ── 2. Go2 Bridge — relays odom/cloud and broadcasts odom→base_link TF ────
    # The Go2 already publishes odom→base_link TF and /utlidar/cloud_base
    # via its internal DDS. This node only forwards Nav2 velocity commands.
    go2_bridge = Node(
        package='go2_real',
        executable='go2_bridge.py',
        name='go2_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 2b. Body self-filter ──────────────────────────────────────────────────────
    # Transforms each /cloud_relay point into base_link frame and removes any
    # point that falls inside the robot's body bounding box (body + full leg reach
    # during trot gait).  Publishes clean cloud to /cloud_filtered.
    # Only the costmap subscribes to /cloud_filtered; SLAM/ICP/laserscan still
    # use the full /cloud_relay so their quality is unaffected.
    body_filter = Node(
        package='go2_real',
        executable='body_filter.py',
        name='body_filter',
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

    # ── 4. ICP odometry — refines go2_bridge odom with scan-to-map ICP ─────────
    # Same pipeline as mapping: /cloud_relay → icp_odometry → /icp_odom → rtabmap.
    # Starts at 3 s so the local map accumulates before RTAB-Map wakes.
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
                    ('odom',       '/icp_odom'),
                ],
            )
        ],
    )

    # ── 4a. Auto initial pose — injects /initialpose from robot_initial_pose.yaml ─
    # Fires 9 s after launch (RTAB-Map starts at 5 s and needs ~4 s to load the DB).
    # Tells RTAB-Map exactly where the robot starts so it finds the matching nodes
    # immediately instead of waiting for a lucky loop closure.
    auto_initial_pose = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='go2_real',
                executable='auto_initial_pose.py',
                name='auto_initial_pose',
                output='screen',
            )
        ],
    )

    # ── 4b. RTAB-Map localization — loads rtabmap.db, publishes map→odom TF ───
    # Mem/IncrementalMemory=false: map is read-only (localization only).
    # In localization mode RTAB-Map loads the full DB at startup and publishes
    # the complete 2D occupancy grid projection to /map (TRANSIENT_LOCAL QoS),
    # so no separate map_server is needed.
    # Starts at 5 s (2 s after icp_odometry has cloud data to work with).
    rtabmap_localization = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[
                    rtab_params,       # base params (Mem/IncrementalMemory: "true" by default)
                    rtab_loc_params,   # override: Mem/IncrementalMemory=false, InitWMWithAllNodes=true
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('scan_cloud', '/cloud_relay'),
                    ('odom',       '/icp_odom'),
                    ('map',        '/rtabmap/grid_map_raw'),  # RTAB-Map publishes 'map' (relative) → intercept before Nav2
                ],
            )
        ],
    )

    # ── 4c. Height-filtered 2D map publisher ─────────────────────────────────────
    # Subscribes to /rtabmap/grid_map_raw (RTAB-Map's full projection) and
    # /cloud_map (3D obstacle voxels).  Removes obstacle cells that have no
    # voxel in [floor_z + min_h, floor_z + max_h] — eliminates ground noise.
    # Publishes clean /map with TRANSIENT_LOCAL QoS for Nav2.
    height_filter_map = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='go2_real',
                executable='height_filter_map.py',
                name='height_filter_map',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'floor_z': 0.0,   # floor Z in map frame (update per floor later)
                    'min_h':   0.20,  # ignore voxels below 20 cm above floor
                    'max_h':   2.00,  # ignore voxels above 2 m above floor
                }],
            )
        ],
    )

    # ── 5. Nav2 navigation stack (15 s delay) ───────────────────────────────────
    # Delayed to 15 s: RTAB-Map loads db at 5 s and needs several seconds to
    # finish loading all nodes, publish the full /map occupancy grid, and produce
    # a valid map→odom TF before Nav2 costmaps try to configure.
    nav2_bringup = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file':  nav2_params,
                    'autostart':    autostart,
                    'default_bt_xml_filename':
                        '/home/alireza/thesis1/navigation_go2/go2_ws/src/go2_real/params/'
                        'navigate_persistent.xml',
                    'map_subscribe_transient_local': 'true',
                }.items(),
            )
        ],
    )

    # ── 6. Obstacle Repulsion (5 s delay — wait for /scan to be available) ─────────
    # max_vel is set to 1.2 × Nav2 desired_linear_vel (read from params YAML above).
    # Output is remapped to /repulsion_vel_raw so repulsion_gate can zero it during
    # stair climbing without modifying obstacle_repulsion.py.
    obstacle_repulsion = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='go2_real',
                executable='obstacle_repulsion.py',
                name='obstacle_repulsion',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'max_vel': _max_repulsion_vel,
                    'deadband_vel': 0.1,
                }],
                remappings=[('/repulsion_vel', '/repulsion_vel_raw')],
            )
        ],
    )

    # ── 6b. Repulsion Gate — zeros repulsion while stair controller is active ────
    repulsion_gate = Node(
        package='go2_real',
        executable='repulsion_gate.py',
        name='repulsion_gate',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 6b-2. Map Repulsion (10 s delay — needs Nav2 global costmap + TF ready) ─────
    # Uses same max_vel as obstacle_repulsion; other params match obstacle_repulsion defaults.
    map_repulsion = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='go2_real',
                executable='map_repulsion.py',
                name='map_repulsion',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'max_vel': _max_repulsion_vel,
                    'deadband_vel': 0.1,
                }],
            )
        ],
    )

    # ── 6c. Velocity Combiner (3 s delay — needs go2_bridge + repulsion_gate up) ─
    velocity_combiner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='go2_real',
                executable='velocity_combiner.py',
                name='velocity_combiner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )


    # ── 6d. Stair Perception (5 s delay — needs /cloud_relay from go2_bridge) ────
    stair_perception = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='go2_real',
                executable='stair_perception.py',
                name='stair_perception',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    # ── 6e. Stair Controller (3 s delay — needs /odom and /imu/data) ─────────────
    stair_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='go2_real',
                executable='stair_controller.py',
                name='stair_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    # ── 7. Visualization (goal circle + path arrows) ─────────────────────────────
    go2_viz = Node(
        package='go2_gazebo',
        executable='go2_viz.py',
        name='go2_viz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 8. RViz2 (5 s delay) ─────────────────────────────────────────────────────
    # LIBGL_ALWAYS_SOFTWARE=1 avoids GL vertex-buffer crashes from exhausted
    # hardware OpenGL contexts left by prior sessions.
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
                additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        utlidar_lidar_tf,
        go2_bridge,
        body_filter,
        pointcloud_to_laserscan,
        icp_odometry,
        auto_initial_pose,
        rtabmap_localization,
        height_filter_map,
        nav2_bringup,
        obstacle_repulsion,
        repulsion_gate,
        map_repulsion,
        velocity_combiner,
        stair_perception,
        stair_controller,
        go2_viz,
        rviz2,
    ])
