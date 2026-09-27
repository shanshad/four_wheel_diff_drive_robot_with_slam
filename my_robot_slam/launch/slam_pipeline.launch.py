"""
slam_pipeline.launch.py  — ROS 2 Jazzy
Full SLAM pipeline in a single launch file.

Pipeline:
  Static TFs → rf2o (/odom_rf2o) → EKF (/odometry/filtered) → slam_toolbox → /map
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

LASER_FRAME = 'laser_frame'
IMU_FRAME   = 'imu_link'
BASE_FRAME  = 'base_link'
ODOM_FRAME  = 'odom'
MAP_FRAME   = 'map'


def generate_launch_description():

    pkg_share   = get_package_share_directory('my_robot_slam')
    ekf_config  = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rf2o_config = os.path.join(pkg_share, 'config', 'rf2o_params.yaml')

    for f in [ekf_config, slam_config, rf2o_config]:
        if not os.path.exists(f):
            raise RuntimeError(f'Missing config: {f}\nRun: colcon build --packages-select my_robot_slam')

    # ═══ 1. STATIC TF : base_link → laser_frame ════════════════════════════ #
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', BASE_FRAME,
            '--child-frame-id', LASER_FRAME,
        ],
        output='screen'
    )

    # ═══ 2. STATIC TF : base_link → imu_link ═══════════════════════════════ #
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', '0.05', '--y', '0.0', '--z', '0.23',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', BASE_FRAME,
            '--child-frame-id', IMU_FRAME,
        ],
        output='screen'
    )

    # ═══ 3. rf2o — laser odometry ═══════════════════════════════════════════ #
    # Reads /scan, publishes /odom_rf2o
    # publish_tf: false — EKF owns the odom→base_link TF
    # init_pose_from_topic: "" — start at origin immediately (no sim ground truth)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_config],
    )

    # ═══ 4. EKF — fuses rf2o + IMU ══════════════════════════════════════════ #
    # Subscribes: /odom_rf2o + /imu/data
    # Publishes:  /odometry/filtered + TF odom→base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # ═══ 5. slam_toolbox ════════════════════════════════════════════════════ #
    # Subscribes: /scan + TF tree
    # Publishes:  /map  + TF map→odom
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'base_frame':   BASE_FRAME,
                'odom_frame':   ODOM_FRAME,
                'map_frame':    MAP_FRAME,
                'scan_topic':   '/scan',
                'use_sim_time': False,
                'mode':         'mapping',
            }
        ],
    )

    # slam_toolbox is a lifecycle node — configure then activate after it starts
    slam_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )
    slam_activate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    # ═══ 6. RViz2 ════════════════════════════════════════════════════════════ #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    startup_msg = LogInfo(msg=(
        '\n'
        '============================================================\n'
        '  SLAM Pipeline — ROS 2 Jazzy\n'
        '------------------------------------------------------------\n'
        f'  rf2o  config : {rf2o_config}\n'
        f'  ekf   config : {ekf_config}\n'
        f'  slam  config : {slam_config}\n'
        '------------------------------------------------------------\n'
        '  t=0s   static TFs (base→laser, base→imu)\n'
        '  t=2s   rf2o       (/scan → /odom_rf2o)\n'
        '  t=4s   EKF        (/odom_rf2o + /imu/data → /odometry/filtered)\n'
        '  t=6s   slam_toolbox (/scan + TF → /map)\n'
        '  t=9s   lifecycle configure + activate\n'
        '  t=12s  RViz2\n'
        '============================================================\n'
    ))

    return LaunchDescription([
        startup_msg,

        # t=0 — static TFs first, everything depends on them
        static_tf_lidar,
        static_tf_imu,

        # t=2 — rf2o needs static TFs in buffer before first scan
        TimerAction(period=2.0,  actions=[rf2o_node]),

        # t=4 — EKF needs /odom_rf2o to exist
        TimerAction(period=4.0,  actions=[ekf_node]),

        # t=6 — slam_toolbox needs full TF chain: map→odom→base_link→laser
        TimerAction(period=6.0,  actions=[slam_node]),

        # t=9/10 — lifecycle activate (sync_slam_toolbox_node needs this)
        TimerAction(period=9.0,  actions=[slam_configure]),
        TimerAction(period=10.0, actions=[slam_activate]),

        # t=12 — RViz last so everything is ready
        TimerAction(period=12.0, actions=[rviz_node]),
    ])
