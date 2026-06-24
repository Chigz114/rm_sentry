"""仿真感知流水线启动：height-gated traversability + 2D ESDF

接入 pb_rm_simulation 仿真环境，假设 FAST-LIO2 已在 bringup_sim.launch.py 中启动。

用法：
    # 终端1（已有）：ros2 launch rm_nav_bringup bringup_sim.launch.py ...
    # 终端2：ros2 launch rm_nav_bringup sim_perception.launch.py

输入：
    - /cloud_registered (sensor_msgs/PointCloud2) - FAST-LIO2 全局点云
    - /Odometry (nav_msgs/Odometry) - FAST-LIO2 位姿

输出：
    - /perception/costmap_2d (nav_msgs/OccupancyGrid) - height-gated 2.5D 占据图
    - /perception/esdf_2d (sensor_msgs/PointCloud2) - 2D ESDF 距离场

注意：
    - use_sim_time 默认 true，与仿真时钟同步
    - traversability_mapper 直接处理点云，不依赖 ROG-Map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包路径
    pkg_nav_bringup = FindPackageShare('rm_nav_bringup')
    rm3v3_world_file = PathJoinSubstitution(
        [FindPackageShare('pb_rm_simulation'), 'world', 'RM3V3', 'rm3v3_sym_v1.world']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    perception_rviz = LaunchConfiguration('perception_rviz')

    sim_perception_rviz_cfg = PathJoinSubstitution(
        [pkg_nav_bringup, 'rviz', 'sim_perception.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='仿真环境必须使用 /clock'),
        DeclareLaunchArgument('perception_rviz', default_value='False',
                              description='启动感知 RViz（sim_perception.rviz）'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='sim_perception_rviz',
            arguments=['-d', sim_perception_rviz_cfg],
            condition=IfCondition(perception_rviz),
            output='screen',
        ),

        # 0) Relocalization node (publishes map → lidar_odom TF)
        Node(
            package='sentry_perception',
            executable='relocalization',
            name='relocalization_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'seed_x': 5.5,
                    'seed_y': 3.5,
                    'seed_yaw': 0.15,
                    'world_file': rm3v3_world_file,
                    'accumulate_count': 30,
                    'refine_with_icp': True,
                    'tf_rate_hz': 50.0,
                    'cloud_topic': '/cloud_registered',
                },
            ],
        ),

        # 1) Height-gated traversability mapper (outputs costmap in map frame)
        Node(
            package='sentry_mapping',
            executable='traversability_mapper',
            name='traversability_mapper',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'resolution': 0.1,
                    'width_m': 13.0,
                    'height_m': 9.0,
                    'x_offset': 6.0,
                    'y_offset': 4.0,
                    'h_climb': 0.10,
                    'ground_clamp_lo': -0.30,
                    'ground_clamp_hi': -0.08,
                    'ground_init': -0.14,
                    'n_min_near': 1,
                    'n_min_mid': 1,
                    'n_min_far': 1,
                    'near_dist': 2.0,
                    'mid_dist': 4.0,
                    'delta_hit': 1.0,
                    'log_odds_cap': 2.0,
                    'log_odds_floor': -2.0,
                    'decay_tau': 3.0,
                    'occ_thresh': 0.5,
                    'rate_hz': 10.0,
                    'cloud_topic': '/cloud_registered',
                    'odom_topic': '/Odometry',
                    'costmap_topic': '/perception/costmap_2d',
                    'frame_id': 'map',
                    'odom_frame': 'lidar_odom',
                    'world_file': rm3v3_world_file,
                },
            ],
        ),

        # 2) 2D ESDF 节点（从 2.5D costmap 计算距离场）
        Node(
            package='sentry_perception',
            executable='esdf2d',
            name='esdf2d_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'costmap_topic': '/perception/costmap_2d',
                    'esdf_topic': '/perception/esdf_2d',
                    'demo_copy_topic': '/perception/esdf_2d_demo',
                    'demo_copy_enable': True,
                    'demo_copy_x_offset': 5.0,
                    'demo_copy_y_offset': 0.0,
                    'demo_copy_z_offset': 0.0,
                    'treat_unknown_as_obstacle': False,
                    'max_distance_m': 5.0,
                },
            ],
        ),
    ])
