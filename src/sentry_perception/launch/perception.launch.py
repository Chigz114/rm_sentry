"""启动感知流水线：obstacle_detector + cluster_detector。
假设 FAST-LIO2 已经在别处跑起来
（例如 `ros2 launch sentry_bringup mid360_fastlio.launch.py` 或 replay 版本）。

用法：
    ros2 launch sentry_perception perception.launch.py
    ros2 launch sentry_perception perception.launch.py use_sim_time:=true  # 回放 bag 时
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sentry_perception')
    detector_cfg = PathJoinSubstitution([pkg, 'config', 'obstacle_detector.yaml'])
    cluster_cfg  = PathJoinSubstitution([pkg, 'config', 'cluster_detector.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true 时用 /clock（bag 回放场景）'),

        # 1) 点云过滤：ROI + voxel + ground
        Node(
            package='sentry_perception',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            emulate_tty=True,
            parameters=[detector_cfg, {'use_sim_time': use_sim_time}],
        ),

        # 2) 欧式聚类：cluster → BoundingBox + centroid
        Node(
            package='sentry_perception',
            executable='cluster_detector',
            name='cluster_detector',
            output='screen',
            emulate_tty=True,
            parameters=[cluster_cfg, {'use_sim_time': use_sim_time}],
        ),
    ])
