"""一键启动「bag 回放 + FAST-LIO2 + 障碍物检测 + 聚类 + RViz」。

用法：
    ros2 launch sentry_perception replay_perception.launch.py

再另一终端播 bag：
    ./src/sentry_bringup/scripts/play_bag.sh data/bags/static_empty_90s 1.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg    = FindPackageShare('sentry_bringup')
    perception_pkg = FindPackageShare('sentry_perception')

    fastlio_replay_launch = PathJoinSubstitution(
        [bringup_pkg, 'launch', 'mid360_fastlio_replay.launch.py'])
    perception_launch     = PathJoinSubstitution(
        [perception_pkg, 'launch', 'perception.launch.py'])
    rviz_cfg              = PathJoinSubstitution(
        [perception_pkg, 'rviz', 'perception.rviz'])

    # 注意：用 'rviz' 而不是复用 'use_rviz'，因为 IncludeLaunchDescription 的
    # launch_arguments 会污染顶层 LaunchConfiguration 值。
    open_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动带 obstacle 显示的 RViz'),

        # 1) FAST-LIO2 replay（关掉它自带的 RViz，我们用 perception.rviz）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_replay_launch),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),

        # 2) Perception pipeline（obstacle_detector + cluster_detector）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # 3) RViz，带 obstacle_cloud + obstacle_markers 显示
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(open_rviz),
            output='screen',
        ),
    ])
