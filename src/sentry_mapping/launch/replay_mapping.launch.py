"""
一键启动「bag 回放 + FAST-LIO2 + ROG-Map + RViz」。

用法:
    ros2 launch sentry_mapping replay_mapping.launch.py

    # 另一终端播 bag:
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
    bringup_pkg = FindPackageShare('sentry_bringup')
    mapping_pkg = FindPackageShare('sentry_mapping')

    fastlio_replay_launch = PathJoinSubstitution(
        [bringup_pkg, 'launch', 'mid360_fastlio_replay.launch.py'])
    mapping_launch = PathJoinSubstitution(
        [mapping_pkg, 'launch', 'mapping.launch.py'])
    rviz_cfg = PathJoinSubstitution(
        [mapping_pkg, 'rviz', 'mapping.rviz'])

    open_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动带 ROG-Map 话题显示的 RViz'),

        # 1) FAST-LIO2 replay（关掉它自带的 RViz，用本包的 mapping.rviz）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_replay_launch),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),

        # 2) ROG-Map 驱动 + static TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # 3) 第 4 层：2D ESDF 节点（订阅 costmap_2d，发布 esdf_2d）
        Node(
            package='sentry_perception',
            executable='esdf2d',
            name='esdf2d',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'costmap_topic': '/perception/costmap_2d',
                'esdf_topic':    '/perception/esdf_2d',
                'demo_copy_topic': '/perception/esdf_2d_demo',
                'demo_copy_enable': True,
                'demo_copy_x_offset': 5.0,
                'demo_copy_y_offset': 0.0,
                'demo_copy_z_offset': 0.0,
                'treat_unknown_as_obstacle': False,
                'max_distance_m': 5.0,
            }],
        ),

        # 4) RViz，显示 ROG-Map 输出 + FAST-LIO2 的 /cloud_registered
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
