"""
只启动 perception_mapper（ROG-Map 驱动节点），假设 FAST-LIO2 已经在别处跑。

用法:
    ros2 launch sentry_mapping mapping.launch.py                 # 实机（use_sim_time=false）
    ros2 launch sentry_mapping mapping.launch.py use_sim_time:=true  # bag 回放

坐标系说明:
    ROG-Map 源码里硬编码发布 frame_id="world" 的话题和 world→drone 的 TF，
    而 FAST-LIO2 发 camera_init→body 的 TF。为了让 RViz 以 camera_init 为
    Fixed Frame 时也能看到 ROG-Map 的话题，需要一条 static TF 把二者桥接：

        camera_init ───(static, identity)──→ world          ← 本 launch 提供

    这样 world 系下的一切（ROG-Map 的 occ / inf_occ / esdf 等）自动在 camera_init 下可见。
    drone 帧独立存在但没人关心。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_mapping')
    default_cfg = os.path.join(pkg_share, 'config', 'rog_map_sentry.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cfg_path     = LaunchConfiguration('cfg_path')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true 时用 /clock（bag 回放场景）'),
        DeclareLaunchArgument('cfg_path', default_value=default_cfg,
                              description='rog_map yaml 配置绝对路径'),

        # 1) static TF: camera_init → world (identity)
        #    让 ROG-Map 发的 world 系话题等价于 camera_init 系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_init_to_world',
            arguments=['0', '0', '0',   # x y z
                       '0', '0', '0',   # roll pitch yaw
                       'camera_init', 'world'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 2) perception_mapper（内含 ROG-Map）
        Node(
            package='sentry_mapping',
            executable='perception_mapper',
            name='perception_mapper',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'cfg_path': cfg_path,
                'use_sim_time': use_sim_time,
                # 2.5D costmap 参数（运行时 override，不需要 rebuild）
                'costmap2d.resolution':  0.05,
                'costmap2d.width_m':     4.0,
                'costmap2d.height_m':    5.5,
                'costmap2d.z_low':      -0.1,
                'costmap2d.z_high':      0.7,
                'costmap2d.z_step':      0.1,
                'costmap2d.rate_hz':    15.0,
                'costmap2d.x_offset':   -0.5,
                'costmap2d.y_offset':    0.0,
                'costmap2d.z_offset':    0.0,
                'costmap2d.use_inflate': False,   # 用非膨胀版，膨胀交给 planner；避免寝室小空间被膨胀吃掉 free 区
            }],
        ),
    ])
