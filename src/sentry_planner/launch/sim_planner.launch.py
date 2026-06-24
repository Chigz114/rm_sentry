"""Stage 3 planner pipeline: costmap inflation + JPS + MINCO + traj_tracker

Launches:
  1. costmap_inflator — subscribes /perception/costmap_2d, publishes /planner/costmap_inflated
  2. jps_node — subscribes /planner/costmap_inflated + /goal_pose + /odom, publishes /planner/path
  3. minco_planner_node — subscribes /planner/path + /perception/costmap_2d, publishes /planner/traj_samples
  4. traj_tracker — subscribes /planner/traj_samples + /odom, publishes /cmd_vel_chassis

Usage:
    # Terminal 1: Gazebo + FAST-LIO2
    ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping
    # Terminal 2: Perception
    ros2 launch rm_nav_bringup sim_perception.launch.py
    # Terminal 3: Planner + Controller
    ros2 launch sentry_planner sim_planner.launch.py
    # In RViz: use "2D Nav Goal" tool to click a target
    #   → JPS path appears (green line)
    #   → MINCO trajectory appears
    #   → traj_tracker follows the timed trajectory
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1) Costmap inflation
        Node(
            package='sentry_planner',
            executable='costmap_inflator',
            name='costmap_inflator',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'inflation_radius_m': 0.30,
                'input_topic': '/perception/costmap_2d',
                'output_topic': '/planner/costmap_inflated',
            }],
        ),

        # 2) JPS path planner
        Node(
            package='sentry_planner',
            executable='jps_node',
            name='jps_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'costmap_topic': '/planner/costmap_inflated',
                'goal_topic': '/goal_pose',
                'odom_topic': '/odom',
                'use_raw_odom': True,
                'path_topic': '/planner/path',
                'max_iter': 200000,
                'goal_tolerance_m': 0.15,
            }],
        ),

        # 3) MINCO planner node (stage 3.4: generates smooth trajectory)
        Node(
            package='sentry_planner',
            executable='minco_planner_node',
            name='minco_planner_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'v_max': 6.0,
                'a_max': 16.0,
                'v_alloc': 3.0,
                'w_smooth': 1.0,
                'w_time': 100.0,
                'w_obs': 3000.0,
                'w_collision': 10000.0,
                'd_soft': 0.50,
                'd_hard': 0.25,
                'w_ref': 20.0,
                'waypoint_bound_m': 1.00,
                'w_dyn': 500.0,
                'min_spacing': 0.8,
                'max_spacing': 1.5,
                'sample_dt': 0.05,
                'max_iter': 150,
                't_min': 1.0,
                'odom_topic': '/odom',
                'traj_samples_topic': '/planner/traj_samples',
            }],
        ),

        # 4) Time-parameterized trajectory tracker — follows /planner/traj_samples
        Node(
            package='sentry_controller',
            executable='traj_tracker',
            name='traj_tracker',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'v_max': 6.0,
                'acc_lim': 12.0,
                'kp_pos': 2.0,
                'kd_vel': 0.8,
                'k_yaw': 0.0,
                'w_max': 0.0,
                'goal_tol': 0.18,
                'rate_hz': 30.0,
                'odom_topic': '/odom',
                'cmd_vel_topic': '/cmd_vel_chassis',
                'traj_topic': '/planner/traj_samples',
                'time_scale': 1.0,
                'progress_anchor': True,
                'lookahead_time': 0.25,
                'max_feedback_speed': 2.0,
                'track_slow_start': 0.25,
                'track_slow_stop': 0.80,
                'offtrack_v_max': 2.0,
                'debug_traj': True,
                'debug_csv_path': '/tmp/traj_tracker_debug.csv',
            }],
        ),
    ])
