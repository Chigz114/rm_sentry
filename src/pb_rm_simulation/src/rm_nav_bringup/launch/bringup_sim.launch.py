import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    # Get the launch directory
    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    pb_rm_simulation_launch_dir = os.path.join(get_package_share_directory('pb_rm_simulation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')

    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'config', 'simulation', 'measurement_params_sim.yaml')))
    robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'urdf', 'sentry_robot_sim.xacro'),
    ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################

    #################################### FAST_LIO parameters start ####################################
    fastlio_mid360_params = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'fastlio.rviz')
    ##################################### FAST_LIO parameters end #####################################

    # Declare launch options
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='False',
        description='Compatibility argument; Navigation2 is not launched by the lightweight sim bringup')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RM3V3',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Compatibility argument; current lightweight sim only starts Gazebo + FAST-LIO')

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fastlio',
        description='Choose LIO algorithm; current lightweight sim supports fastlio')

    # Specify the actions
    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pb_rm_simulation_launch_dir, 'rm_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description,
            'rviz': 'False'}.items()
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    bringup_LIO_group = GroupAction([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                # Useless arguments, provided by LIO in publish_odometry() function
                # '--x', '0.0',
                # '--y', '0.0',
                # '--z', '0.0',
                # '--roll', '0.0',
                # '--pitch', '0.0',
                # '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'lidar_odom'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[
                    fastlio_mid360_params,
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir],
            condition = IfCondition(use_lio_rviz),
            ),
        ])
    ])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_LIO_cmd)

    ld.add_action(start_rm_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_LIO_group)

    return ld
