#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition

# Enum for world types
class WorldType:
    RM3V3 = 'RM3V3'

def get_world_config(world_type):
    world_configs = {
        WorldType.RM3V3: {
            'x': '6.0',
            'y': '4.0',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RM3V3/rm3v3_sym_v1.world',
        }
    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    default_robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('pb_rm_simulation'), 'urdf', 'simulation_waking_robot.xacro')])

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    robot_description = LaunchConfiguration('robot_description')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RM3V3,
        description='Choose <RM3V3>'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        world_path = world_config['world_path']
        if not os.path.isabs(world_path):
            world_path = os.path.join(bringup_dir, 'world', world_path)

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw']
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': world_path}.items(),
                )
            ]
        )

    bringup_RM3V3_cmd_group = create_gazebo_launch_group(WorldType.RM3V3)

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_RM3V3_cmd_group) # type: ignore

    # Uncomment this line if you want to start RViz
    ld.add_action(start_rviz_cmd)

    return ld
