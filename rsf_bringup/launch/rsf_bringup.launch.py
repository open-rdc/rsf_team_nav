import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('rsf_bringup'),
        'config',
        'bringup_params.yaml'
    )

    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    sim_arg = DeclareLaunchArgument('sim', default_value=str(launch_params['sim']).lower())
    sim = LaunchConfiguration('sim')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file_path],
        output='screen',
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file_path],
        output='screen',
    )
    tf_odom_to_footprint_node = Node(
        package='rsf_bringup',
        executable='tf_odom_to_footprint',
        name='tf_odom_to_footprint',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rsf_description'), 'launch', 'display.launch.py')),
        launch_arguments={'use_sim_time': sim}.items(),
    )

    launch_description = LaunchDescription()
    launch_description.add_action(sim_arg)
    launch_description.add_action(display_launch)
    launch_description.add_action(tf_odom_to_footprint_node)
    launch_description.add_action(joy_node)
    launch_description.add_action(teleop_node)
    icart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('icart_driver'), 'launch', 'icart_drive.launch.py'])),
        condition=UnlessCondition(sim),
    )
    hokuyo_rsf_node = Node(
        package='hokuyo_rsf',
        executable='hokuyo_rsf',
        name='hokuyo_rsf',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('hokuyo_rsf'), 'config', 'hokuyo_rsf.yaml']),
            {'param_files_dir': PathJoinSubstitution([
                FindPackageShare('hokuyo_rsf'), 'config'])},
            {'broadcast_tf': False},
        ],
        condition=UnlessCondition(sim),
    )
    launch_description.add_action(icart_launch)
    launch_description.add_action(hokuyo_rsf_node)

    return launch_description
