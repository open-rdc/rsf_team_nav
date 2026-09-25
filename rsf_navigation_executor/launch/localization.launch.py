import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_executor_dir, 'maps', 'tsudanuma2-3.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[
            os.path.join(navigation_executor_dir, 'config', 'pointcloud_to_laserscan_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/rsf/hokuyo_cloud2'),
            ('scan', '/scan'),
        ],
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    emcl2_node = Node(
        package='emcl2',
        executable='emcl2_node',
        name='emcl2_node',
        parameters=[
            os.path.join(navigation_executor_dir, 'config', 'emcl2_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
        output='screen',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(map_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(autostart_arg)
    launch_description.add_action(pointcloud_to_laserscan_node)
    launch_description.add_action(map_server_node)
    launch_description.add_action(emcl2_node)
    launch_description.add_action(lifecycle_manager_localization_node)

    return launch_description
