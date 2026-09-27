import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_executor_dir, 'config', 'tsudanuma2-3.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    map_to_odom_x_arg = DeclareLaunchArgument('map_to_odom_x', default_value='0.0')
    map_to_odom_y_arg = DeclareLaunchArgument('map_to_odom_y', default_value='0.0')
    map_to_odom_yaw_arg = DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')

    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=[
            '--x', map_to_odom_x,
            '--y', map_to_odom_y,
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', map_to_odom_yaw,
            '--frame-id', 'map',
            '--child-frame-id', 'rsf_odom',
        ],
        output='screen',
    )

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

    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
        output='screen',
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': os.path.join(navigation_executor_dir, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(navigation_executor_dir, 'rviz', 'navigation.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(map_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(autostart_arg)
    launch_description.add_action(use_rviz_arg)
    launch_description.add_action(map_to_odom_x_arg)
    launch_description.add_action(map_to_odom_y_arg)
    launch_description.add_action(map_to_odom_yaw_arg)
    launch_description.add_action(map_to_odom_node)
    launch_description.add_action(pointcloud_to_laserscan_node)
    launch_description.add_action(map_server_node)
    launch_description.add_action(lifecycle_manager_map_node)
    launch_description.add_action(navigation_launch)
    launch_description.add_action(rviz_node)

    return launch_description
