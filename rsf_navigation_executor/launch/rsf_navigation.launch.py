import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tsukuba',
        choices=['tsudanuma2-3', 'tsukuba'],
    )
    world = LaunchConfiguration('world')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            navigation_executor_dir, 'config', [world, '.yaml']]),
    )
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('rsf_waypoint_manager'),
            'waypoints', [world, '_wp.yaml']]),
    )
    profiles_file_arg = DeclareLaunchArgument(
        'profiles_file',
        default_value=os.path.join(navigation_executor_dir, 'config', 'nav_profiles.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    map_to_odom_x_arg = DeclareLaunchArgument('map_to_odom_x', default_value='0.0')
    map_to_odom_y_arg = DeclareLaunchArgument('map_to_odom_y', default_value='0.0')
    map_to_odom_yaw_arg = DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0')

    map_yaml = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_executor_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_rviz': use_rviz,
            'map_to_odom_x': map_to_odom_x,
            'map_to_odom_y': map_to_odom_y,
            'map_to_odom_yaw': map_to_odom_yaw,
        }.items(),
    )

    waypoint_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rsf_waypoint_manager'),
                'launch', 'waypoint_navigation.launch.py')
        ),
        launch_arguments={
            'waypoints_file': waypoints_file,
            'profiles_file': LaunchConfiguration('profiles_file'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        waypoints_file_arg,
        profiles_file_arg,
        use_sim_time_arg,
        autostart_arg,
        use_rviz_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        navigation_launch,
        waypoint_navigation_launch,
    ])
