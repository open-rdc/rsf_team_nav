import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_executor_dir, 'maps', 'tsudanuma2-3.yaml'),
    )
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(
            navigation_executor_dir, 'waypoints', 'tsudanuma2-3_wp.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    map_yaml = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_executor_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_rviz': use_rviz,
        }.items(),
    )

    waypoint_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_executor_dir, 'launch', 'waypoint_navigation.launch.py')
        ),
        launch_arguments={
            'waypoints_file': waypoints_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        waypoints_file_arg,
        use_sim_time_arg,
        autostart_arg,
        use_rviz_arg,
        navigation_launch,
        waypoint_navigation_launch,
    ])
