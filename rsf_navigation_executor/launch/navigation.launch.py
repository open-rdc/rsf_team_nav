import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_executor_dir, 'config', 'tsudanuma2-3.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='true')
    keepout_mask_arg = DeclareLaunchArgument(
        'keepout_mask', default_value='',
        description='Keepout mask yaml for the global costmap KeepoutFilter (skipped when the file does not exist)')
    localization_arg = DeclareLaunchArgument(
        'localization', default_value='filter', choices=['filter', 'static'],
        description='filter: rsf_localization_filter publishes map->rsf_odom (GNSS + gyro dead reckoning, AMCL in GNSS gaps); '
                    'static: fixed map->rsf_odom from map_to_odom_x/y/yaw')
    use_mcl_stack_arg = DeclareLaunchArgument('use_mcl_stack', default_value='true')
    map_to_odom_x_arg = DeclareLaunchArgument('map_to_odom_x', default_value='0.0')
    map_to_odom_y_arg = DeclareLaunchArgument('map_to_odom_y', default_value='0.0')
    map_to_odom_yaw_arg = DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')
    localization = LaunchConfiguration('localization')

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
        condition=LaunchConfigurationEquals('localization', 'static'),
    )

    odom_to_footprint_node = Node(
        package='rsf_bringup',
        executable='tf_odom_to_footprint',
        name='tf_odom_to_footprint',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'input_topic': '/rsf/lio_lidar_rate_odom'}],
        condition=LaunchConfigurationEquals('localization', 'static'),
    )

    localization_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rsf_localization_filter'), 'launch', 'filter.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'use_mcl_stack': LaunchConfiguration('use_mcl_stack')}.items(),
        condition=LaunchConfigurationEquals('localization', 'filter'),
    )

    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ground_segmentation_ros2',
                plugin='GroundSegmentatioNode',
                name='ground_segmentation',
                parameters=[
                    os.path.join(navigation_executor_dir, 'config', 'ground_segmentation_params.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[('/ground_segmentation/input_pointcloud', '/rsf/hokuyo_cloud2')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan',
                parameters=[
                    os.path.join(navigation_executor_dir, 'config', 'pointcloud_to_laserscan_params.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('cloud_in', '/ground_segmentation/obstacle_points'),
                    ('scan', '/scan'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
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

    def map_nodes(context):
        mask = LaunchConfiguration('keepout_mask').perform(context)
        managed = ['map_server']
        nodes = []
        if mask and os.path.exists(mask):
            nodes += [
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='keepout_mask_server',
                    parameters=[{
                        'yaml_filename': mask,
                        'topic_name': '/keepout_filter_mask',
                        'use_sim_time': use_sim_time,
                    }],
                    output='screen',
                ),
                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='keepout_filter_info_server',
                    parameters=[{
                        'type': 0,
                        'filter_info_topic': '/costmap_filter_info',
                        'mask_topic': '/keepout_filter_mask',
                        'base': 0.0,
                        'multiplier': 1.0,
                        'use_sim_time': use_sim_time,
                    }],
                    output='screen',
                ),
            ]
            managed += ['keepout_mask_server', 'keepout_filter_info_server']
        nodes.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': managed,
            }],
            output='screen',
        ))
        return nodes

    params_file = RewrittenYaml(
        source_file=os.path.join(navigation_executor_dir, 'config', 'nav2_params.yaml'),
        param_rewrites={
            'use_sim_time': use_sim_time,
            'odom_topic': PythonExpression(["'/rsf_filter/odom' if '", localization, "' == 'filter' else '/rsf/rsf_odom'"]),
        },
        convert_types=True,
    )

    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[ParameterFile(params_file, allow_substs=True)],
        output='screen',
        condition=IfCondition(use_composition),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_executor_dir, 'launch', 'nav2_core.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
            'container_name': 'nav2_container',
            'params_file': params_file,
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
    launch_description.add_action(use_composition_arg)
    launch_description.add_action(keepout_mask_arg)
    launch_description.add_action(localization_arg)
    launch_description.add_action(use_mcl_stack_arg)
    launch_description.add_action(map_to_odom_x_arg)
    launch_description.add_action(map_to_odom_y_arg)
    launch_description.add_action(map_to_odom_yaw_arg)
    launch_description.add_action(map_to_odom_node)
    launch_description.add_action(odom_to_footprint_node)
    launch_description.add_action(localization_filter_launch)
    launch_description.add_action(perception_container)
    launch_description.add_action(map_server_node)
    launch_description.add_action(OpaqueFunction(function=map_nodes))
    launch_description.add_action(nav2_container)
    launch_description.add_action(navigation_launch)
    launch_description.add_action(rviz_node)

    return launch_description
