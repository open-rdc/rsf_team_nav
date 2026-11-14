import glob
import math
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def world_nodes(event):
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')
    waypoints_dir = os.path.join(get_package_share_directory('rsf_waypoint_manager'), 'waypoints')
    origin = next(yaml.safe_load_all(event.text))
    sim_time = {'use_sim_time': True}
    nodes = []
    for map_yaml in glob.glob(os.path.join(navigation_executor_dir, 'config', '*.yaml')):
        world = os.path.basename(map_yaml)[:-len('.yaml')]
        waypoints_file = os.path.join(waypoints_dir, world + '_wp.yaml')
        if not os.path.exists(waypoints_file):
            continue
        with open(map_yaml) as f:
            x, y = yaml.safe_load(f)['origin'][:2]
        if math.hypot(x - origin['x'], y - origin['y']) > 0.01:
            continue
        nodes.append(Node(
            package='rsf_waypoint_manager',
            executable='logging_simulator',
            name='logging_simulator_waypoints',
            parameters=[sim_time, {'waypoints_file': waypoints_file}],
            remappings=[('~/waypoints', '/waypoint_navigator/waypoints')],
            output='screen',
        ))
        pointcloud_map = os.path.join(navigation_executor_dir, 'config', world + '.pcd')
        if os.path.exists(pointcloud_map):
            nodes.append(Node(
                package='rsf_3d_localization',
                executable='rsf_3d_localization_node',
                name='logging_simulator_pointcloud_map',
                parameters=[
                    os.path.join(
                        get_package_share_directory('rsf_3d_localization'), 'config',
                        'rsf_3d_localization.param.yaml'),
                    sim_time,
                    {'map_path': pointcloud_map, 'publish_tf': False},
                ],
                remappings=[('~/output/pointcloud_map', '/rsf_3d_localization/pointcloud_map')],
                output='screen',
            ))
    return nodes


def launch_setup(context):
    navigation_executor_dir = get_package_share_directory('rsf_navigation_executor')
    planning = LaunchConfiguration('planning').perform(context) == 'true'
    control = LaunchConfiguration('control').perform(context) == 'true'
    sim_time = {'use_sim_time': True}

    with open(os.path.join(navigation_executor_dir, 'rviz', 'navigation.rviz')) as f:
        rviz_config = f.read().replace(
            'Enabled: false\n    Name: Waypoints\n', 'Enabled: true\n    Name: Waypoints\n').replace(
            'Value: /waypoint_navigator/waypoints\n    Value: false\n',
            'Value: /waypoint_navigator/waypoints\n    Value: true\n')
    if planning:
        rviz_config = rviz_config.replace('Value: /plan\n', 'Value: /logging_simulator/plan\n')
    if control:
        rviz_config = rviz_config.replace('Topic: /cmd_vel\n', 'Topic: /logging_simulator/cmd_vel\n')
    with tempfile.NamedTemporaryFile('w', suffix='.rviz', delete=False) as f:
        f.write(rviz_config)

    map_origin = ExecuteProcess(cmd=[
        'ros2', 'topic', 'echo', '/map', '--once', '--field', 'info.origin.position',
        '--qos-durability', 'transient_local', '--qos-reliability', 'reliable'])
    nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', f.name],
            parameters=[sim_time],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[sim_time, {'robot_description': ParameterValue(
                Command(['xacro ', os.path.join(
                    get_package_share_directory('rsf_description'), 'urdf', 'orne_boxF.urdf.xacro')]),
                value_type=str)}],
            output='screen',
        ),
        Node(
            package='rsf_waypoint_manager',
            executable='logging_simulator',
            parameters=[sim_time, {'planning': planning, 'control': control}],
            output='screen',
        ),
        map_origin,
        RegisterEventHandler(OnProcessIO(target_action=map_origin, on_stdout=world_nodes)),
    ]
    if not (planning or control):
        return nodes

    params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(navigation_executor_dir, 'config', 'nav2_params.yaml'),
            param_rewrites={'use_sim_time': 'true', 'visualize': 'true'},
            convert_types=True,
        ),
        allow_substs=True,
    )
    managed = []
    if planning:
        nodes.append(Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[params],
            remappings=[('plan', '/logging_simulator/plan')],
            output='screen',
        ))
        managed.append('planner_server')
    if control:
        nodes += [
            ComposableNodeContainer(
                name='logging_simulator_perception_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[ComposableNode(
                    package='ground_segmentation_ros2',
                    plugin='GroundSegmentatioNode',
                    name='ground_segmentation',
                    parameters=[
                        os.path.join(navigation_executor_dir, 'config', 'ground_segmentation_params.yaml'),
                        sim_time,
                    ],
                    remappings=[('/ground_segmentation/input_pointcloud', '/rsf/hokuyo_cloud2')],
                )],
                output='screen',
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                parameters=[params],
                remappings=[('cmd_vel', '/logging_simulator/cmd_vel')],
                output='screen',
            ),
        ]
        managed.append('controller_server')
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_logging_simulator',
        parameters=[sim_time, {'autostart': False, 'node_names': managed}],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('control', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
