from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

NAV2_SERVERS = [
    ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer', 'controller_server', True),
    ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer', 'planner_server', False),
    ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer', 'behavior_server', True),
    ('nav2_velocity_smoother', 'velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother', True),
    ('nav2_collision_monitor', 'collision_monitor', 'nav2_collision_monitor::CollisionMonitor', 'collision_monitor', False),
    ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', False),
    ('nav2_waypoint_follower', 'waypoint_follower', 'nav2_waypoint_follower::WaypointFollower', 'waypoint_follower', False),
]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [name for _, _, _, name, _ in NAV2_SERVERS]
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def remappings_for(remap_cmd_vel):
        return tf_remappings + ([('cmd_vel', 'cmd_vel_nav')] if remap_cmd_vel else [])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    nodes = [
        Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings_for(remap_cmd_vel),
        )
        for package, executable, _, name, remap_cmd_vel in NAV2_SERVERS
    ]
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}],
    ))

    composable_nodes = [
        ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[configured_params],
            remappings=remappings_for(remap_cmd_vel),
        )
        for package, _, plugin, name, remap_cmd_vel in NAV2_SERVERS
    ]
    composable_nodes.append(ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation',
        parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}],
    ))

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[SetParameter('use_sim_time', use_sim_time), *nodes],
        ),
        GroupAction(
            condition=IfCondition(use_composition),
            actions=[
                SetParameter('use_sim_time', use_sim_time),
                LoadComposableNodes(
                    target_container=(namespace, '/', container_name),
                    composable_node_descriptions=composable_nodes,
                ),
            ],
        ),
    ])
