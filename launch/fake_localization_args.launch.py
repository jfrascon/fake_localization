import ros2_launch_helpers as rlh
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node

from fake_localization.launch_utils import to_float
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='namespace'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('global_frame', default_value='map', description='Global frame'),
            DeclareLaunchArgument(
                'odometry_frame', default_value='robot_odom', description='Odometry frame for the robot'
            ),
            DeclareLaunchArgument(
                'robot_base_frame',
                default_value='robot_base_footprint_link',
                description='Base frame name for the robot',
            ),
            DeclareLaunchArgument('delta_x', default_value='0.0', description='Offset in x'),
            DeclareLaunchArgument('delta_y', default_value='0.0', description='Offset in y'),
            DeclareLaunchArgument('delta_yaw', default_value='0.0', description='Offset in yaw'),
            DeclareLaunchArgument(
                'transform_tolerance', default_value='0.1', description='Tolerance to consider transforms as up-to-date'
            ),
            DeclareLaunchArgument('node_name', default_value='fake_localization', description='Node name'),
            # Remappings can be applied to the following topics:
            # amcl_pose, base_pose_ground_truth, initialpose, particlecloud.
            DeclareLaunchArgument(
                'node_remappings', default_value=rlh.default_node_remappings_json_str(), description=rlh.REMAPPINGS_DESC
            ),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_json_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_logging_options',
                default_value=rlh.default_node_logging_options_json_str(),
                description=rlh.LOGGING_OPTIONS_DESC,
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    use_sim_time_lc = LaunchConfiguration('use_sim_time')
    use_sim_time = perform_typed_substitution(ctx, normalize_typed_substitution(use_sim_time_lc, bool), bool)

    parameters = [
        {
            'use_sim_time': use_sim_time,
            'global_frame': LaunchConfiguration('global_frame').perform(ctx),
            'odometry_frame': LaunchConfiguration('odometry_frame').perform(ctx),
            'base_frame': LaunchConfiguration('robot_base_frame').perform(ctx),
            'delta_x': to_float(ctx, 'delta_x'),
            'delta_y': to_float(ctx, 'delta_y'),
            'delta_yaw': to_float(ctx, 'delta_yaw'),
            'transform_tolerance': to_float(ctx, 'transform_tolerance'),
        }
    ]

    node_name = LaunchConfiguration('node_name').perform(ctx)
    node_options, node_remappings, node_ros_arguments = rlh.resolve_node_launch_configs(
        [node_name],
        LaunchConfiguration('node_options').perform(ctx),
        LaunchConfiguration('node_logging_options').perform(ctx),
        LaunchConfiguration('node_remappings').perform(ctx),
    )

    return [
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            remappings=node_remappings[node_name],
            ros_arguments=node_ros_arguments[node_name],
            output=node_options[node_name]['output'],
            emulate_tty=node_options[node_name]['emulate_tty'],
            respawn=node_options[node_name]['respawn'],
            respawn_delay=node_options[node_name]['respawn_delay'],
        )
    ]
