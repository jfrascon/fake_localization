from pathlib import Path
from typing import Any, List

import ros2_launch_helpers as rlh
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='namespace'),
            # If params_file is not empty, the node is configured with the
            # parameters from that file. Otherwise, the node is configured with
            # the parameters declared after params_file.
            DeclareLaunchArgument('params_file', default_value='', description='YAML file with node parameters'),
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
                'base_frame', default_value='robot_base_footprint_link', description='Base frame name for the robot'
            ),
            DeclareLaunchArgument('delta_x', default_value='0.0', description='Offset in x'),
            DeclareLaunchArgument('delta_y', default_value='0.0', description='Offset in y'),
            DeclareLaunchArgument('delta_yaw', default_value='0.0', description='Offset in yaw'),
            DeclareLaunchArgument(
                'transform_tolerance', default_value='0.1', description='Tolerance to consider transforms as up-to-date'
            ),
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
            OpaqueFunction(function=launch_fake_localization_node),
        ]
    )


def launch_fake_localization_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # The launch file has two exclusive configuration modes for node parameters.
    # If `params_file` is not empty, every node parameter comes from that file.
    # If `params_file` is empty, every node parameter comes from the launch
    # arguments below.
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))
    else:
        use_sim_time_lc = LaunchConfiguration('use_sim_time')
        use_sim_time = perform_typed_substitution(ctx, normalize_typed_substitution(use_sim_time_lc, bool), bool)

        try:
            delta_x = float(LaunchConfiguration('delta_x').perform(ctx))
        except ValueError as exc:
            raise ValueError('Invalid value for delta_x. Must be a float.') from exc

        try:
            delta_y = float(LaunchConfiguration('delta_y').perform(ctx))
        except ValueError as exc:
            raise ValueError('Invalid value for delta_y. Must be a float.') from exc

        try:
            delta_yaw = float(LaunchConfiguration('delta_yaw').perform(ctx))
        except ValueError as exc:
            raise ValueError('Invalid value for delta_yaw. Must be a float.') from exc

        try:
            transform_tolerance = float(LaunchConfiguration('transform_tolerance').perform(ctx))
        except ValueError as exc:
            raise ValueError('Invalid value for transform_tolerance. Must be a float.') from exc

        parameters.append(
            {
                'use_sim_time': use_sim_time,
                'global_frame': LaunchConfiguration('global_frame').perform(ctx),
                'odometry_frame': LaunchConfiguration('odometry_frame').perform(ctx),
                'base_frame': LaunchConfiguration('base_frame').perform(ctx),
                'delta_x': delta_x,
                'delta_y': delta_y,
                'delta_yaw': delta_yaw,
                'transform_tolerance': transform_tolerance,
            }
        )

    node_name = 'fake_localization'
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
