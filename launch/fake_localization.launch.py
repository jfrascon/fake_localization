from pathlib import Path
from typing import Any, List

import ros2_launch_helpers as rlh
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='namespace'),
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
                'base_frame', default_value='robot_base_link', description='Base frame name for the robot'
            ),
            DeclareLaunchArgument('delta_x', default_value='0.0', description='Offset in x'),
            DeclareLaunchArgument('delta_y', default_value='0.0', description='Offset in y'),
            DeclareLaunchArgument('delta_yaw', default_value='0.0', description='Offset in yaw'),
            DeclareLaunchArgument(
                'transform_tolerance', default_value='0.1', description='Tolerance to consider transforms as up-to-date'
            ),
            DeclareLaunchArgument('node_remappings', default_value='', description=rlh.REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_logging_options',
                default_value=rlh.default_logging_options_str(),
                description=rlh.LOGGING_OPTIONS_DESC,
            ),
            OpaqueFunction(function=launch_fake_localization_node),
        ]
    )


def launch_fake_localization_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # The launch file has two exclusive configuration modes for the node-specific parameters.
    # If `params_file` is not empty, those parameters come from that file.
    # If `params_file` is empty, those parameters come from the launch arguments below.
    # `use_sim_time` is the only exception: it is always taken from its launch argument.
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))
    else:
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
                'global_frame': LaunchConfiguration('global_frame').perform(ctx),
                'odometry_frame': LaunchConfiguration('odometry_frame').perform(ctx),
                'base_frame': LaunchConfiguration('base_frame').perform(ctx),
                'delta_x': delta_x,
                'delta_y': delta_y,
                'delta_yaw': delta_yaw,
                'transform_tolerance': transform_tolerance,
            }
        )

    # The `use_sim_time` parameter is always taken from the launch argument, even if a params_file
    # is provided.
    # This allows to easily switch between real and simulated time without modifying the params
    # file.
    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'fake_localization'

    return [
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            remappings=rlh.process_remappings(LaunchConfiguration('node_remappings').perform(ctx)),
            ros_arguments=rlh.process_node_logging_options(LaunchConfiguration('node_logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
