import os
from pathlib import Path  # There is no need to set the keys `namespace`, `params_file`, `use_sim_time`,
from typing import Any, List

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='namespace'),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('fake_localization'), 'config', 'example_fake_localization.yaml'
                ),
                description='YAML file with node parameters',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('global_frame', default_value='', description='Global frame'),
            DeclareLaunchArgument('odometry_frame', default_value='', description='Odometry frame for the robot'),
            DeclareLaunchArgument('base_frame', default_value='', description='Base frame name for the robot'),
            DeclareLaunchArgument('delta_x', default_value='', description='Offset in x'),
            DeclareLaunchArgument('delta_y', default_value='', description='Offset in y'),
            DeclareLaunchArgument('delta_yaw', default_value='', description='Offset in yaw'),
            DeclareLaunchArgument(
                'transform_tolerance', default_value='', description='Tolerance to consider transforms as up-to-date'
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
    # If the params_file exists, load it as a ParameterFile.
    # If any parameter is also provided to this launch file, it takes precedence over the
    # params_file.
    # This allows to override specific parameters in the params_file without having to create a new
    # params file.
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx)
    global_frame = LaunchConfiguration('global_frame').perform(ctx)
    odometry_frame = LaunchConfiguration('odometry_frame').perform(ctx)
    base_frame = LaunchConfiguration('base_frame').perform(ctx)
    delta_x = LaunchConfiguration('delta_x').perform(ctx)
    delta_y = LaunchConfiguration('delta_y').perform(ctx)
    delta_yaw = LaunchConfiguration('delta_yaw').perform(ctx)
    transform_tolerance = LaunchConfiguration('transform_tolerance').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))

    if global_frame:
        parameters.append({'global_frame': global_frame})

    if odometry_frame:
        parameters.append({'odometry_frame': odometry_frame})

    if base_frame:
        parameters.append({'base_frame': base_frame})

    if delta_x:
        try:
            parameters.append({'delta_x': float(delta_x)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for delta_x: '{delta_x}'. Must be a float.") from exc

    if delta_y:
        try:
            parameters.append({'delta_y': float(delta_y)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for delta_y: '{delta_y}'. Must be a float.") from exc

    if delta_yaw:
        try:
            parameters.append({'delta_yaw': float(delta_yaw)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for delta_yaw: '{delta_yaw}'. Must be a float.") from exc

    if transform_tolerance:
        try:
            parameters.append({'transform_tolerance': float(transform_tolerance)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for transform_tolerance: '{transform_tolerance}'. Must be a float."
            ) from exc

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
