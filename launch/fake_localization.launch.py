import os
from typing import Any, Dict

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('namespace', default_value='', description='namespace (Optional, default: "")'),
            # 'robot_name' is used to set 'robot_namespace' and 'robot_prefix'.
            DeclareLaunchArgument('robot_name', default_value='robot', description='The unique name for the robot'),
            # Parameters can be passed through the parameter file or through the launch file.
            # The parameters set in the launch file have precedence over those set in the parameter file.
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('fake_localization'), 'config', 'example_fake_localization.yaml'
                ),
                description='Base YAML with ros__parameters',
            ),
            DeclareLaunchArgument(
                'global_frame', default_value='', description='Global frame name for the project (Optional)'
            ),
            DeclareLaunchArgument(
                'odometry_frame',
                default_value='',
                description='Odometry frame name for the robot. Do not prepend robot prefix here (Optional)',
            ),
            DeclareLaunchArgument(
                'base_frame',
                default_value='',
                description='Base frame name for the robot. Do not prepend robot prefix here (Optional)',
            ),
            DeclareLaunchArgument(
                'delta_x',
                default_value='',
                description='Position of the global_frame in x .w.r.t the simulator frame (Optional)',
            ),
            DeclareLaunchArgument(
                'delta_y',
                default_value='',
                description='Position of the global_frame in y .w.r.t the simulator frame (Optional)',
            ),
            DeclareLaunchArgument(
                'delta_yaw',
                default_value='',
                description='Orientation of the global_frame yaw .w.r.t the simulator frame (Optional)',
            ),
            DeclareLaunchArgument(
                'transform_tolerance',
                default_value='',
                description='Tolerance for considering transforms as up-to-date',
            ),
            DeclareLaunchArgument('topic_remappings', default_value='', description=rlh.TOPIC_REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'logging_options', default_value=rlh.default_logging_options_str(), description=rlh.LOGGING_OPTIONS_DESC
            ),
            OpaqueFunction(function=launch_fake_localization_node),
        ]
    )


def launch_fake_localization_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters = []

    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        parameters.append(ParameterFile(params_file, allow_substs=True))

    # If parameters 'global_frame', 'odometry_frame', 'base_frame', 'delta_x', 'delta_y', 'delta_yaw',
    # 'transform_tolerance' are set through the launch file, they override those set in the parameter file.
    parameters_dict: Dict[str, Any] = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }

    robot_prefix = LaunchConfiguration('robot_prefix').perform(ctx)  # No need to do strip(), it's been done already.
    global_frame = LaunchConfiguration('global_frame').perform(ctx).strip()

    if global_frame:
        # Global frame is not prepended with robot prefix.
        parameters_dict['global_frame'] = global_frame

    odometry_frame = LaunchConfiguration('odometry_frame').perform(ctx).strip()

    if odometry_frame:
        parameters_dict['odometry_frame'] = robot_prefix + odometry_frame

    base_frame = LaunchConfiguration('base_frame').perform(ctx).strip()

    if base_frame:
        parameters_dict['base_frame'] = robot_prefix + base_frame

    delta_x = LaunchConfiguration('delta_x').perform(ctx).strip()

    if delta_x:
        parameters_dict['delta_x'] = float(delta_x)

    delta_y = LaunchConfiguration('delta_y').perform(ctx).strip()

    if delta_y:
        parameters_dict['delta_y'] = float(delta_y)

    delta_yaw = LaunchConfiguration('delta_yaw').perform(ctx).strip()

    if delta_yaw:
        parameters_dict['delta_yaw'] = float(delta_yaw)

    transform_tolerance = LaunchConfiguration('transform_tolerance').perform(ctx).strip()

    if transform_tolerance:
        parameters_dict['transform_tolerance'] = float(transform_tolerance)

    parameters.append(parameters_dict)

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'fake_localization'

    robot_ns = rlh.create_robot_namespace(
        LaunchConfiguration('namespace').perform(ctx).strip(), LaunchConfiguration('robot_name').perform(ctx)
    )

    return [
        # Set the LaunchConfiguration 'robot_prefix' before launching the node, since the ParameterFile action
        # needs the 'robot_prefix' to be set to substitute its value in the content of the parameter file.
        SetLaunchConfiguration('robot_prefix', robot_prefix),
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            name=node_name,
            # Insert the node into the robot_namespace.
            namespace=robot_ns,
            parameters=parameters,
            remappings=rlh.process_topic_remappings(LaunchConfiguration('topic_remappings').perform(ctx)),
            ros_arguments=rlh.process_logging_options(LaunchConfiguration('logging_options').perform(ctx).strip()),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        ),
    ]
