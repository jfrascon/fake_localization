from pathlib import Path

import ros2_launch_helpers as rlh
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='namespace'),
            DeclareLaunchArgument('params_file', description='YAML file with all node parameters'),
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
    params_file = LaunchConfiguration('params_file').perform(ctx)

    if not Path(params_file).is_file():
        raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

    node_name = LaunchConfiguration('node_name').perform(ctx)

    if not rlh.is_valid_name(node_name):
        raise RuntimeError(f"The name of the node must be ASCII [A-Za-z0-9_] only: '{node_name}'")

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
            parameters=[ParameterFile(params_file, allow_substs=True)],
            remappings=node_remappings[node_name],
            ros_arguments=node_ros_arguments[node_name],
            output=node_options[node_name]['output'],
            emulate_tty=node_options[node_name]['emulate_tty'],
            respawn=node_options[node_name]['respawn'],
            respawn_delay=node_options[node_name]['respawn_delay'],
        )
    ]
