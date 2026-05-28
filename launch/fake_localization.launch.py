from pathlib import Path

import ros2_launch_helpers as rlh
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity


def generate_launch_description() -> LaunchDescription:
    """
    Build the fake localization launch description from a parameter file.

    The caller must pass `params_file`, `params_file_allow_substs`, and
    `use_sim_time` explicitly. This launch file loads the YAML file first and
    then sets use_sim_time from the launch argument.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='namespace'),
            DeclareLaunchArgument('params_file', description='YAML file with all node parameters'),
            DeclareLaunchArgument(
                'params_file_allow_substs',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in params_file',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
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
    params_file = rlh.resolve_file(LaunchConfiguration('params_file').perform(ctx))

    if not params_file:
        raise RuntimeError('params_file must point to the fake localization parameters YAML file.')

    if not Path(params_file).is_file():
        raise FileNotFoundError(f"Params file '{params_file}' does not exist.")

    params_file_allow_substs_lc = LaunchConfiguration('params_file_allow_substs')
    params_file_allow_substs = perform_typed_substitution(
        ctx, normalize_typed_substitution(params_file_allow_substs_lc, bool), bool
    )
    use_sim_time_lc = LaunchConfiguration('use_sim_time')
    use_sim_time = perform_typed_substitution(ctx, normalize_typed_substitution(use_sim_time_lc, bool), bool)

    node_name = LaunchConfiguration('node_name').perform(ctx)

    if not rlh.is_valid_name(node_name):
        raise RuntimeError(f"The name of the node must be ASCII [A-Za-z0-9_] only: '{node_name}'")

    node_options, node_remappings, node_ros_arguments = rlh.resolve_node_launch_configs(
        [node_name],
        LaunchConfiguration('node_options').perform(ctx),
        LaunchConfiguration('node_logging_options').perform(ctx),
        LaunchConfiguration('node_remappings').perform(ctx),
    )

    # When params_file_allow_substs is true, the caller must provide every launch
    # context key used by the parameter file. If it is false, the file is loaded
    # without expanding launch substitutions.
    return [
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=[
                ParameterFile(params_file, allow_substs=params_file_allow_substs),
                {'use_sim_time': use_sim_time},
            ],
            remappings=node_remappings[node_name],
            ros_arguments=node_ros_arguments[node_name],
            output=node_options[node_name]['output'],
            emulate_tty=node_options[node_name]['emulate_tty'],
            respawn=node_options[node_name]['respawn'],
            respawn_delay=node_options[node_name]['respawn_delay'],
        )
    ]
