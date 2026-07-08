from pathlib import Path

import ros2_launch_helpers as rlh
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


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
                'use_sim_time', choices=['True', 'true', 'False', 'false'], description='Use simulation clock if true'
            ),
            DeclareLaunchArgument(
                'node_arguments',
                default_value=(
                    '{"output":"both", "respawn":false, "respawn_delay":0.0, "ros_arguments":["--log-level","info"]}'
                ),
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # The client must always provide a parameter file.
    #
    params_file = LaunchConfiguration('params_file').perform(ctx)

    if not params_file:
        raise RuntimeError('params_file must point to the fake localization parameters YAML file.')

    if not Path(params_file).is_file():
        raise FileNotFoundError(f"Params file '{params_file}' does not exist.")

    # When params_file_allow_substs is true, the caller must provide every launch
    # context key used by the parameter file. If it is false, the file is loaded
    # without expanding launch substitutions.
    params_file_allow_substs = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool), bool
    )

    # `namespace` argument must be passed explicitly to the Node, so it is not allowed to be
    # configured via `node_arguments` (rejected here).
    # Rejecting it here prevents the same Node field from being configured from two different
    # places.
    node_arguments = rlh.resolve_node_arguments(
        LaunchConfiguration('node_arguments').perform(ctx), extra_rejected_arguments={'namespace'}
    )

    return [
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                ParameterFile(params_file, allow_substs=params_file_allow_substs),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            **node_arguments,
        )
    ]
