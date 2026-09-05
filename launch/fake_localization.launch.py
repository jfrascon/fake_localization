from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """
    Build the fake localization launch description from a parameter file.

    The caller must pass `params_file`, `params_file_allow_substs`, and `use_sim_time` explicitly.
    This launch file loads the YAML file first and then overrides `use_sim_time` from the launch
    argument.
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
                description='Use ROS time from /clock if true.',
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('params_file')),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # `namespace` argument must be passed explicitly to the Node, so it is not allowed to be
    # configured via `node_args` (rejected here).
    # Rejecting it here prevents the same Node field from being configured from two different
    # places.

    # 'params_file' exists and is a file. This is guaranteed by the `rlh.RequireFile` action.
    params_allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool),
        bool,
    )

    return [
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                ParameterFile(
                    LaunchConfiguration('params_file'), allow_substs=params_allow_substs
                ),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    )
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'fake_localization'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]
