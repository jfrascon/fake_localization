import ros2_launch_helpers as rlh
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue


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
                'node_arguments', default_value=('{"output":"both"}'), description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC
            ),
            rlh.RequireFile(path=LaunchConfiguration('params_file')),
            rlh.RenderParamsFile(
                params_file=LaunchConfiguration('params_file'),
                output_context_key='params_file',
                condition=IfCondition(LaunchConfiguration('params_file_allow_substs')),
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # `namespace` argument must be passed explicitly to the Node, so it is not allowed to be
    # configured via `node_arguments` (rejected here).
    # Rejecting it here prevents the same Node field from being configured from two different
    # places.

    return [
        Node(
            package='fake_localization',
            executable='fake_localization_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                ParameterFile(LaunchConfiguration('params_file'), allow_substs=False),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_arguments').perform(ctx), extra_rejected_arguments={'namespace'}
            ),
        )
    ]
