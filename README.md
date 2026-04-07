# `fake_localization`

`fake_localization` provides one localization node that converts simulator ground-truth odometry
into `amcl`-style outputs. It subscribes to ground-truth odometry, publishes the robot pose and a
single-particle cloud in the global frame, and broadcasts the transform from the global frame to
the odometry frame.

The node is also available as the composable node `fake_localization::FakeLocalization`.

This ROS 2 version was migrated by [Juan Francisco Rascon Crespo](mailto:jfrascon@gmail.com) from
the original ROS 1 `fake_localization` package by Ioan A. Sucan.

## What this package launches

`fake_localization.launch.py` launches one `fake_localization_node`.

The `namespace` launch argument places that node under a ROS namespace. Use a different namespace
per robot instance in multirobot setups so node names and topics do not collide.

The node subscribes to:
- `base_pose_ground_truth` (`nav_msgs/msg/Odometry`)
- `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)

The node publishes:
- `amcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `particlecloud` (`geometry_msgs/msg/PoseArray`)

The node broadcasts this TF transform:
- `<global_frame> -> <odometry_frame>`

## Configuration model

The launch file always loads a parameter YAML file through `params_file`.

If you do not pass `params_file`, the default file installed by this package is used:
`config/example_fake_localization.yaml`.

If a YAML value is written as `$(var <launch_argument_name>)`, that value is resolved from the
current launch context. This allows a YAML file to delegate selected values to launch arguments.
If a YAML value is written as a literal, that literal value is used as-is.

An empty `params_file` is not a supported value. `ParameterFile` expects a real file path, so a
wrapper launch should pass a valid YAML path or leave `params_file` unset and let this launch file
use its default.

The launch file also accepts:
- `use_sim_time`
- `global_frame`
- `odometry_frame`
- `base_frame`
- `delta_x`
- `delta_y`
- `delta_yaw`
- `transform_tolerance`
- `node_remappings`
- `node_options`
- `node_logging_options`

`node_options` controls the ROS node process options handled by `ros2_launch_helpers`, including the
node name, output mode, and respawn settings.

`node_logging_options` forwards ROS logging CLI arguments. For example, use
`node_logging_options:="--ros-args --log-level debug"` to run the node at debug log level.

## Parameters

The node reads these ROS parameters:
- `use_sim_time` (`bool`)
- `global_frame` (`string`, default: `map`)
- `odometry_frame` (`string`, default: `odom`)
- `base_frame` (`string`, default: `base_link`)
- `delta_x` (`double`, default: `0.0`)
- `delta_y` (`double`, default: `0.0`)
- `delta_yaw` (`double`, default: `0.0`)
- `transform_tolerance` (`double`, default: `0.1`)

`delta_x`, `delta_y`, and `delta_yaw` define the pose of `global_frame` with respect to the
simulator world frame. Keep them at zero when both frames coincide.

`transform_tolerance` shifts the TF timestamp slightly into the future so downstream consumers can
accept the transform when message and TF timestamps are not perfectly aligned.

## Examples

Launch with the package example YAML and override selected `$(var ...)` values from the CLI:

```bash
ros2 launch fake_localization fake_localization.launch.py \
  namespace:=robot_1 \
  global_frame:=map \
  odometry_frame:=odom \
  base_frame:=base_link \
  delta_x:=0.0 \
  delta_y:=0.0 \
  delta_yaw:=0.0 \
  transform_tolerance:=0.1 \
  node_remappings:="base_pose_ground_truth:=odom" \
  node_logging_options:="--ros-args --log-level debug"
```

Use a custom `params_file` where all values are delegated to launch arguments:

```yaml
/**/fake_localization:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    global_frame: $(var global_frame)
    odometry_frame: $(var odometry_frame)
    base_frame: $(var base_frame)
    delta_x: $(var delta_x)
    delta_y: $(var delta_y)
    delta_yaw: $(var delta_yaw)
    transform_tolerance: $(var transform_tolerance)
```

In this case, the YAML structure stays fixed and the launch arguments provide the actual values.

Use a custom `params_file` where some values are literal and some still come from launch arguments:

```yaml
/**/fake_localization:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    global_frame: map
    odometry_frame: robot_1/odom
    base_frame: robot_1/base_link
    delta_x: 0.0
    delta_y: 0.0
    delta_yaw: $(var delta_yaw)
    transform_tolerance: 0.2
```

In this case, the literal YAML values are used directly, and only `delta_yaw` and `use_sim_time`
are resolved from the launch context.

Use a custom `params_file` where every node parameter is literal except `use_sim_time`:

```yaml
/**/fake_localization:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    global_frame: map
    odometry_frame: robot_1/odom
    base_frame: robot_1/base_link
    delta_x: 1.5
    delta_y: -0.2
    delta_yaw: 0.1
    transform_tolerance: 0.2
```

In this case, the YAML file fully defines the node configuration except for the simulation-clock
switch.
