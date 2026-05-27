# [`fake_localization`](https://github.com/jfrascon/fake_localization/tree/ros2)

`fake_localization` provides one localization node that converts simulator ground-truth odometry
into `amcl`-style outputs. It subscribes to ground-truth odometry, publishes the robot pose and a
single-particle cloud in the global frame, and broadcasts the transform from the global frame to
the odometry frame.

The node is also available as the composable node `fake_localization::FakeLocalization`.

This ROS 2 version was migrated by [Juan Francisco Rascon Crespo](mailto:jfrascon@gmail.com) from
the original ROS 1 `fake_localization` package by Ioan A. Sucan.

## What this package launches

`fake_localization.launch.py` launches one `fake_localization_node` from a YAML parameter file.
`fake_localization_args.launch.py` launches the same node from launch arguments.

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

The package provides two launch files with separate configuration contracts.

`fake_localization.launch.py` requires `params_file`. In this mode every node parameter comes from
that YAML file. The launch file does not declare per-parameter launch arguments.

`fake_localization_args.launch.py` does not use a YAML file. In this mode the node parameters are
built from these launch arguments:
- `use_sim_time`
- `global_frame`
- `odometry_frame`
- `base_frame`
- `delta_x`
- `delta_y`
- `delta_yaw`
- `transform_tolerance`

Both launch files also accept:
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

Launch with a YAML parameter file:

```bash
ros2 launch fake_localization fake_localization.launch.py \
  namespace:=robot_1 \
  params_file:=/path/to/fake_localization.yaml \
  node_remappings:="base_pose_ground_truth:=odom" \
  node_logging_options:="--ros-args --log-level debug"
```

Launch without a YAML parameter file. In this mode the node parameters come from launch arguments:

```bash
ros2 launch fake_localization fake_localization_args.launch.py \
  namespace:=robot_1 \
  use_sim_time:=false \
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

Example YAML file:

```yaml
/**/fake_localization:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    odometry_frame: robot_1_odom
    base_frame: robot_1_base_link
    delta_x: 0.0
    delta_y: 0.0
    delta_yaw: 0.0
    transform_tolerance: 0.1
```
