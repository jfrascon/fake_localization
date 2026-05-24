# [`fake_localization`](https://github.com/jfrascon/fake_localization/tree/ros2)

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

The launch file has two exclusive configuration modes.

If `params_file` is empty, the node parameters are built from the launch arguments:
- `global_frame`
- `odometry_frame`
- `base_frame`
- `delta_x`
- `delta_y`
- `delta_yaw`
- `transform_tolerance`

If `params_file` is not empty, the node parameters are loaded from that YAML file instead. In this
mode, the launch arguments listed above are ignored.

`use_sim_time` is the only exception. It is always taken from the `use_sim_time` launch argument.
If a YAML file also contains `use_sim_time`, the launch argument still wins.

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

Launch without `params_file`. In this mode the node parameters come from the launch arguments:

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

Use a custom `params_file`. In this mode the YAML file defines the node parameters, except for
`use_sim_time`, which still comes from the launch argument:

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

In this example, the value `use_sim_time: false` in the YAML file is not the effective value if the
launch command passes `use_sim_time:=True`. The launch argument is applied after the YAML file.
