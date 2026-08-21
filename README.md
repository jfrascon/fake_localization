# [`fake_localization`](https://github.com/jfrascon/fake_localization/tree/ros2)

## Description

`fake_localization` provides one localization node that converts a simulator pose into `amcl`-style outputs. It subscribes to a `sim_pose` message encoded as `nav_msgs/msg/Odometry`, publishes the robot pose and a single-particle cloud in the configured global reference frame, and broadcasts the transform from the global reference frame to the robot odometry frame.

The node is also available as the composable node `fake_localization::FakeLocalization`.

This ROS 2 version was migrated by [Juan Francisco Rascon Crespo](mailto:jfrascon@gmail.com) from the original ROS 1 `fake_localization` package by Ioan A. Sucan.

## Workspace setup

Clone this repository inside the `src` directory of a ROS 2 Jazzy workspace.
Import its reusable source dependencies before resolving system dependencies:

```bash
cd <workspace>
vcs import src < src/fake_localization/deps.repos
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

The `deps.repos` file pins `ros2_launch_helpers` to a tested commit. A parent
project may provide the same package through its own repository manifest.

## Build

Source ROS 2 and build the package together with its workspace dependencies:

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --packages-up-to fake_localization --symlink-install
source install/setup.bash
```

## Tests and code quality

Run the package tests from the workspace root:

```bash
colcon test --packages-select fake_localization
colcon test-result --test-result-base build/fake_localization --verbose
```

The system hooks in `pre-commit` use ROS 2 lint executables. Source ROS 2
before running them:

```bash
source /opt/ros/jazzy/setup.bash
cd src/fake_localization
pre-commit run --all-files
```

## Nodes

### `fake_localization_node`

Runs the `fake_localization::FakeLocalization` node as a standalone process.

#### Subscribed Topics

- `sim_pose` (`nav_msgs/msg/Odometry`): Simulator pose of `robot_frame` expressed in the simulated world's reference frame.
- `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Initial pose estimate, usually sent by RViz. The callback updates the internal transform between the configured global reference frame and the simulated world's reference frame.

#### Published Topics

- `amcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Robot pose expressed in `global_frame`, the configured global reference frame.
- `particlecloud` (`geometry_msgs/msg/PoseArray`): Single-particle cloud containing the same pose published on `amcl_pose`.

#### Services

This package does not define package-specific services.

#### Parameters

- `use_sim_time` (`bool`): Standard ROS 2 parameter that selects simulation time when true. When using `fake_localization.launch.py`, this parameter is set by the launch argument `use_sim_time`, not by the YAML file alone.
- `global_frame` (`string`, default: `map`): Name of the global reference frame in which `amcl_pose` and `particlecloud` are published. This is also the parent frame of the TF transform broadcast by the node.
- `robot_odometry_frame` (`string`, default: `odom`): Name of the robot odometry frame. This is the child frame of the TF transform broadcast by the node.
- `robot_frame` (`string`, default: `base_link`): Name of the robot-attached frame used to combine the incoming simulator pose with the available odometry transform.
- `delta_x` (`double`, default: `0.0`): X position of `global_frame` expressed in the simulated world's reference frame.
- `delta_y` (`double`, default: `0.0`): Y position of `global_frame` expressed in the simulated world's reference frame.
- `delta_yaw` (`double`, default: `0.0`): Yaw orientation of `global_frame` expressed in the simulated world's reference frame, in radians.
- `transform_tolerance` (`double`, default: `0.1`): Time tolerance added to the TF timestamp so downstream consumers can accept the transform when message and TF timestamps are not perfectly aligned.

#### Transform model

This section uses the notation ${}^{a}T_{b}$ to mean the position and orientation of frame $b$ expressed in frame $a$. Equivalently, ${}^{a}T_{b}$ converts coordinates expressed in frame $b$ into coordinates expressed in frame $a$.

The `sim_pose` message is encoded as `nav_msgs/msg/Odometry`, but the message is used here as a pose container. Therefore, the message does not necessarily represent odometry in the semantic sense. Its pose field must represent the pose of `robot_frame` expressed in the simulated world's reference frame:

$$
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
$$

In practice, this means that `sim_pose.header.frame_id` must contain the name of the simulated world's reference frame, i.e., the frame in which `sim_pose.pose.pose` is expressed. The field `sim_pose.child_frame_id` must match the configured `robot_frame`, and `sim_pose.pose.pose` represents the transformation ${}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}$.

The parameters `delta_x`, `delta_y`, and `delta_yaw` define the position and orientation (pose) of `global_frame`, the global reference frame, with respect to the simulated world's reference frame:

$$
\begin{aligned}
{}^{\text{sim-world-reference-frame}}T_{\text{global-frame}}
&=
\begin{bmatrix}
\cos(\delta_{\mathrm{yaw}}) & -\sin(\delta_{\mathrm{yaw}}) & 0 & \delta_x\\
\sin(\delta_{\mathrm{yaw}}) & \cos(\delta_{\mathrm{yaw}}) & 0 & \delta_y\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

Internally, `fake_localization` needs the inverse transform to convert simulator poses from the simulated world's reference frame into `global_frame`, the global reference frame:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
&=
\left({}^{\text{sim-world-reference-frame}}T_{\text{global-frame}}\right)^{-1}\\
&=
\begin{bmatrix}
\cos(\delta_{\mathrm{yaw}}) & \sin(\delta_{\mathrm{yaw}}) & 0 & -\delta_x\cos(\delta_{\mathrm{yaw}}) - \delta_y\sin(\delta_{\mathrm{yaw}})\\
-\sin(\delta_{\mathrm{yaw}}) & \cos(\delta_{\mathrm{yaw}}) & 0 & \delta_x\sin(\delta_{\mathrm{yaw}}) - \delta_y\cos(\delta_{\mathrm{yaw}})\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

When `delta_x`, `delta_y`, and `delta_yaw` are all zero, `global_frame` coincides with the simulated world's reference frame, so the simulator pose is already expressed in `global_frame`.

With the incoming simulator pose and the inverse transform computed from the deltas, the node computes the pose of `robot_frame` expressed in `global_frame`:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
\cdot
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
\end{aligned}
$$

That transform is the pose published on `amcl_pose`. The `particlecloud` topic contains a single particle with the same pose.

A localization node usually works with the relationship between the robot pose in the global reference frame, the transform from the global reference frame to the robot odometry frame, and the robot pose in the robot odometry frame:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-frame}}
&=
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
\cdot
{}^{\text{robot-odometry-frame}}T_{\text{robot-frame}}
\end{aligned}
$$

The objective of `fake_localization` is to compute and broadcast:

$$
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
$$

Solving the previous expression for that transform gives:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
&=
{}^{\text{global-frame}}T_{\text{robot-frame}}
\cdot
\left({}^{\text{robot-odometry-frame}}T_{\text{robot-frame}}\right)^{-1}\\
&=
{}^{\text{global-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}
\end{aligned}
$$

and, substituting the expression previously computed for the pose of `robot_frame` in `global_frame`:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
\cdot
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}
\end{aligned}
$$

Therefore, the node needs the incoming `sim_pose` message and the TF tree must provide the relationship between `robot_odometry_frame` and `robot_frame` at the timestamp of that message.

#### Caveat when `sim_pose` is odometry

`fake_localization` does not check whether `sim_pose` is a global/reference pose or odometry in the semantic sense. It always computes:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
\cdot
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}
\end{aligned}
$$

If `sim_pose` is actually odometry, then the frame written in `sim_pose.header.frame_id` is not acting as a simulated world's reference frame. It is acting as an odometry-like frame of the simulator, called `sim_odom_frame` here for explanation purposes. In that case, `sim_pose.pose.pose` represents ${}^{\text{sim-odom-frame}}T_{\text{robot-frame}}$, i.e., the position and orientation of `robot_frame` expressed in `sim_odom_frame`. If another odometry source also provides ${}^{\text{robot-odometry-frame}}T_{\text{robot-frame}}$, `fake_localization` just multiplies both transforms, as shown before.

If both odometry estimates are exactly equal:

$$
\begin{aligned}
{}^{\text{sim-odom-frame}}T_{\text{robot-frame}}
&=
{}^{\text{robot-odometry-frame}}T_{\text{robot-frame}}
\end{aligned}
$$

then:

$$
\begin{aligned}
I_{4 \times 4}
&=
{}^{\text{sim-odom-frame}}T_{\text{robot-frame}}
\cdot
\left({}^{\text{robot-odometry-frame}}T_{\text{robot-frame}}\right)^{-1}\\
&=
{}^{\text{sim-odom-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}
\end{aligned}
$$

In that case, the transform published by `fake_localization`, ${}^{\text{global-frame}}T_{\text{robot-odometry-frame}}$, is the constant transform ${}^{\text{global-frame}}T_{\text{sim-odom-frame}}$, initialized from the deltas provided by the user as parameters, as shown in the following derivation:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
\cdot
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}\\
&=
{}^{\text{global-frame}}T_{\text{sim-odom-frame}}
\cdot
{}^{\text{sim-odom-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}\\
&=
{}^{\text{global-frame}}T_{\text{sim-odom-frame}}
\cdot
I_{4 \times 4}\\
&=
{}^{\text{global-frame}}T_{\text{sim-odom-frame}}
\end{aligned}
$$

If the two odometry estimates differ, even slightly, the product is not the identity transform $I_{4 \times 4}$:

$$
\begin{aligned}
I_{4 \times 4}
&\neq
{}^{\text{sim-odom-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}
\end{aligned}
$$

Therefore, the transform published by `fake_localization`, ${}^{\text{global-frame}}T_{\text{robot-odometry-frame}}$, keeps the full expression:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-odometry-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-odom-frame}}
\cdot
{}^{\text{sim-odom-frame}}T_{\text{robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{robot-odometry-frame}}
\end{aligned}
$$

**This may be useful only if it is intentionally what the user of this package wants to achieve. It is not the usual purpose of this package.**

If the simulator already provides the odometry transform ${}^{\text{sim-odom-frame}}T_{\text{robot-frame}}$, then `fake_localization` is usually not needed to obtain ${}^{\text{global-frame}}T_{\text{sim-odom-frame}}$. In that setup, publish ${}^{\text{global-frame}}T_{\text{sim-odom-frame}}$ directly, for example with the utility `static_transform_publisher` from the `tf2_ros` package.

For example, to publish an identity transform from `global_frame` to `sim_odom_frame`:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id map \
  --child-frame-id robot_odom
```

where `map` is the `global_frame` and `robot_odom` is the `sim_odom_frame`. The transform is static, so it does not need to be published at a high frequency.

This way, you also obtain the transform tree:

```text
global_frame -> sim_odom_frame -> robot_frame
```

#### Updating the global transform from RViz

The `initialpose` topic can be used from RViz to update the transform:

$$
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
$$

This is useful when the user wants to change the relation between the two frames in a graphical way, without manually computing the transform.

In RViz, the user selects the `2D Pose Estimate` tool, clicks in the main view, and drags to choose an orientation. For this interpretation to be valid, the fixed frame in RViz must be set to `global_frame`, so that the pose selected in RViz, interpreted as the desired pose of `robot_frame`, is expressed in `global_frame`.

The pose selected in RViz can be interpreted as a transform:

$$
{}^{\text{global-frame}}T_{\text{desired-robot-frame}}
$$

In this scenario, `initialpose.header.frame_id` is `global_frame`. This action does not move the robot in Gazebo. It also does not change the pose received from the simulator. The incoming `sim_pose` message still represents the same transformation:

$$
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
$$

What changes is the internal transform used by `fake_localization` to express simulator poses in `global_frame`:

$$
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
$$

Before applying the pose selected in RViz, `fake_localization` is already computing the current pose of `robot_frame` in `global_frame`:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
\cdot
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}
\end{aligned}
$$

The selected RViz pose is the desired new value for the pose of `robot_frame` in `global_frame`. Therefore, `fake_localization` looks for a correction transform:

$$
T_{\text{frame-correction}}
$$

such that:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{desired-robot-frame}}
&=
T_{\text{frame-correction}}
\cdot
{}^{\text{global-frame}}T_{\text{robot-frame}}
\end{aligned}
$$

Solving for the correction gives:

$$
\begin{aligned}
T_{\text{frame-correction}}
&=
{}^{\text{global-frame}}T_{\text{desired-robot-frame}}
\cdot
\left({}^{\text{global-frame}}T_{\text{robot-frame}}\right)^{-1}\\
&=
{}^{\text{global-frame}}T_{\text{desired-robot-frame}}
\cdot
{}^{\text{robot-frame}}T_{\text{global-frame}}
\end{aligned}
$$

Then the node applies that correction to its internal transform:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame},\mathrm{new}}
&=
T_{\text{frame-correction}}
\cdot
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame},\mathrm{old}}
\end{aligned}
$$

In practical terms, RViz lets the user say: *I want `robot_frame` to appear here in `global_frame`.* `fake_localization` satisfies that request by changing the internal offset between `global_frame` and the simulated world's reference frame. The simulator pose itself remains unchanged.

## Launch File

`fake_localization.launch.py` launches one `fake_localization_node` from a YAML parameter file.

Launch arguments:

- `namespace` (default: `robot`): ROS namespace for the node. Use a different namespace per robot instance in multirobot setups so node names and topics do not collide.
- `fake_loc_params_file` (required): YAML file with all node parameters except the launch-managed `use_sim_time` override.
- `fake_loc_params_file_allow_substs` (required, `True`/`False`): Whether ROS launch substitutions are allowed in `fake_loc_params_file`.
- `use_sim_time` (required, `True`/`False`): Value passed to the node as the `use_sim_time` parameter.
- `fake_loc_node_args` (default declared in the launch file): JSON object resolved by `ros2_launch_helpers` and passed to the ROS 2 `Node` action. It can set launch action fields such as `output`, `emulate_tty`, `respawn`, `respawn_delay`, `name`, `remappings`, and `ros_arguments`.

Do not put `namespace` inside `fake_loc_node_args`; the launch file keeps `namespace` as an explicit launch argument and rejects duplicate configuration.

## Examples

Launch with a YAML parameter file:

```bash
ros2 launch fake_localization fake_localization.launch.py \
  namespace:=robot_1 \
  fake_loc_params_file:=/path/to/fake_localization.yaml \
  fake_loc_params_file_allow_substs:=False \
  use_sim_time:=True \
  fake_loc_node_args:='{"output":"both","respawn":true,"respawn_delay":2.0,"remappings":[["sim_pose","odom"]],"ros_arguments":["--log-level","debug"]}'
```

Example YAML file:

```yaml
/**/fake_localization:
  ros__parameters:
    # fake_localization.launch.py sets use_sim_time after loading this file.
    # Do not configure use_sim_time in this file when using that launch file.
    global_frame: map
    robot_odometry_frame: robot_1_odom
    robot_frame: robot_1_base_link
    delta_x: 0.0
    delta_y: 0.0
    delta_yaw: 0.0
    transform_tolerance: 0.1
```

Example with a shifted global reference frame and no yaw offset:

```yaml
delta_x: 5.0
delta_y: 2.0
delta_yaw: 0.0
```

This means that `global_frame` is located at `x=5.0`, `y=2.0` in the simulated world's reference frame, with the same orientation as the simulated world's reference frame.

The deltas represent the transformation ${}^{\text{sim-world-reference-frame}}T_{\text{global-frame}}$:

$$
\begin{aligned}
{}^{\text{sim-world-reference-frame}}T_{\text{global-frame}}
&=
\begin{bmatrix}
1 & 0 & 0 & 5.0\\
0 & 1 & 0 & 2.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

The user provides the deltas as ${}^{\text{sim-world-reference-frame}}T_{\text{global-frame}}$, but the node internally computes the inverse transform, ${}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}$, before applying them to simulator poses.

The resulting internal transformation ${}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}$ is:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
&=
\begin{bmatrix}
1 & 0 & 0 & -5.0\\
0 & 1 & 0 & -2.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

The parameters are defined this way because it is usually easier for the user to decide where `global_frame` is located with respect to the simulated world's reference frame than to manually provide the inverse transform.

If the simulator reports the robot at `x=8.0`, `y=6.0`, `yaw=0.0`, the pose published in `global_frame` is `x=3.0`, `y=4.0`, `yaw=0.0`.

The computation is:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{robot-frame}}
&=
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
\cdot
{}^{\text{sim-world-reference-frame}}T_{\text{robot-frame}}\\
&=
\begin{bmatrix}
1 & 0 & 0 & -5.0\\
0 & 1 & 0 & -2.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}
\cdot
\begin{bmatrix}
1 & 0 & 0 & 8.0\\
0 & 1 & 0 & 6.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}\\
&=
\begin{bmatrix}
1 & 0 & 0 & 3.0\\
0 & 1 & 0 & 4.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

Example with a shifted and rotated global reference frame:

```yaml
delta_x: 5.0
delta_y: 2.0
delta_yaw: 1.5708
```

This defines the full rigid transform:

$$
\begin{aligned}
{}^{\text{sim-world-reference-frame}}T_{\text{global-frame}}
&=
\begin{bmatrix}
\cos(1.5708) & -\sin(1.5708) & 0 & 5.0\\
\sin(1.5708) & \cos(1.5708) & 0 & 2.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}\\
&=
\begin{bmatrix}
0 & -1 & 0 & 5.0\\
1 & 0 & 0 & 2.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

The node computes the inverse transform internally to obtain:

$$
\begin{aligned}
{}^{\text{global-frame}}T_{\text{sim-world-reference-frame}}
&=
\begin{bmatrix}
\cos(1.5708) & \sin(1.5708) & 0 & -5.0\cos(1.5708) - 2.0\sin(1.5708)\\
-\sin(1.5708) & \cos(1.5708) & 0 & 5.0\sin(1.5708) - 2.0\cos(1.5708)\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}\\
&=
\begin{bmatrix}
0 & 1 & 0 & -2.0\\
-1 & 0 & 0 & 5.0\\
0 & 0 & 1 & 0.0\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$
