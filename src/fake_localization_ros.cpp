// Copyright 2026 Juan Francisco Rascon Crespo

#include <fake_localization/fake_localization.hpp>
#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_localization
{
  // Notation: T:a->b means homogeneous transformation matrix that converts coordinates expressed in
  // frame b into frame a.

  // Constructor
  FakeLocalization::FakeLocalization(const rclcpp::NodeOptions& options):
    Node("fake_localization", options),
    global_frame_{this->declare_parameter<std::string>("global_frame", std::string{"map"})},
    robot_odometry_frame_{this->declare_parameter<std::string>("robot_odometry_frame", std::string{"odom"})},
    robot_frame_{this->declare_parameter<std::string>("robot_frame", std::string{"base_link"})},
    tolerance_secs_{0},
    tolerance_nanosecs_{0},
    tf_buffer_{this->get_clock()},
    // The third argument makes TransformListener spin a dedicated thread. The listener marks the
    // buffer as using a dedicated thread internally, which allows blocking TF calls with timeouts.
    tf_listener_{tf_buffer_, this, true},
    tf_broadcaster_{*this},
    // A tf2_ros::MessageFilter does not subscribe to a topic by itself. It receives messages from
    // the message_filters::Subscriber object passed to its constructor. For each received message,
    // the filter reads received_msg.header.frame_id and waits until the TF buffer can provide a
    // transform from received_msg.header.frame_id to target_frame at received_msg.header.stamp.
    // Once that transform is available, the filter calls the registered callback.
    //
    // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.
    //
    // In a normal setup, msg_filter_sub_ would subscribe directly to the topic that carries the
    // messages to be filtered. We do not do that here. The sim_pose message must be inspected and
    // edited before it is passed to the TF filter.
    //
    // The incoming sim_pose message is an Odometry message used as a pose container. Before any
    // edit, its fields have this meaning:
    //
    //   sim_pose_msg.header.frame_id = sim_world_reference_frame
    //   sim_pose_msg.child_frame_id  = robot_frame_
    //   sim_pose_msg.pose.pose       = T:sim_world_reference_frame->robot_frame_
    //
    // The child_frame_id check is important. fake_localization uses robot_frame_ as the
    // robot-attached frame for its computations, so the frame named by
    // sim_pose_msg.child_frame_id must match robot_frame_. Otherwise the simulator pose and the
    // odometry TF would refer to different robot frames.
    //
    // If msg_filter_sub_ subscribed directly to sim_pose, the TF filter would see:
    //
    //   target_frame                 = robot_frame_
    //   received_msg.header.frame_id = sim_world_reference_frame
    //
    // and it would wait for:
    //
    //   T:robot_frame_->sim_world_reference_frame
    //
    // That is not the transform needed by update_cb.
    //
    // update_cb needs:
    //
    //   T:robot_frame_->robot_odometry_frame_
    //
    // because it computes T:global_frame_->robot_odometry_frame_:
    //
    //   T:global_frame_->robot_frame_ =
    //     T:global_frame_->robot_odometry_frame_ * T:robot_odometry_frame_->robot_frame_
    //
    //   T:global_frame_->robot_odometry_frame_ =
    //     T:global_frame_->robot_frame_ * (T:robot_odometry_frame_->robot_frame_)^-1 =
    //     T:global_frame_->robot_frame_ * T:robot_frame_->robot_odometry_frame_
    //
    // For that reason, msg_filter_sub_ is constructed with an empty topic name. It exists only
    // because tf2_ros::MessageFilter requires a message_filters::Subscriber object in its
    // constructor. The real ROS subscription is sim_pose_sub_.
    //
    // sim_pose_sub_ receives the sim_pose message first. Its callback, sim_pose_cb, validates
    // sim_pose_msg.child_frame_id and then rewrites only:
    //
    //   sim_pose_msg.header.frame_id = robot_odometry_frame_
    //
    // The pose payload is not changed. It still represents:
    //
    //   T:sim_world_reference_frame->robot_frame_
    //
    // After that header edit, sim_pose_cb manually adds the message to tf_filter_. Now the TF
    // filter sees:
    //
    //   target_frame                 = robot_frame_
    //   received_msg.header.frame_id = robot_odometry_frame_
    //
    // so it waits for:
    //
    //   T:robot_frame_->robot_odometry_frame_
    //
    // When that transform is available at sim_pose_msg.header.stamp, the filter calls update_cb.
    msg_filter_sub_{this, ""},
    sim_pose_sub_{this->create_subscription<nav_msgs::msg::Odometry>("sim_pose",
                                                                     rclcpp::SensorDataQoS(),
                                                                     std::bind(&FakeLocalization::sim_pose_cb,
                                                                               this,
                                                                               std::placeholders::_1))},
    tf_filter_{msg_filter_sub_,
               tf_buffer_,
               robot_frame_,
               10,
               this->get_node_logging_interface(),
               this->get_node_clock_interface()},
    // initialpose is handled with a normal ROS subscription. The callback requires
    // msg->header.frame_id to be global_frame_, so waiting for a TF transform from another frame
    // would not add useful behavior here.
    initial_pose_sub_{this->create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",
                                                     1,
                                                     std::bind(&FakeLocalization::init_pose_received_cb,
                                                               this,
                                                               std::placeholders::_1))},
    pose_pub_{this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1)},
    particle_cloud_pub_{this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 1)}
  {
    RCLCPP_DEBUG(this->get_logger(), "global_frame parameter set successfully to %s", global_frame_.c_str());
    RCLCPP_DEBUG(this->get_logger(),
                 "robot_odometry_frame parameter set successfully to %s",
                 robot_odometry_frame_.c_str());
    RCLCPP_DEBUG(this->get_logger(), "robot_frame parameter set successfully to %s", robot_frame_.c_str());

    const auto transform_tolerance{this->declare_parameter<double>("transform_tolerance", 0.1)};
    RCLCPP_DEBUG(this->get_logger(), "transform_tolerance parameter set successfully to %f", transform_tolerance);
    tolerance_secs_ = static_cast<int32_t>(transform_tolerance);
    tolerance_nanosecs_ = static_cast<uint32_t>((transform_tolerance - static_cast<double>(tolerance_secs_)) * 1e9);

    const auto delta_x{this->declare_parameter<double>("delta_x", 0.0)};
    RCLCPP_DEBUG(this->get_logger(), "delta_x parameter set successfully to %f", delta_x);

    const auto delta_y{this->declare_parameter<double>("delta_y", 0.0)};
    RCLCPP_DEBUG(this->get_logger(), "delta_y parameter set successfully to %f", delta_y);

    const auto delta_yaw{this->declare_parameter<double>("delta_yaw", 0.0)};
    RCLCPP_DEBUG(this->get_logger(), "delta_yaw parameter set successfully to %f", delta_yaw);

    // Create the timer interface before waitForTransform is used, to avoid a
    // tf2_ros::CreateTimerInterfaceException exception.
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                                     this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(timer_interface);
    // Register update_cb to be called when the required odometry transform is available.
    tf_filter_.registerCallback(&FakeLocalization::update_cb, this);

    // Notation: T:a->b means a homogeneous transform that represents the position and
    // orientation of frame b with respect to frame a.
    // Therefore, T:a->b converts coordinates expressed in frame b into frame a.

    // Parameters 'delta_x', 'delta_y', and 'delta_yaw' define the position and orientation of
    // 'global_frame' with respect to the sim world reference frame:
    // T:sim_world_reference_fr->global_fr.
    // Internally, this node needs T:global_fr->sim_world_reference_fr to convert poses coming
    // from the simulator into 'global_frame'.
    // Compute T:global_fr->sim_world_reference_fr as the inverse of
    // T:sim_world_reference_fr->global_fr.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, delta_yaw);
    const tf2::Transform T2_sim_world_reference_fr_global_fr{q, tf2::Vector3(delta_x, delta_y, 0.0)};
    T2_global_fr_sim_world_reference_fr_ = T2_sim_world_reference_fr_global_fr.inverse();
  }

  //////////////////////////////////////////////////////////////////////////////

  void FakeLocalization::sim_pose_cb(const nav_msgs::msg::Odometry::SharedPtr sim_pose_msg)
  {
    // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.

    // The incoming 'sim_pose_msg' is an odometry message used as a pose container, i.e., it does
    // not necessarily represent odometry in the semantic sense.
    // Its pose field represents the position and orientation of the `sim_pose_msg.child_frame_id`
    // with respect to the `sim_pose_msg.header.frame_id`.
    // In other words, the pose in the message represents the transform
    // T:sim_pose_msg.header.frame_id->sim_pose_msg.child_frame_id.

    // The 'sim_pose_msg.header.frame_id' is expected to be the simulated world's reference frame,
    // and the 'sim_pose_msg.child_frame_id' is expected to be the robot's frame, i.e., the
    // 'robot_frame_' parameter.
    // Therefore, the pose in the message represents the transform
    // T:sim_world_reference_frame->robot_frame_.
    // If child_frame_id is not the expected robot_frame_, reject the message and log an error.
    if(sim_pose_msg->child_frame_id != robot_frame_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(),
                            *this->get_clock(),
                            5000,
                            "Rejected sim_pose message because child_frame_id (%s) does not match "
                            "robot_frame (%s)",
                            sim_pose_msg->child_frame_id.c_str(),
                            robot_frame_.c_str());
      return;
    }

    // The goal of this node is to compute the transform `T:global_frame_->robot_odometry_frame_`
    // and publish it.

    // Procedure to compute the transform T:global_frame_->robot_odometry_frame_:

    // T:global_frame_->robot_frame_ =
    //   T:global_frame_->sim_world_reference_frame * T:sim_world_reference_frame->robot_frame_

    // T:global_frame_->sim_world_reference_frame is initialized in the constructor from the
    // parameters 'delta_x', 'delta_y', and 'delta_yaw'. It can later include corrections from
    // accepted initialpose messages.

    // T:global_frame_->robot_frame_ =
    //                T:global_frame_->robot_odometry_frame_ * T:robot_odometry_frame_->robot_frame_

    // T:global_frame_->robot_odometry_frame_ =
    //                  T:global_frame_->robot_frame_ * (T:robot_odometry_frame_->robot_frame_)^-1 =
    //                  T:global_frame_->robot_frame_ * T:robot_frame_->robot_odometry_frame_

    // By setting in this function the field 'sim_pose_msg->header.frame_id' to
    // 'robot_odometry_frame_', and adding the message to the TF filter, we ensure that the TF
    // filter will wait for the transform T:robot_frame_->robot_odometry_frame_ to be available at
    // the time specified in 'sim_pose_msg->header.stamp', and once it is available, it will call
    // the function 'update_cb'.

    // When 'update_cb' is called, it has access to:
    // - The `sim_pose_msg` message, whose pose field represents the transform
    //   T:sim_world_reference_frame->robot_frame_.
    // - The transform T:robot_frame_->robot_odometry_frame_, from the TF buffer.
    // Therefore, the function can compute the transform T:global_frame_->robot_odometry_frame_
    // using the mathematical derivation above, and then publish it using the tf_broadcaster_.

    sim_pose_msg->header.frame_id = robot_odometry_frame_;
    tf_filter_.add(sim_pose_msg);
  }

  void FakeLocalization::update_cb(const nav_msgs::msg::Odometry::SharedPtr sim_pose_msg)
  {
    // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.

    // If execution reaches this function, we can be sure that:
    // 1. The 'sim_pose_msg' message represents the transform
    //    T:sim_world_reference_frame->robot_frame_.
    // 2. The transform T:robot_frame_->robot_odometry_frame_ is available in the TF buffer at time
    //    sim_pose_msg->header.stamp.

    // This function obtains the transform T:global_frame_->robot_odometry_frame_ and publishes it.

    // Procedure to compute the transform T:global_frame_->robot_odometry_frame_:

    // T:global_frame_->robot_frame_ =
    //   T:global_frame_->sim_world_reference_frame * T:sim_world_reference_frame->robot_frame_

    // T:global_frame_->sim_world_reference_frame is initialized in the constructor from the
    // parameters 'delta_x', 'delta_y', and 'delta_yaw'. It can later include corrections from
    // accepted initialpose messages.

    tf2::Transform T2_sim_world_reference_fr_robot_fr;
    tf2::convert(sim_pose_msg->pose.pose, T2_sim_world_reference_fr_robot_fr);
    tf2::Transform T2_global_fr_robot_fr{T2_global_fr_sim_world_reference_fr_ * T2_sim_world_reference_fr_robot_fr};

    // T:global_frame_->robot_frame_ =
    //                T:global_frame_->robot_odometry_frame_ * T:robot_odometry_frame_->robot_frame_

    // T:global_frame_->robot_odometry_frame_ =
    //                  T:global_frame_->robot_frame_ * (T:robot_odometry_frame_->robot_frame_)^-1 =
    //                  T:global_frame_->robot_frame_ * T:robot_frame_->robot_odometry_frame_
    geometry_msgs::msg::TransformStamped Ts_robot_fr_odom_fr;

    try
    {
      Ts_robot_fr_odom_fr = tf_buffer_.lookupTransform(robot_frame_, robot_odometry_frame_, sim_pose_msg->header.stamp);
    }
    catch(tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to lookup transform from %s to %s: %s\n",
                   robot_frame_.c_str(),
                   robot_odometry_frame_.c_str(),
                   e.what());
      return;
    }

    tf2::Transform T2_robot_fr_odom_fr;
    tf2::convert(Ts_robot_fr_odom_fr.transform, T2_robot_fr_odom_fr);

    tf2::Transform T2_global_fr_odom_fr{T2_global_fr_robot_fr * T2_robot_fr_odom_fr};

    // Fill the header of the transform message to be published, T:global_fr->odom_fr.
    geometry_msgs::msg::TransformStamped Ts_global_fr_odom_fr;
    Ts_global_fr_odom_fr.header.frame_id = global_frame_;
    Ts_global_fr_odom_fr.child_frame_id = robot_odometry_frame_;
    // TF interprets 'header.stamp' as the time at which this transform is valid.
    // By adding 'transform_tolerance' to the pose timestamp, this node sets the timestamp of the
    // published T:global_fr->odom_fr transform slightly after the pose timestamp.
    // This is a common localization pattern: downstream nodes that request the transform at a time
    // close to "now" are less likely to fail because the latest published transform is older than
    // their requested time.
    Ts_global_fr_odom_fr.header.stamp.sec = sim_pose_msg->header.stamp.sec + tolerance_secs_;
    Ts_global_fr_odom_fr.header.stamp.nanosec = sim_pose_msg->header.stamp.nanosec + tolerance_nanosecs_;
    std::uint64_t nanosec_overflow{Ts_global_fr_odom_fr.header.stamp.nanosec / 1000000000};

    if(nanosec_overflow > 0)
    {
      Ts_global_fr_odom_fr.header.stamp.sec += nanosec_overflow;
      Ts_global_fr_odom_fr.header.stamp.nanosec -= nanosec_overflow * 1000000000;
    }

    // Convert from tf2::Transform to geometry_msgs::msg::Transform to publish the transform
    // T:global_fr->odom_fr.
    tf2::convert(T2_global_fr_odom_fr, Ts_global_fr_odom_fr.transform);
    tf_broadcaster_.sendTransform(Ts_global_fr_odom_fr);

    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_in_global_fr;
    robot_pose_in_global_fr.header.stamp = sim_pose_msg->header.stamp;
    robot_pose_in_global_fr.header.frame_id = global_frame_;
    robot_pose_in_global_fr.pose.pose.position.x = T2_global_fr_robot_fr.getOrigin().x();
    robot_pose_in_global_fr.pose.pose.position.y = T2_global_fr_robot_fr.getOrigin().y();
    robot_pose_in_global_fr.pose.pose.position.z = T2_global_fr_robot_fr.getOrigin().z();
    robot_pose_in_global_fr.pose.pose.orientation = tf2::toMsg(T2_global_fr_robot_fr.getRotation());
    pose_pub_->publish(robot_pose_in_global_fr);

    // The particle cloud contains one particle at the same pose published in
    // robot_pose_in_global_fr.
    geometry_msgs::msg::PoseArray particle_cloud;
    particle_cloud.header = robot_pose_in_global_fr.header;
    particle_cloud.poses.resize(1);
    particle_cloud.poses[0] = robot_pose_in_global_fr.pose.pose;
    particle_cloud_pub_->publish(particle_cloud);
  }

  void FakeLocalization::init_pose_received_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.

    // The "2D Pose Estimate" tool in RViz publishes a PoseWithCovarianceStamped message on the
    // initialpose topic. In the usual RViz setup, msg->header.frame_id is the RViz fixed frame.
    // This callback only accepts that message when the RViz fixed frame is the same frame as
    // global_frame_. Otherwise the selected pose would be expressed in a different frame from the
    // one used by fake_localization.
    if(msg->header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Frame ID of \"initialpose\" (%s) is different from the global reference frame %s",
                   msg->header.frame_id.c_str(),
                   global_frame_.c_str());
      return;
    }

    // The pose selected in RViz is interpreted as the desired pose of robot_frame_ in
    // global_frame_.
    // In transform notation, the selected pose is:
    //
    //   T:global_frame_->desired_robot_frame
    //
    // The selected pose does not move the robot in Gazebo and it does not change the incoming
    // sim_pose data. It asks fake_localization to change its internal relationship between
    // global_frame_ and the sim world reference frame so that the robot pose published by this node
    // appears at the selected pose.
    tf2::Transform T2_global_fr_desired_robot_fr;
    tf2::convert(msg->pose.pose, T2_global_fr_desired_robot_fr);

    // Before applying the selected pose, fake_localization is already publishing a current robot
    // pose in global_frame_:
    //
    //   T:global_frame_->robot_frame_ =
    //     T:global_frame_->sim_world_reference_frame *
    //     T:sim_world_reference_frame->robot_frame_
    //
    // T:global_frame_->sim_world_reference_frame is initialized from the delta_x, delta_y, and
    // delta_yaw parameters. If this callback has already accepted an initialpose message before,
    // the same transform also includes the previous RViz-based corrections.
    //
    // The selected RViz pose is the desired new value for T:global_frame_->robot_frame_.
    // Therefore, this callback looks for a correction transform that satisfies:
    //
    //   T:global_frame_->desired_robot_frame =
    //     T2_frame_correction * T:global_frame_->robot_frame_
    //
    // which gives:
    //
    //   T2_frame_correction =
    //     T:global_frame_->desired_robot_frame * (T:global_frame_->robot_frame_)^-1
    //     T:global_frame_->desired_robot_frame * T:robot_frame_->global_frame_
    //
    // The inverse of the current published robot pose is obtained from the TF tree. The latest
    // available transform is used because this callback is handling an interactive user command,
    // not a time-synchronized simulator pose message.
    geometry_msgs::msg::TransformStamped Ts_robot_fr_global_fr;
    try
    {
      Ts_robot_fr_global_fr = tf_buffer_.lookupTransform(robot_frame_, global_frame_, rclcpp::Time(0));
    }
    catch(tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", e.what());
      return;
    }

    tf2::Transform T2_robot_fr_global_fr;
    tf2::convert(Ts_robot_fr_global_fr.transform, T2_robot_fr_global_fr);

    tf2::Transform T2_frame_correction{T2_global_fr_desired_robot_fr * T2_robot_fr_global_fr};

    // Apply the correction to the transform used to convert simulator poses into global_frame_:
    //
    //   T:global_frame_->robot_frame_ =
    //     T:global_frame_->sim_world_reference_frame *
    //     T:sim_world_reference_frame->robot_frame_
    //
    //   T:global_frame_->desired_robot_frame =
    //     T2_frame_correction * T:global_frame_->robot_frame_ =
    //     T2_frame_correction * T:global_frame_->sim_world_reference_frame *
    //     T:sim_world_reference_frame->robot_frame_
    //
    //   T:global_frame_->desired_robot_frame =
    //     (T2_frame_correction * T:global_frame_->sim_world_reference_frame) *
    //     T:sim_world_reference_frame->robot_frame_
    //
    // The term in parentheses is the new internal value of
    // T:global_frame_->sim_world_reference_frame.

    // In practical terms, clicking a pose in RViz changes the offset between global_frame_ and the
    // sim world reference frame so that the robot appears at the selected pose in global_frame_.
    T2_global_fr_sim_world_reference_fr_ = T2_frame_correction * T2_global_fr_sim_world_reference_fr_;
  }
}  // namespace fake_localization
