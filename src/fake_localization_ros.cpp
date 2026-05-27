#include <fake_localization.hpp>
#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

namespace fake_localization
{
  // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.

  // Main constructor with custom node name
  FakeLocalization::FakeLocalization(const rclcpp::NodeOptions& options):
    Node("fake_localization", options),
    global_frame_{this->declare_parameter<std::string>("global_frame", std::string{"map"})},
    odometry_frame_{this->declare_parameter<std::string>("odometry_frame", std::string{"odom"})},
    robot_base_frame_{this->declare_parameter<std::string>("robot_base_frame", std::string{"base_link"})},
    transform_tolerance_{this->declare_parameter<double>("transform_tolerance", 0.1)},
    tf_buffer_{this->get_clock()},
    tf_listener_{tf_buffer_, this, true},
    tf_broadcaster_{*this},
    // The 'tf_filter_' (tf2_ros::MessageFilter) needs a 'message_filters::Subscriber' to be created, and that's why we
    // have 'msg_filter_sub_'.
    // However, the odometry messages coming in the topic 'base_pose_ground_truth' may not have a frame_id set,
    // in the header ('header.frame_id') or it might have a frame_id that is different from what 'tf_filter_' needs,
    // which is the odometry frame, i.e; the 'odometry_frame_' parameter.
    // For that reason, even though the 'msg_filter_sub_' is created (to construct the 'tf_filter_'), we need to use
    // a regular subscription ('base_pose_ground_truth_sub_') to receive the odometry messages (in the associated
    // callback) to SET the correct frame_id in its header, and after that editing operation the msg is passed to the
    // 'tf_filter_'. This way we can ensure the 'tf_filter_' waits for the correct transform
    // T:<target_frame=robot_base_frame> -> <odom_msg->header.frame_id>, and once the transform is available, the 'update_cb'
    // method is called.
    // Note: It is also valid to receive the tranformation T:<odom_msg->header.frame_id> -> <target_frame=robot_base_frame>,
    // since one transformation can be inverted to get the other and vice versa. And, in fact this last
    // transformation shown above is very likely to be the one received in the 'tf_buffer_', since it is the
    // transformation that usually is broadcasted by the robot simulation or the robot odometry driver (the source of
    // the odometry data in use. Only one source of odometry data must be in used, not both, since if both are used the
    // transformations coming from both sources will overwrite each other in the 'tf_buffer_', leading to erratic
    // behavior). However, from a pure theoretical point of view, it is more correct to set the 'odometry_frame_' in
    // the 'odom_msg->header.frame_id' in the subscription callback, and the obviously the target frame in the
    // 'tf_filter_' constructor must set to 'robot_base_frame_', leading to the waiting for the transformation
    // T:<target_frame=robot_base_frame> -> <odom_msg->header.frame_id>, although we have in mind to use/receive the inverse
    // transformation, T:<odom_msg->header.frame_id> -> <target_frame=robot_base_frame>.
    // To say in other words, we could have set the target frame in the 'tf_filter_' to 'odometry_frame_' and then
    // in the subscription callback set the 'odom_msg->header.frame_id' to 'robot_base_frame_', leading to the waiting for the
    // transformation T:<target_frame=odometry_frame_> -> <odom_msg->header.frame_id = robot_base_frame_>, but probably this
    // will lead to confusion when reading the code, since what you expect to see/set in the 'odom_msg->header.frame_id'
    // is the odometry frame.
    // However, both approaches explained here are valid from a pure theoretical point of view, since they provide the
    // same result, which is to get:
    // T:<robot_base_frame> -> <odometry_frame_> and its inverse. T:<odometry_frame_> -> <robot_base_frame>.
    msg_filter_sub_{this, ""},
    base_pose_ground_truth_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
      "base_pose_ground_truth",
      rclcpp::SensorDataQoS(),
      std::bind(&FakeLocalization::ground_truth_cb, this, std::placeholders::_1))},
    // tf_filter will wait for transforms T_<target_frame=robot_base_frame>_<odom_msg->header.frame_id>.
    // Once the transform is available, the 'update_cb' is called.
    tf_filter_{msg_filter_sub_,
               tf_buffer_,
               robot_base_frame_,
               10,
               this->get_node_logging_interface(),
               this->get_node_clock_interface()},
    // In this second pair of 'messages_filters::Subscriber' and 'tf2_ros::MessageFilter', we handle the initial pose
    // estimates coming from RViz, in the topic 'initialpose', without using a rare manipulation of the
    // 'header.frame_id' as we did before. In this case, the 'message_filters::Subscriber' can subscribe directly to the
    // topic 'initialpose' and pass it to the 'init_pose_filter_'. Besides, the 'initial_pose_filter_' waits for the
    // transform T_<target_frame=global_frame_><pose_with_covariance_stamped_msg->header.frame_id>, and once the
    // transform is available, the function 'init_pose_received_cb' is called.
    // Subscription to '2D Pose estimate' from RViz.
    initital_pose_sub_{this, "initialpose"},
    // 'init_pose_filter_' will wait for transforms
    // T_<target_frame=global_frame>_<pose_with_covariance_stamped_msg->header.frame_id>.
    // Once the transform is available, the function 'init_pose_received_cb' is called.
    init_pose_filter_{
      initital_pose_sub_,
      tf_buffer_,
      global_frame_,
      1,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
    },
    pose_pub_{this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1)},
    particle_cloud_pub_{this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 1)}
  {
    RCLCPP_DEBUG(this->get_logger(), "global_frame parameter set successfully to %s", global_frame_.c_str());
    RCLCPP_DEBUG(this->get_logger(), "odometry_frame parameter set successfully to %s", odometry_frame_.c_str());
    RCLCPP_DEBUG(this->get_logger(), "robot_base_frame parameter set successfully to %s", robot_base_frame_.c_str());

    RCLCPP_DEBUG(this->get_logger(), "transform_tolerance parameter set successfully to %f", transform_tolerance_);

    const auto delta_x{this->declare_parameter<double>("delta_x", 0.0)};
    RCLCPP_DEBUG(this->get_logger(), "delta_x parameter set successfully to %f", delta_x);

    const auto delta_y{this->declare_parameter<double>("delta_y", 0.0)};
    RCLCPP_DEBUG(this->get_logger(), "delta_y parameter set successfully to %f", delta_y);

    const auto delta_yaw{this->declare_parameter<double>("delta_yaw", 0.0)};
    RCLCPP_DEBUG(this->get_logger(), "delta_yaw parameter set successfully to %f", delta_yaw);


    // const auto msg_rx_timeout = this->declare_parameter<double>("msg_reception_timeout", 1.0);

    // if(msg_rx_timeout < 0.0)
    // {
    //   throw std::runtime_error("msg_reception_timeout parameter must be non-negative");
    // }

    // std::chrono::milliseconds msg_rx_timeout_ms{static_cast<int>(1000.0 * msg_rx_timeout)};

    // RCLCPP_INFO(this->get_logger(),
    //             "msg_reception_timeout parameter set successfully to %d ms",
    //             msg_rx_timeout_ms.count());

    // int timeout_waiting_for_first_transform{1000 *
    //                                         this->declare_parameter<int>("timeout_waiting_for_first_transform", 10)};

    // RCLCPP_INFO(this->get_logger(),
    //             "timeout_waiting_for_first_transform parameter set successfully to %ld",
    //             timeout_waiting_for_first_transform);

    // Create the timer interface before call to waitForTransform, to avoid a tf2_ros::CreateTimerInterfaceException
    // exception.
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                                     this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(timer_interface);
    // Since the tf_listener_ uses its own thread (third argument true above), we let the tf_buffer_ know that
    // multiple threads will be accessing it, the thread running this node and the tf_listener_ thread.
    tf_buffer_.setUsingDedicatedThread(true);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf_filter_.registerCallback(&FakeLocalization::update_cb, this);
    init_pose_filter_.registerCallback(&FakeLocalization::init_pose_received_cb, this);

    // Parameters 'delta_x', 'delta_y', and 'delta_yaw' define the position and orientation of the 'global_frame' w.r.t
    // the simulator's coordinate frame: T:sim_fr->global_fr (FROM the sim_fr TO the global_fr).
    // However, we need the tranformation T:global_fr->sim_fr (FROM the global_fr TO the sim_fr), to convert poses
    // coming from the simulator into the 'global_frame', and for that reason we need to negate the 'delta' parameters.
    // Example: If delta_x = 5.0 (map is 5m east of simulator), then to convert a point from simulator coords to map
    // coords, we subtract 5.0 (move west), hence -delta_x.

    // Mathematical relationship: T:global_fr->sim_fr = inverse(T:sim_fr->global_fr)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -delta_yaw);
    Tf2_global_fr_sim_fr_ = tf2::Transform(q, tf2::Vector3(-delta_x, -delta_y, 0.0));
  }

  //////////////////////////////////////////////////////////////////////////////

  void FakeLocalization::ground_truth_cb(const nav_msgs::msg::Odometry::SharedPtr base_pose_ground_truth_msg)
  {
    // 'msg' is the 'base_pose_ground_truth' odometry message coming from the simulator, and it contains the
    // position and orientation of the robot in the simulator's world-fixed frame.

    // The frame_id in 'msg->header.frame_id' can be anything, from no frame_id at all to any string like 'map',
    // 'world', 'sim', etc.

    // We are going to 'force' the 'tf_filter_' to wait for transformations 'T:base_fr->odom_fr':
    // T_<target_frame=robot_base_frame>_<odom_msg->header.frame_id>

    // To do that we have to do two things, we already did the first one when the element 'tf_filter_' was constructed,
    // by indicating the 'target_frame = robot_base_frame_', which is the frame the filter will transform data into from
    // the frame indicated in the header of the odometry messages received (base_pose_ground_truth messages).
    // Second thing is to change the frame id of 'msg' (base_pose_ground_truth message) to be equal to the odometry
    // frame, i.e; to 'odometry_frame_'.
    // I know, I know, the frame_id of the 'msg' is, and should be, 'map' or 'world' or 'sim', or empty, because the
    // pose contained in 'msg' (base_pose_ground_truth messages) represents the position and orientation of the robot in
    // the world-fixed frame of the simulator, as said before.
    // However to 'trick' the 'tf_filter_' to listen to the transform 'T:base_fr->odom_fr', we have to do this trick,
    // and it does not have side effects.
    base_pose_ground_truth_msg->header.frame_id = odometry_frame_;

    // After we added the received 'msg' into the 'tf_filter_', with the 'tricked' frame_id, the moment a transformation
    // 'T:base_fr->odom_fr' is received, at the time (more or less) of the 'msg', the 'update_cb' callback is called.
    tf_filter_.add(base_pose_ground_truth_msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  void FakeLocalization::update_cb(const nav_msgs::msg::Odometry::SharedPtr base_pose_ground_truth_msg)
  {
    // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.

    // This function is used to obtain the transform T:global_fr->odom_fr.

    // If the execution flows is here, we can be sure that:
    // 1. The element 'msg' (base_pose_ground_truth message) contains the position and orientation of the robot in the
    // simulator's frame. This is a global pose: T:sim_fr->base_fr.
    // 2. The transform T:base_fr->odom_fr is available at time msg->header.stamp (more or less).

    // T:global_fr->base_fr = T:global_fr->sim_fr * T:sim_fr->base_fr
    tf2::Transform Tf2_sim_fr_base_fr;
    tf2::convert(base_pose_ground_truth_msg->pose.pose, Tf2_sim_fr_base_fr);
    tf2::Transform Tf2_global_fr_base_fr{Tf2_global_fr_sim_fr_ * Tf2_sim_fr_base_fr};

    // The function needs to publish T:global_fr->odom_fr, but what we can compute right now, based on the available
    // data is T:odom_fr->global_fr, and then we will invert it to get the desired transform.
    geometry_msgs::msg::TransformStamped T_odom_fr_global_fr;

    try
    {
      // Compute T:base_fr->global_fr.
      // T:base_fr->global_fr = (T:global_fr->base_fr)^-1
      geometry_msgs::msg::TransformStamped T_base_fr_global_fr;
      T_base_fr_global_fr.header.frame_id = robot_base_frame_;
      T_base_fr_global_fr.header.stamp    = base_pose_ground_truth_msg->header.stamp;
      tf2::convert(Tf2_global_fr_base_fr.inverse(), T_base_fr_global_fr.transform);

      // The transform function works as this:
      // Note: the 'target_frame' is the third argument of the function.
      // 1. Look for the transform T:<target_frame> -> <first_parameter.header.frame_id>
      //    target_frame = odometry_frame_
      //    first_parameter.header.frame_id = robot_base_frame_
      //    So, it looks for T:odom_fr->base_fr.
      // 2. Compute the output transform as:
      //    output = T:<target_frame> -> <first_parameter.header.frame_id> * first_parameter.transform
      //    output = T:odom_fr->base_fr * T:base_fr->global_fr = T:odom_fr->global_fr
      //    So, the output is T:odom_fr->global_fr
      tf_buffer_.transform(T_base_fr_global_fr, T_odom_fr_global_fr, odometry_frame_);
    }
    catch(tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to transform to %s from %s: %s\n",
                   odometry_frame_.c_str(),
                   robot_base_frame_.c_str(),
                   e.what());
      return;
    }

    // At this point, we have T:odom_fr->global_fr computed.
    // The function needs to publish T:global_fr->odom_fr = (T:odom_fr->global_fr)^-1
    // First, we fill the header of the transform message to be published, T:global_fr->odom_fr.
    geometry_msgs::msg::TransformStamped T_global_fr_odom_fr;
    T_global_fr_odom_fr.header.frame_id = global_frame_;
    T_global_fr_odom_fr.child_frame_id  = odometry_frame_;
    // In ROS, the 'header.stamp' field of a transform (such as in TransformStamped) not only indicates the exact time
    // at which the transformation should be applied, but also, in practice, acts as the time range within which the
    // transformation can be considered valid or approximate for some consumers.
    // By adding a transform_tolerance to the timestamp, the transform is made available for a small window into the
    // future.
    // This helps downstream nodes handle small message or processing delays more robustly, avoiding errors when an
    // exact timestamp match is unavailable and enabling interpolation.
    int32_t toler_secs{static_cast<int32_t>(transform_tolerance_)};
    uint32_t toler_nanosecs{static_cast<uint32_t>((transform_tolerance_ - static_cast<double>(toler_secs)) * 1e9)};
    T_global_fr_odom_fr.header.stamp.sec     = base_pose_ground_truth_msg->header.stamp.sec + toler_secs;
    T_global_fr_odom_fr.header.stamp.nanosec = base_pose_ground_truth_msg->header.stamp.nanosec + toler_nanosecs;
    unsigned long int nanosec_overflow{T_global_fr_odom_fr.header.stamp.nanosec / 1000000000};

    if(nanosec_overflow > 0)
    {
      T_global_fr_odom_fr.header.stamp.sec += static_cast<unsigned long int>(nanosec_overflow);
      T_global_fr_odom_fr.header.stamp.nanosec -= nanosec_overflow * 1000000000;
    }

    // We need to invert 'T_odom_fr_global_fr', but to do that we have to convert from T_... to Tf2_... to do the
    // required computations, i.e; change data types.
    // Get Tf2:odom_fr->global_fr from T:odom_fr->global_fr message.
    tf2::Transform Tf2_odom_fr_global_fr;
    tf2::convert(T_odom_fr_global_fr.transform, Tf2_odom_fr_global_fr);

    // Get Tf2:global_fr->odom_fr, by inverting Tf2:odom_fr->global_fr.
    tf2::Transform Tf2_global_fr_odom_fr{Tf2_odom_fr_global_fr.inverse()};

    // Convert data type again, from Tf2:global_fr->odom_fr to T:global_fr->odom_fr message to be published.
    tf2::convert(Tf2_global_fr_odom_fr, T_global_fr_odom_fr.transform);

    // Publish the transform T:global_fr->odom_fr.
    tf_broadcaster_.sendTransform(T_global_fr_odom_fr);

    tf2::Transform Tf2_sim_fr;  // Represents the position and orientation of the robot in the simulator frame.
    tf2::convert(base_pose_ground_truth_msg->pose.pose, Tf2_sim_fr);

    // Represents the position and orientation of the robot in the global frame, in format tf2::Transform.
    tf2::Transform Tf2_global_fr{Tf2_global_fr_sim_fr_ * Tf2_sim_fr};

    // Change data type from tf2::Transform to geometry_msgs::msg::Transform.
    geometry_msgs::msg::Transform T_global_fr;
    tf2::convert(Tf2_global_fr, T_global_fr);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_global_fr;
    pose_global_fr.header.stamp    = base_pose_ground_truth_msg->header.stamp;
    pose_global_fr.header.frame_id = global_frame_;
    tf2::convert(T_global_fr.rotation, pose_global_fr.pose.pose.orientation);
    pose_global_fr.pose.pose.position.x = T_global_fr.translation.x;
    pose_global_fr.pose.pose.position.y = T_global_fr.translation.y;
    pose_global_fr.pose.pose.position.z = T_global_fr.translation.z;
    pose_pub_->publish(pose_global_fr);

    // The particle cloud is the pose_global_fr position. Quite convenient.
    geometry_msgs::msg::PoseArray particle_cloud;
    particle_cloud.header = pose_global_fr.header;
    particle_cloud.poses.resize(1);
    particle_cloud.poses[0] = pose_global_fr.pose.pose;
    particle_cloud_pub_->publish(particle_cloud);
  }

  void FakeLocalization::init_pose_received_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // Notation: T:a->b means transform that converts coordinates expressed in frame b into frame a.

    // This represents the pose chosen with the "2D Pose Estimate" tool in RViz w.r.t the global frame.
    // T:global_fr->desired_base_fr
    tf2::Transform Tf2_global_fr_desired_base_fr;
    tf2::convert(msg->pose.pose, Tf2_global_fr_desired_base_fr);

    if(msg->header.frame_id != global_frame_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Frame ID of \"initialpose\" (%s) is different from the global frame %s",
                  msg->header.frame_id.c_str(),
                  global_frame_.c_str());
    }

    // The function is looking for a delta so that:
    // T:global_fr->desired_base_fr = delta * T:global_fr->current_base_fr
    // delta = T:global_fr->desired_base_fr * (T:global_fr->current_base_fr)^-1
    //       = T:global_fr->desired_base_fr * T:current_base_fr->global_fr

    geometry_msgs::msg::TransformStamped T_current_base_fr_global_fr;
    try
    {
      T_current_base_fr_global_fr = tf_buffer_.lookupTransform(robot_base_frame_, global_frame_, rclcpp::Time(0));
    }
    catch(tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", e.what());
      return;
    }

    tf2::Transform Tf2_current_base_fr_global_fr;
    tf2::convert(T_current_base_fr_global_fr.transform, Tf2_current_base_fr_global_fr);

    tf2::Transform delta{Tf2_global_fr_desired_base_fr * Tf2_current_base_fr_global_fr};

    // When a pose is received from the simulator, i.e T:sim_fr->current_base_fr, we transform that pose into the
    // global frame:
    // T:global->current_base_fr = T:global_fr->sim_fr * T:sim_fr->current_base_fr
    // Next we transform that pose into the desired pose:
    // T:global_fr->desired_base_fr = delta * T:global_fr->current_base_fr
    //                              = delta * T:global_fr->sim_fr * T:sim_fr->current_base_fr
    //                              =(delta * T:global_fr->sim_fr) * T:sim_fr->current_base_fr
    //                               ----------------------------
    //                                new value of T:global_fr->sim_fr
    Tf2_global_fr_sim_fr_ = delta * Tf2_global_fr_sim_fr_;
  }
}  // namespace fake_localization
