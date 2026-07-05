#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <message_filters/subscriber.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>

namespace fake_localization
{
  class FakeLocalization: public rclcpp::Node
  {
    public:
    explicit FakeLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void sim_pose_cb(const nav_msgs::msg::Odometry::SharedPtr sim_pose_msg);
    void update_cb(const nav_msgs::msg::Odometry::SharedPtr sim_pose_msg);
    void init_pose_received_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    private:
    std::string global_frame_;
    std::string robot_odometry_frame_;
    std::string robot_frame_;

    int32_t tolerance_secs_;
    uint32_t tolerance_nanosecs_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // msg_filter_sub_ is needed to construct tf_filter_, but it does not subscribe to the
    // sim_pose topic directly. The cpp file explains why sim_pose_sub_ receives the message first
    // and then passes it to tf_filter_ manually.
    message_filters::Subscriber<nav_msgs::msg::Odometry> msg_filter_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_pose_sub_;
    tf2_ros::MessageFilter<nav_msgs::msg::Odometry> tf_filter_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;

    tf2::Transform T2_global_fr_sim_world_reference_fr_;
  };

}  // namespace fake_localization
