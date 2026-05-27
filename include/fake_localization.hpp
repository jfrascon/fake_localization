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

    void ground_truth_cb(const nav_msgs::msg::Odometry::SharedPtr base_pose_ground_truth_msg);
    void update_cb(const nav_msgs::msg::Odometry::SharedPtr base_pose_ground_truth_msg);
    void init_pose_received_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    private:
    std::string global_frame_;
    std::string odometry_frame_;
    std::string robot_base_frame_;

    double transform_tolerance_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // msg_filter_sub is necessary to create tf_filter_ (tf2_ros::MessageFilter); however, for a reason that is
    // described in the cpp file, 'msg_filter_sub_' is not actually used to subscribe to the odometry topic.
    // Instead, 'base_pose_ground_truth_sub_' is used to subscribe to the odometry topic and pass it to tf_filter_.
    // Rationale for this rare design choice is explained in the cpp file.
    message_filters::Subscriber<nav_msgs::msg::Odometry> msg_filter_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_pose_ground_truth_sub_;
    tf2_ros::MessageFilter<nav_msgs::msg::Odometry> tf_filter_;

    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> initital_pose_sub_;
    tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped> init_pose_filter_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;

    tf2::Transform Tf2_global_fr_sim_fr_;
  };

}  // namespace fake_localization
