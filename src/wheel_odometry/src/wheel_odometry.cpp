#include "wheel_odometry/wheel_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robonav_training_common/angles.hpp"

namespace robonav_training
{

static int findJointIndex(
  const sensor_msgs::msg::JointState & msg, const std::string & name)
{
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

WheelOdometry::WheelOdometry(const rclcpp::NodeOptions & options)
: Node("wheel_odometry", options)
{
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
  wheel_separation_ = declare_parameter<double>("wheel_separation", 0.34);
  left_joint_name_ = declare_parameter<std::string>("left_joint_name", "left_wheel_joint");
  right_joint_name_ = declare_parameter<std::string>("right_joint_name", "right_wheel_joint");

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&WheelOdometry::onJointState, this, std::placeholders::_1));
}

void WheelOdometry::onJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const int li = findJointIndex(*msg, left_joint_name_);
  const int ri = findJointIndex(*msg, right_joint_name_);
  
  // validating the joints found
  if (li < 0 || ri < 0 ||
    li >= static_cast<int>(msg->position.size()) ||
    ri >= static_cast<int>(msg->position.size()))
  {
    return;
  }

  const double left = msg->position[li];
  const double right = msg->position[ri];   
  const rclcpp::Time stamp(msg->header.stamp);

  // if the first run, update values
  if (!have_prev_) {
    left_prev_ = left;
    right_prev_ = right;
    stamp_prev_ = stamp;
    have_prev_ = true;

    publishOdometry(stamp, 0.0, 0.0);
    return;
  }

  // validate dt
  const double dt = (stamp - stamp_prev_).seconds();
  if (dt <= 0.0) {
    return;
  }

  const double left_delta = left - left_prev_;
  const double right_delta = right - right_prev_;
  const double d_left = left_delta * wheel_radius_;
  const double d_right = right_delta * wheel_radius_;
  const double d_center = (d_left + d_right) / 2.0;
  const double d_theta = (d_right - d_left) / wheel_separation_;

  const double mid_theta = theta_ + d_theta / 2.0;

  x_ += d_center * std::cos(mid_theta);
  y_ += d_center * std::sin(mid_theta);
  theta_ = normalizeAngle(theta_ + d_theta);

  const double vx = d_center / dt;
  const double wz = d_theta / dt;

  // update baseline
  left_prev_ = left;
  right_prev_ = right;
  stamp_prev_ = stamp;

  publishOdometry(stamp, vx, wz);
}

void WheelOdometry::publishOdometry(
  const rclcpp::Time & stamp, double vx, double wz)
{
  nav_msgs::msg::Odometry odom;
  
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = wz;

  odom.pose.covariance[0] = 1e-3;
  odom.pose.covariance[7] = 1e-3;
  odom.pose.covariance[35] = 1e-2;

  odom.twist.covariance[0] = 1e-3;
  odom.twist.covariance[35] = 1e-2;
  
  odom_pub_->publish(odom);
}

}  // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::WheelOdometry)