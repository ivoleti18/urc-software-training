#pragma once

#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robonav_training
{

class WheelOdometry : public rclcpp::Node
{
public:
  explicit WheelOdometry(const rclcpp::NodeOptions & options);

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishOdometry(const rclcpp::Time & stamp, double vx, double wz);

  double wheel_radius_;
  double wheel_separation_;
  std::string left_joint_name_;
  std::string right_joint_name_;

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  bool have_prev_{false};
  double left_prev_{0.0};
  double right_prev_{0.0};
  rclcpp::Time stamp_prev_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

};

}  // namespace robonav_training