#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <deque> // sliding window of imu timestamps

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wsped_msg);
  void steering_wheel_angle_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr wangle_msg);
  void imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);
  void curvlin_callback(std_msgs::msg::Float32::ConstSharedPtr cdist_msg);

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_wheel_angle_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_top_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_jitter_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvlin_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher;

  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
  double fl = 0.0;
  double fr = 0.0;
  double rl = 0.0;
  double rr = 0.0;
  double steering_angle = 0.0;

  // make sure wheel speed + steering are a thing before doing calculations
  bool got_wheel_speed_ = false;
  bool got_steering_angle_ = false;

  std::deque<double> jitters;
  std::deque<rclcpp::Time> jitter_times;
  rclcpp::Time last_time;

  double last_distance = 0.0;
  rclcpp::Time lap_start_time;
  bool lap_started = false; // Lap start tracker

  void calc_slip();
};
