#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath> // used it for pi
#include <novatel_oem7_msgs/msg/rawimu.hpp>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (TakeHome::odometry_callback in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_wheel_angle_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_wheel_angle_callback, this, std::placeholders::_1));

    imu_top_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_top_callback, this, std::placeholders::_1));

    curvlin_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "/curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvlin_callback, this, std::placeholders::_1));

      metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
      slip_rr_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
      slip_rl_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
      slip_fr_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
      slip_fl_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
      imu_jitter_publisher = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
      lap_time_publisher = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running ros2 bag info on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);

  vx = odom_msg->twist.twist.linear.x;
  vy = odom_msg->twist.twist.linear.y;
  yaw_rate = odom_msg->twist.twist.angular.z; // angular velocity

  calc_slip(); // helper to calculate wheel slips
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wsped_msg) {  
  fl = wsped_msg->front_left * 1000 / 3600;
  fr = wsped_msg->front_right * 1000 / 3600;
  rl = wsped_msg->rear_left * 1000 / 3600;
  rr = wsped_msg->rear_right * 1000 / 3600;

  got_wheel_speed_ = true; // data for wheel speed is ready
}

void TakeHome::steering_wheel_angle_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr wangle_msg) {
  steering_angle = wangle_msg->primary_steering_angle_fbk;
  got_steering_angle_ = true; // data for steering ang readyy
}

void TakeHome::calc_slip() {
  const double wr = 1.523;
  const double wf = 1.638;
  const double lf = 1.7238;

  double delta = (steering_angle / 15.0) * M_PI / 180.0;

  // rr
  double vx_rr = vx - 0.5 * yaw_rate * wr;
  double k_rr = (rr - vx_rr) / vx_rr;

  // rl
  double vx_rl = vx + 0.5 * yaw_rate * wr;
  double k_rl = (rl - vx_rl) / vx_rl;

  // fr
  double vx_fr = vx - 0.5 * yaw_rate * wf;
  double vy_fr = vy + yaw_rate * lf;
  double vx_fr_d = cos(delta) * vx_fr - sin(delta) * vy_fr;
  double k_fr = (fr - vx_fr_d) / vx_fr_d;

  // fl
  double vx_fl = vx + 0.5 * yaw_rate * wf;
  double vy_fl = vy + yaw_rate * lf;
  double vx_fl_d = cos(delta) * vx_fl - sin(delta) * vy_fl;
  double k_fl = (fl - vx_fl_d) / vx_fl_d;

  auto pub_msg = [](double val) {
    std_msgs::msg::Float32 msg;
    msg.data = val;
    return msg;
  };

  slip_rr_publisher->publish(pub_msg(k_rr));
  slip_rl_publisher->publish(pub_msg(k_rl));
  slip_fr_publisher->publish(pub_msg(k_fr));
  slip_fl_publisher->publish(pub_msg(k_fl));

  // Kept getting -1 for wheel slip ratio due to dividing by 0 (at least I think)
  RCLCPP_INFO(this->get_logger(), "vx: %.3f, yaw: %.3f, rr: %.3f, vx_rr: %.3f, k_rr: %.3f", vx, yaw_rate, rr, vx_rr, k_rr);
}

void TakeHome::imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
  rclcpp::Time current_time = imu_msg->header.stamp;
  if (last_time.nanoseconds() == 0) { // Apparently Time doesnt have a built in is_zero function
    last_time = current_time;
    return;
  }
  double dt = (current_time - last_time).seconds(); // dt since last time recorded
  last_time = current_time;

  jitters.push_back(dt);
  jitter_times.push_back(current_time);

  while (!jitter_times.empty() && (current_time - jitter_times.front()).seconds() > 1.0) { // 1 sec sliding window
    jitters.pop_front();
    jitter_times.pop_front();
  }
  double mean = 0.0;
  for (double val : jitters) mean += val;
  mean /= jitters.size();

  double variance = 0.0;
  for (double val : jitters) variance += (val - mean) * (val - mean);
  variance /= jitters.size();

  std_msgs::msg::Float32 msg;
  msg.data = variance;
  imu_jitter_publisher->publish(msg);
}

void TakeHome::curvlin_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
  double curv_dist = msg->data;
  if (last_distance > 100.0 && curv_dist < 10.0) { // detects a new lap if frossing happens >100 to <10 m
    rclcpp::Time now = this->get_clock()->now();

    if (lap_started) {
      double lap_time = (now - lap_start_time).seconds();
      std_msgs::msg::Float32 msg_out;
      msg_out.data = lap_time;
      lap_time_publisher->publish(msg_out);
    }
    lap_start_time = now;
    lap_started = true;
  }
  last_distance = curv_dist;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)