/**
 * @file odometry_publisher.cpp
 * @brief Odometry Publisher for AMR-POLEBOT
 *
 * Reads encoder data from motor driver and computes/publishes odometry.
 * AMR-POLEBOT | Politeknik Manufaktur Bandung
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "polebot_interfaces/msg/motor_status.hpp"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_publisher"),
    x_(0.0), y_(0.0), theta_(0.0),
    last_left_enc_(0), last_right_enc_(0),
    first_reading_(true)
  {
    this->declare_parameter("wheel_radius",         0.065);
    this->declare_parameter("wheel_separation",     0.35);
    this->declare_parameter("ticks_per_revolution", 4096);
    this->declare_parameter("odom_frame",           "odom");
    this->declare_parameter("base_frame",           "base_footprint");
    this->declare_parameter("publish_tf",           true);

    wheel_radius_         = this->get_parameter("wheel_radius").as_double();
    wheel_separation_     = this->get_parameter("wheel_separation").as_double();
    ticks_per_rev_        = this->get_parameter("ticks_per_revolution").as_int();
    odom_frame_           = this->get_parameter("odom_frame").as_string();
    base_frame_           = this->get_parameter("base_frame").as_string();
    publish_tf_           = this->get_parameter("publish_tf").as_bool();

    meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / static_cast<double>(ticks_per_rev_);

    // Subscriptions
    motor_status_sub_ = this->create_subscription<polebot_interfaces::msg::MotorStatus>(
      "motor_status", 10,
      std::bind(&OdometryPublisher::motorStatusCallback, this, std::placeholders::_1)
    );

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "OdometryPublisher initialized");
  }

private:
  void motorStatusCallback(const polebot_interfaces::msg::MotorStatus::SharedPtr msg)
  {
    if (first_reading_) {
      last_left_enc_  = msg->left_encoder;
      last_right_enc_ = msg->right_encoder;
      last_time_ = msg->header.stamp;
      first_reading_ = false;
      return;
    }

    // Compute encoder deltas
    int32_t delta_left  = msg->left_encoder  - last_left_enc_;
    int32_t delta_right = msg->right_encoder - last_right_enc_;
    last_left_enc_  = msg->left_encoder;
    last_right_enc_ = msg->right_encoder;

    // Convert to distance
    double dist_left  = delta_left  * meters_per_tick_;
    double dist_right = delta_right * meters_per_tick_;

    // Differential drive odometry
    double dist     = (dist_left + dist_right) / 2.0;
    double d_theta  = (dist_right - dist_left) / wheel_separation_;

    x_     += dist * std::cos(theta_ + d_theta / 2.0);
    y_     += dist * std::sin(theta_ + d_theta / 2.0);
    theta_ += d_theta;

    // Normalize theta to [-pi, pi]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // Compute velocities
    auto dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_time_)).seconds();
    last_time_ = msg->header.stamp;

    double vx    = (dt > 0.0) ? (dist / dt) : 0.0;
    double vtheta = (dt > 0.0) ? (d_theta / dt) : 0.0;

    // Quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // Publish odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;

    odom.pose.pose.position.x    = x_;
    odom.pose.pose.position.y    = y_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x  = vx;
    odom.twist.twist.angular.z = vtheta;

    odom_pub_->publish(odom);

    // Publish TF
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // ROS interfaces
  rclcpp::Subscription<polebot_interfaces::msg::MotorStatus>::SharedPtr motor_status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Odometry state
  double x_, y_, theta_;
  int32_t last_left_enc_, last_right_enc_;
  builtin_interfaces::msg::Time last_time_;
  bool first_reading_;

  // Parameters
  double   wheel_radius_;
  double   wheel_separation_;
  int64_t  ticks_per_rev_;
  double   meters_per_tick_;
  std::string odom_frame_;
  std::string base_frame_;
  bool     publish_tf_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
