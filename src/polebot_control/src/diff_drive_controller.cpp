/**
 * @file diff_drive_controller.cpp
 * @brief Differential Drive Controller for AMR-POLEBOT
 *
 * Converts cmd_vel (Twist) to left/right wheel velocity commands
 * and publishes motor set-points via polebot_interfaces/SetSpeed service.
 *
 * AMR-POLEBOT | Politeknik Manufaktur Bandung
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "polebot_interfaces/msg/motor_status.hpp"

using namespace std::chrono_literals;

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController()
  : Node("diff_drive_controller")
  {
    // Parameters
    this->declare_parameter("wheel_radius",    0.065);
    this->declare_parameter("wheel_separation", 0.35);
    this->declare_parameter("max_linear_vel",  0.5);
    this->declare_parameter("max_angular_vel", 2.0);
    this->declare_parameter("cmd_vel_timeout", 0.5);

    wheel_radius_    = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    max_linear_vel_  = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

    // Subscriptions
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1)
    );

    // Publishers
    motor_cmd_pub_ = this->create_publisher<polebot_interfaces::msg::MotorStatus>(
      "motor_commands", 10
    );

    // Timeout watchdog
    watchdog_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&DiffDriveController::watchdogCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "DiffDriveController initialized");
    RCLCPP_INFO(this->get_logger(), "  Wheel radius:    %.3f m", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "  Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "  Max linear vel:   %.2f m/s", max_linear_vel_);
    RCLCPP_INFO(this->get_logger(), "  Max angular vel:  %.2f rad/s", max_angular_vel_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    // Clamp velocities
    double linear  = std::clamp(msg->linear.x, -max_linear_vel_,  max_linear_vel_);
    double angular = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

    // Differential drive kinematics
    // v_left  = (linear - angular * wheel_separation / 2) / wheel_radius
    // v_right = (linear + angular * wheel_separation / 2) / wheel_radius
    double v_left  = (linear - angular * wheel_separation_ / 2.0) / wheel_radius_;
    double v_right = (linear + angular * wheel_separation_ / 2.0) / wheel_radius_;

    publishMotorCommand(v_left, v_right);
  }

  void watchdogCallback()
  {
    double elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed > cmd_vel_timeout_) {
      // Stop motors if no command received
      publishMotorCommand(0.0, 0.0);
    }
  }

  void publishMotorCommand(double left_vel, double right_vel)
  {
    auto msg = polebot_interfaces::msg::MotorStatus();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.left_velocity  = left_vel;
    msg.right_velocity = right_vel;

    motor_cmd_pub_->publish(msg);
  }

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<polebot_interfaces::msg::MotorStatus>::SharedPtr motor_cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Parameters
  double wheel_radius_;
  double wheel_separation_;
  double max_linear_vel_;
  double max_angular_vel_;
  double cmd_vel_timeout_;

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
