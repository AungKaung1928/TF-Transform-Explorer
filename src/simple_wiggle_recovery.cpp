#include "tf_explorer/simple_wiggle_recovery.hpp"

namespace tf_explorer
{

SimpleWiggleRecovery::SimpleWiggleRecovery()
: Node("simple_wiggle_recovery")
{
  // Declare parameters
  this->declare_parameter("wiggle_speed", 0.5);
  this->declare_parameter("wiggle_duration", 0.5);
  this->declare_parameter("wiggle_count", 3);

  wiggle_speed_ = this->get_parameter("wiggle_speed").as_double();
  wiggle_duration_ = this->get_parameter("wiggle_duration").as_double();
  wiggle_count_ = this->get_parameter("wiggle_count").as_int();

  // Create velocity publisher
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer for wiggle control (10 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SimpleWiggleRecovery::timer_callback, this));

  // Start wiggling immediately
  is_wiggling_ = true;
  wiggle_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "Wiggle Recovery started: speed=%.2f, duration=%.2f, count=%d",
    wiggle_speed_, wiggle_duration_, wiggle_count_);
}

void SimpleWiggleRecovery::timer_callback()
{
  if (!is_wiggling_) {
    return;
  }

  // Check if all wiggles done
  if (current_wiggle_ >= wiggle_count_ * 2) {
    RCLCPP_INFO(this->get_logger(), "Wiggle Recovery complete!");

    // Stop robot
    geometry_msgs::msg::Twist stop_cmd;
    vel_pub_->publish(stop_cmd);

    is_wiggling_ = false;
    return;
  }

  // Check if current wiggle direction is done
  double elapsed = (this->now() - wiggle_start_time_).seconds();
  if (elapsed >= wiggle_duration_) {
    wiggle_left_ = !wiggle_left_;
    current_wiggle_++;
    wiggle_start_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "Switching direction, wiggle #%d", current_wiggle_);
  }

  // Publish velocity
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = wiggle_left_ ? wiggle_speed_ : -wiggle_speed_;
  vel_pub_->publish(cmd);
}

}  // namespace tf_explorer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tf_explorer::SimpleWiggleRecovery>();
  
  // Spin until wiggle is done (or use spin_some in a loop)
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}