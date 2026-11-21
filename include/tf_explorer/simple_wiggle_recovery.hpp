#ifndef TF_EXPLORER__SIMPLE_WIGGLE_RECOVERY_HPP_
#define TF_EXPLORER__SIMPLE_WIGGLE_RECOVERY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace tf_explorer
{

/**
 * Simple "Wiggle" Recovery Node
 * 
 * Standalone node that wiggles the robot left-right.
 * Starts wiggling as soon as it is run, then stops the robot.
 * 
 * This is a SIMPLIFIED version - demonstrates the concept
 * without complex Nav2 plugin inheritance.
 */
class SimpleWiggleRecovery : public rclcpp::Node
{
public:
  SimpleWiggleRecovery();

private:
  void timer_callback();

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Wiggle parameters
  double wiggle_speed_{0.5};
  double wiggle_duration_{0.5};
  int wiggle_count_{3};

  // State
  int current_wiggle_{0};
  bool wiggle_left_{true};
  rclcpp::Time wiggle_start_time_;
  bool is_wiggling_{false};
};

}  // namespace tf_explorer

#endif  // TF_EXPLORER__SIMPLE_WIGGLE_RECOVERY_HPP_