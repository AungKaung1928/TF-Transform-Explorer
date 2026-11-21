#ifndef TF_EXPLORER__PATROL_NODE_HPP_
#define TF_EXPLORER__PATROL_NODE_HPP_

#include <chrono>
#include <random>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace tf_explorer
{

// Sends random NavigateToPose goals inside a 2 x 2 m area, one after the other.
class PatrolNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit PatrolNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("patrol_node", options)
  {
    startup_delay_ = declare_parameter("startup_delay", 15.0);   // seconds, for Nav2 to activate
    retry_delay_ = declare_parameter("retry_delay", 2.0);        // seconds between goals
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(get_logger(), "Patrol node started. Waiting %.1f seconds for Nav2...", startup_delay_);
    startup_timer_ = create_wall_timer(
      std::chrono::duration<double>(startup_delay_),
      [this]() {
        startup_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Starting patrol!");
        send_next_goal();
      });
  }

private:
  void send_next_goal()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available, retrying...");
      schedule_next_goal();
      return;
    }

    // Random goal within safe area
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(-1.0, 1.0);
    std::uniform_real_distribution<> dis_y(-1.0, 1.0);

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = dis_x(gen);
    goal.pose.pose.position.y = dis_y(gen);
    goal.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(
      get_logger(), "Going to (%.2f, %.2f)",
      goal.pose.pose.position.x, goal.pose.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = [this](const auto & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Goal reached!");
        } else {
          RCLCPP_WARN(get_logger(), "Goal failed, trying another...");
        }
        schedule_next_goal();
      };
    // A rejected goal never reaches result_callback: without this the patrol stops for good.
    options.goal_response_callback = [this](const auto & goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "Goal rejected, trying another...");
          schedule_next_goal();
        }
      };

    client_->async_send_goal(goal, options);
  }

  void schedule_next_goal()
  {
    goal_timer_ = create_wall_timer(
      std::chrono::duration<double>(retry_delay_),
      [this]() {
        goal_timer_->cancel();
        send_next_goal();
      });
  }

  double startup_delay_;
  double retry_delay_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
};

}  // namespace tf_explorer

#endif  // TF_EXPLORER__PATROL_NODE_HPP_
