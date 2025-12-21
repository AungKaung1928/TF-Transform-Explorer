#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class PatrolNode : public rclcpp::Node
{
public:
  PatrolNode() : Node("patrol_node")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    RCLCPP_INFO(get_logger(), "Patrol node started. Waiting 15 seconds for Nav2...");
    
    // Wait 15 seconds for Nav2 to fully activate
    startup_timer_ = create_wall_timer(
      std::chrono::seconds(15),
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
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available!");
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

    RCLCPP_INFO(get_logger(), "Going to (%.2f, %.2f)", 
      goal.pose.pose.position.x, goal.pose.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    options.result_callback = [this](const auto & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Goal reached!");
      } else {
        RCLCPP_WARN(get_logger(), "Goal failed, trying another...");
      }
      // Send next goal after 2 seconds
      goal_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          goal_timer_->cancel();
          send_next_goal();
        });
    };

    client_->async_send_goal(goal, options);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}