// PatrolNode against a fake NavigateToPose server: the patrol must go on after a rejected
// goal, after a finished goal, and when the server only appears later.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tf_explorer/patrol_node.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

// Rejects the first `reject_first` goals, then accepts and succeeds every goal at once.
class FakeNav : public rclcpp::Node
{
public:
  explicit FakeNav(int reject_first)
  : Node("fake_nav"), reject_first_(reject_first)
  {
    server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
        return ++goals <= reject_first_ ?
               rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> handle) {
        workers_.emplace_back(
          [handle]() {handle->succeed(std::make_shared<NavigateToPose::Result>());});
      });
  }

  ~FakeNav() override
  {
    for (auto & worker : workers_) {
      worker.join();
    }
  }

  std::atomic<int> goals{0};

private:
  int reject_first_;
  std::vector<std::thread> workers_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

class PatrolNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<tf_explorer::PatrolNode> fast_patrol()
  {
    return std::make_shared<tf_explorer::PatrolNode>(
      rclcpp::NodeOptions().parameter_overrides(
        {{"startup_delay", 0.1}, {"retry_delay", 0.1}}));
  }

  template<typename Done>
  static void spin_until(rclcpp::Executor & exec, Done done, std::chrono::seconds limit)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (!done() && std::chrono::steady_clock::now() < end) {
      exec.spin_some(50ms);
    }
  }
};

TEST_F(PatrolNodeTest, GoesOnAfterRejectedAndFinishedGoals)
{
  auto nav = std::make_shared<FakeNav>(1);
  auto patrol = fast_patrol();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(nav);
  exec.add_node(patrol);

  // goal 1 is rejected, goal 2 exists only if a rejection is retried,
  // goal 3 only if a finished goal is followed by the next one
  spin_until(exec, [&]() {return nav->goals >= 3;}, 20s);
  EXPECT_GE(nav->goals, 3);
}

TEST_F(PatrolNodeTest, GoesOnWhenTheServerAppearsLate)
{
  auto patrol = fast_patrol();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(patrol);
  spin_until(exec, []() {return false;}, 7s);   // one 5 s server wait times out in here

  auto nav = std::make_shared<FakeNav>(0);
  exec.add_node(nav);
  spin_until(exec, [&]() {return nav->goals >= 1;}, 20s);
  EXPECT_GE(nav->goals, 1);
}
