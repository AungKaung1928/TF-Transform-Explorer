// SimpleWiggleRecovery on /cmd_vel with its defaults (3 wiggles of 0.5 s per side at 0.5 rad/s):
// the turn direction alternates, the last turn is to the right, and the node ends with a stop.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf_explorer/simple_wiggle_recovery.hpp"

using namespace std::chrono_literals;

TEST(SimpleWiggleRecovery, AlternatesEndsOnTheRightTurnAndStops)
{
  rclcpp::init(0, nullptr);
  {
    auto probe = std::make_shared<rclcpp::Node>("cmd_vel_probe");
    std::vector<double> turns;
    auto sub = probe->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 50,
      [&turns](geometry_msgs::msg::Twist::SharedPtr msg) {turns.push_back(msg->angular.z);});
    auto wiggle = std::make_shared<tf_explorer::SimpleWiggleRecovery>();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(probe);
    exec.add_node(wiggle);

    // 6 half-wiggles of 0.5 s = 3 s; the stop is the first zero command
    const auto end = std::chrono::steady_clock::now() + 15s;
    while ((turns.empty() || turns.back() != 0.0) && std::chrono::steady_clock::now() < end) {
      exec.spin_once(20ms);
    }

    ASSERT_GE(turns.size(), 3u);
    EXPECT_EQ(turns.back(), 0.0);                       // ends with a stop
    EXPECT_EQ(turns[turns.size() - 2], -0.5);           // last turn command: right, the 6th half
    int halves = 1;
    for (size_t i = 1; i + 1 < turns.size(); ++i) {
      EXPECT_EQ(std::abs(turns[i]), 0.5);
      halves += turns[i] != turns[i - 1];
    }
    EXPECT_LE(halves, 6);                               // 6 when the first command was received

    // nothing more is sent after the stop
    const size_t count = turns.size();
    const auto quiet_until = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < quiet_until) {
      exec.spin_once(20ms);
    }
    EXPECT_EQ(turns.size(), count);
  }
  rclcpp::shutdown();
}
