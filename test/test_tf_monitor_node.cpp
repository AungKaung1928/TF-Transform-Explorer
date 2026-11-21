// TFMonitorNode's pose trail: /pose_history must carry the frame the odometry poses are in.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf_explorer/tf_monitor_node.hpp"

using namespace std::chrono_literals;

TEST(TFMonitorNode, PoseHistoryKeepsTheOdometryFrame)
{
  rclcpp::init(0, nullptr);
  {
    auto monitor = std::make_shared<tf_explorer::TFMonitorNode>();
    auto helper = std::make_shared<rclcpp::Node>("pose_history_probe");
    auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    geometry_msgs::msg::PoseArray::SharedPtr last;
    auto sub = helper->create_subscription<geometry_msgs::msg::PoseArray>(
      "/pose_history", 10,
      [&last](geometry_msgs::msg::PoseArray::SharedPtr msg) {last = msg;});

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(monitor);
    exec.add_node(helper);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = 1.5;
    odom.pose.pose.orientation.w = 1.0;

    const auto end = std::chrono::steady_clock::now() + 10s;
    while (!last && std::chrono::steady_clock::now() < end) {
      odom_pub->publish(odom);   // repeated: the first messages can leave before discovery
      exec.spin_once(50ms);
    }

    ASSERT_TRUE(last);
    EXPECT_EQ(last->header.frame_id, "odom");
    ASSERT_FALSE(last->poses.empty());
    EXPECT_DOUBLE_EQ(last->poses.back().position.x, 1.5);
  }
  rclcpp::shutdown();
}
