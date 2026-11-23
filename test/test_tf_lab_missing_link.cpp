// TF debugging lab: a broken chain map -> odom -> base_link.
// Broken setup: map -> odom is broadcast, odom -> base_link is not (odometry node down or
// publishing under another frame name). map -> odom stays healthy, map -> base_link is MISSING.
// Fix: broadcast odom -> base_link; map -> base_link turns healthy.
// Same /tf_diagnostics report the TFAnomalyDetector node publishes at run time.
#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "tf_explorer/msg/tf_diagnostics.hpp"
#include "tf_explorer/tf_anomaly_detector.hpp"

using tf_explorer::msg::TFDiagnostics;
using namespace std::chrono_literals;

class TFLabMissingLink : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    detector_ = std::make_shared<tf_explorer::TFAnomalyDetector>();
    probe_ = std::make_shared<rclcpp::Node>("tf_lab_probe");
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(probe_);
    sub_ = probe_->create_subscription<TFDiagnostics>(
      "/tf_diagnostics", 30,
      [this](TFDiagnostics::SharedPtr m) {last_[m->parent_frame + "->" + m->child_frame] = m;});
    exec_.add_node(detector_);
    exec_.add_node(probe_);
  }

  void send(const std::string & parent, const std::string & child)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = probe_->now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(t);
  }

  // Broadcasts the chain, spins 3.5 s (about 7 detector cycles at 2 Hz): reports are current.
  void run_chain(bool with_odom_to_base_link)
  {
    const auto end = std::chrono::steady_clock::now() + 3500ms;
    while (std::chrono::steady_clock::now() < end) {
      send("map", "odom");
      if (with_odom_to_base_link) {
        send("odom", "base_link");
      }
      exec_.spin_once(20ms);
    }
  }

  std::shared_ptr<tf_explorer::TFAnomalyDetector> detector_;
  rclcpp::Node::SharedPtr probe_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<TFDiagnostics>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::map<std::string, TFDiagnostics::SharedPtr> last_;
};

TEST_F(TFLabMissingLink, BrokenChainReportsMapToBaseLinkMissing)
{
  run_chain(false);
  ASSERT_TRUE(last_["map->odom"]);
  EXPECT_TRUE(last_["map->odom"]->is_healthy) << last_["map->odom"]->status_message;
  ASSERT_TRUE(last_["map->base_link"]);
  EXPECT_FALSE(last_["map->base_link"]->is_healthy);
  EXPECT_EQ(last_["map->base_link"]->status_message.rfind("MISSING", 0), 0u)
    << last_["map->base_link"]->status_message;
}

TEST_F(TFLabMissingLink, FixedChainReportsMapToBaseLinkHealthy)
{
  run_chain(true);
  ASSERT_TRUE(last_["map->base_link"]);
  EXPECT_TRUE(last_["map->base_link"]->is_healthy) << last_["map->base_link"]->status_message;
  ASSERT_TRUE(last_["odom->base_link"]);
  EXPECT_TRUE(last_["odom->base_link"]->is_healthy) << last_["odom->base_link"]->status_message;
}
