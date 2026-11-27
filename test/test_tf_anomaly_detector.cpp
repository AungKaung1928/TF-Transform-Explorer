// TFAnomalyDetector on real /tf traffic: what it reports on /tf_diagnostics for a missing,
// a fresh, a jumping and a stale map -> odom transform (defaults: max age 1.0 s, max jump 0.5 m).
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "tf_explorer/msg/tf_diagnostics.hpp"
#include "tf_explorer/tf_anomaly_detector.hpp"

using tf_explorer::msg::TFDiagnostics;
using namespace std::chrono_literals;

class TFAnomalyDetectorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    detector_ = std::make_shared<tf_explorer::TFAnomalyDetector>();
    probe_ = std::make_shared<rclcpp::Node>("tf_diagnostics_probe");
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(probe_);
    sub_ = probe_->create_subscription<TFDiagnostics>(
      "/tf_diagnostics", 30,
      [this](TFDiagnostics::SharedPtr msg) {
        if (msg->parent_frame == "map" && msg->child_frame == "odom" && wanted_(*msg)) {
          found_ = msg;
        }
      });
    exec_.add_node(detector_);
    exec_.add_node(probe_);
  }

  void send_map_to_odom(double x, double age_s)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = probe_->now() - rclcpp::Duration::from_seconds(age_s);
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = x;
    t.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(t);
  }

  // Spins up to 10 s, calling `publish` each round, until a map -> odom report matches `wanted`.
  TFDiagnostics::SharedPtr wait_for(
    std::function<bool(const TFDiagnostics &)> wanted, std::function<void()> publish = []() {})
  {
    wanted_ = wanted;
    found_.reset();
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (!found_ && std::chrono::steady_clock::now() < end) {
      publish();
      exec_.spin_once(20ms);
    }
    return found_;
  }

  std::shared_ptr<tf_explorer::TFAnomalyDetector> detector_;
  rclcpp::Node::SharedPtr probe_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<TFDiagnostics>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::function<bool(const TFDiagnostics &)> wanted_ = [](const TFDiagnostics &) {return false;};
  TFDiagnostics::SharedPtr found_;
};

TEST_F(TFAnomalyDetectorTest, MissingTransformIsUnhealthy)
{
  auto d = wait_for([](const TFDiagnostics &) {return true;});
  ASSERT_TRUE(d);
  EXPECT_FALSE(d->is_healthy);
  EXPECT_EQ(d->status_message.rfind("MISSING", 0), 0u) << d->status_message;
  EXPECT_DOUBLE_EQ(d->transform_age_seconds, -1.0);
}

TEST_F(TFAnomalyDetectorTest, FreshTransformIsHealthyThenAJumpIsReported)
{
  auto ok = wait_for(
    [](const TFDiagnostics & m) {return m.status_message == "OK";},
    [this]() {send_map_to_odom(0.0, 0.0);});
  ASSERT_TRUE(ok);
  EXPECT_TRUE(ok->is_healthy);
  EXPECT_FALSE(ok->jump_detected);
  EXPECT_LT(ok->transform_age_seconds, 1.0);

  auto jump = wait_for(
    [](const TFDiagnostics & m) {return m.jump_detected;},
    [this]() {send_map_to_odom(1.0, 0.0);});
  ASSERT_TRUE(jump);
  EXPECT_FALSE(jump->is_healthy);
  EXPECT_NEAR(jump->position_jump_meters, 1.0, 1e-9);
  EXPECT_EQ(jump->status_message.rfind("JUMP", 0), 0u) << jump->status_message;
}

TEST_F(TFAnomalyDetectorTest, OldTransformIsStale)
{
  auto stale = wait_for(
    [](const TFDiagnostics & m) {return m.transform_age_seconds > 0.0;},
    [this]() {send_map_to_odom(0.0, 5.0);});
  ASSERT_TRUE(stale);
  EXPECT_FALSE(stale->is_healthy);
  EXPECT_GT(stale->transform_age_seconds, 1.0);
  EXPECT_EQ(stale->status_message.rfind("STALE", 0), 0u) << stale->status_message;
}

TEST_F(TFAnomalyDetectorTest, NonPositiveThresholdIsRefused)
{
  for (const char * name : {"max_transform_age", "max_position_jump"}) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter(name, 0.0)});
    EXPECT_THROW(tf_explorer::TFAnomalyDetector{opts}, std::invalid_argument) << name;
  }
}
