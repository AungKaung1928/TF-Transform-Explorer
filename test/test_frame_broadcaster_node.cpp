// FrameBroadcasterNode seen through a tf2 buffer: the static sensor mount and the rotating sensor.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tf_explorer/frame_broadcaster_node.hpp"

using namespace std::chrono_literals;

TEST(FrameBroadcasterNode, StaticMountAndRotatingSensor)
{
  rclcpp::init(0, nullptr);
  {
    auto broadcaster = std::make_shared<tf_explorer::FrameBroadcasterNode>();
    tf2_ros::Buffer buffer(broadcaster->get_clock());
    tf2_ros::TransformListener listener(buffer);   // spins its own thread
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(broadcaster);

    // spin until both transforms are known and the sensor has turned since the first look
    bool have_first = false;
    double first_z = 0.0;
    geometry_msgs::msg::TransformStamped mount, sensor;
    bool turned = false;
    const auto end = std::chrono::steady_clock::now() + 15s;
    while (!turned && std::chrono::steady_clock::now() < end) {
      exec.spin_once(20ms);
      try {
        mount = buffer.lookupTransform("base_link", "sensor_mount", tf2::TimePointZero);
        sensor = buffer.lookupTransform("sensor_mount", "rotating_sensor", tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }
      if (!have_first) {
        have_first = true;
        first_z = sensor.transform.rotation.z;
      } else {
        turned = std::abs(sensor.transform.rotation.z - first_z) > 1e-3;
      }
    }

    ASSERT_TRUE(have_first);
    EXPECT_DOUBLE_EQ(mount.transform.translation.x, 0.1);
    EXPECT_DOUBLE_EQ(mount.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(mount.transform.translation.z, 0.2);
    EXPECT_DOUBLE_EQ(mount.transform.rotation.w, 1.0);

    EXPECT_TRUE(turned);
    EXPECT_DOUBLE_EQ(sensor.transform.translation.z, 0.05);
    const auto & q = sensor.transform.rotation;
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_NEAR(q.z * q.z + q.w * q.w, 1.0, 1e-12);   // a pure yaw, unit length
  }
  rclcpp::shutdown();
}
