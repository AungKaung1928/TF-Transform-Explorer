#include "tf_explorer/frame_broadcaster_node.hpp"
#include <cmath>

namespace tf_explorer
{

FrameBroadcasterNode::FrameBroadcasterNode() : Node("frame_broadcaster_node")
{
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  broadcast_static_frames();

  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&FrameBroadcasterNode::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Frame Broadcaster Node started");
}

void FrameBroadcasterNode::broadcast_static_frames()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "sensor_mount";
  t.transform.translation.x = 0.1;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.2;
  t.transform.rotation.w = 1.0;
  
  static_broadcaster_->sendTransform(t);
  RCLCPP_INFO(get_logger(), "Published static transform: base_link -> sensor_mount");
}

void FrameBroadcasterNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "sensor_mount";
  t.child_frame_id = "rotating_sensor";
  
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.05;
  
  angle_ += 0.05;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(angle_ / 2.0);
  t.transform.rotation.w = std::cos(angle_ / 2.0);
  
  dynamic_broadcaster_->sendTransform(t);
}

}  // namespace tf_explorer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_explorer::FrameBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}