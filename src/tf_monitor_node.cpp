#include "tf_explorer/tf_monitor_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf_explorer
{

TFMonitorNode::TFMonitorNode() : Node("tf_monitor_node")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&TFMonitorNode::timer_callback, this));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&TFMonitorNode::odom_callback, this, std::placeholders::_1));

  pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/points", 10,
    std::bind(&TFMonitorNode::pointcloud_callback, this, std::placeholders::_1));

  pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/pose_history", 10);
  
  pose_history_.header.frame_id = "map";
  
  RCLCPP_INFO(get_logger(), "TF Monitor Node started - tracking transforms");
}

void TFMonitorNode::timer_callback()
{
  RCLCPP_INFO(get_logger(), "=== TF Transform Report ===");
  log_transform("map", "odom");
  log_transform("odom", "base_link");
  log_transform("base_link", "base_scan");
  log_transform("map", "base_link");
  
  // Only check camera_link if it exists (Waffle model)
  if (tf_buffer_->canTransform("base_link", "camera_link", tf2::TimePointZero, 
                                tf2::durationFromSec(0.1))) {
    log_transform("base_link", "camera_link");
  }
}

void TFMonitorNode::log_transform(const std::string & from, const std::string & to)
{
  try {
    auto t = tf_buffer_->lookupTransform(from, to, tf2::TimePointZero);
    RCLCPP_INFO(get_logger(), 
      "[%s -> %s] XYZ(%.2f, %.2f, %.2f)",
      from.c_str(), to.c_str(),
      t.transform.translation.x,
      t.transform.translation.y,
      t.transform.translation.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "TF %s->%s: %s", from.c_str(), to.c_str(), ex.what());
  }
}

void TFMonitorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::Pose pose;
  pose.position = msg->pose.pose.position;
  pose.orientation = msg->pose.pose.orientation;
  
  pose_history_.poses.push_back(pose);
  if (pose_history_.poses.size() > max_poses_) {
    pose_history_.poses.erase(pose_history_.poses.begin());
  }
  
  pose_history_.header.stamp = this->now();
  pose_array_pub_->publish(pose_history_);
}

void TFMonitorNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), 
    "PointCloud2: %ux%u points, frame: %s",
    msg->width, msg->height, msg->header.frame_id.c_str());
}

}  // namespace tf_explorer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_explorer::TFMonitorNode>());
  rclcpp::shutdown();
  return 0;
}