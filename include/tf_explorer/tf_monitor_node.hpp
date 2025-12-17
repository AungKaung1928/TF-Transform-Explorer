#ifndef TF_EXPLORER__TF_MONITOR_NODE_HPP_
#define TF_EXPLORER__TF_MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace tf_explorer
{

class TFMonitorNode : public rclcpp::Node
{
public:
  TFMonitorNode();

private:
  void timer_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void log_transform(const std::string & from, const std::string & to);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  
  geometry_msgs::msg::PoseArray pose_history_;
  size_t max_poses_{100};
};

}  // namespace tf_explorer

#endif  // TF_EXPLORER__TF_MONITOR_NODE_HPP_