#ifndef TF_EXPLORER__FRAME_BROADCASTER_NODE_HPP_
#define TF_EXPLORER__FRAME_BROADCASTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tf_explorer
{

class FrameBroadcasterNode : public rclcpp::Node
{
public:
  FrameBroadcasterNode();

private:
  void broadcast_static_frames();
  void timer_callback();

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double angle_{0.0};
};

}  // namespace tf_explorer

#endif  // TF_EXPLORER__FRAME_BROADCASTER_NODE_HPP_