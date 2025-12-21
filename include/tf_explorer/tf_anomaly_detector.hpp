#ifndef TF_EXPLORER__TF_ANOMALY_DETECTOR_HPP_
#define TF_EXPLORER__TF_ANOMALY_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf_explorer/msg/tf_diagnostics.hpp"

namespace tf_explorer
{

/**
 * TF Anomaly Detector - Monitors TF tree for common problems:
 * 1. Stale transforms (too old)
 * 2. Position jumps (sudden large movements)
 * 3. Missing frames (transform lookup failures)
 * 
 * This demonstrates CUSTOM NODE + CUSTOM MESSAGE creation
 */
class TFAnomalyDetector : public rclcpp::Node
{
public:
  TFAnomalyDetector();

private:
  void check_transforms();
  void check_single_transform(const std::string & parent, const std::string & child);
  
  // TF infrastructure
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Timer for periodic checks
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Publisher for our CUSTOM message type
  rclcpp::Publisher<tf_explorer::msg::TFDiagnostics>::SharedPtr diagnostics_pub_;
  
  // Store previous transforms to detect jumps
  std::map<std::string, geometry_msgs::msg::TransformStamped> previous_transforms_;
  
  // Thresholds
  double max_transform_age_{1.0};      // seconds
  double max_position_jump_{0.5};       // meters (detect teleportation/localization jumps)
};

}  // namespace tf_explorer

#endif  // TF_EXPLORER__TF_ANOMALY_DETECTOR_HPP_