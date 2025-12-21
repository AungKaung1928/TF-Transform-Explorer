#include "tf_explorer/tf_anomaly_detector.hpp"
#include <cmath>

namespace tf_explorer
{

TFAnomalyDetector::TFAnomalyDetector() : Node("tf_anomaly_detector")
{
  // Declare parameters
  this->declare_parameter("max_transform_age", 1.0);
  this->declare_parameter("max_position_jump", 0.5);
  
  max_transform_age_ = this->get_parameter("max_transform_age").as_double();
  max_position_jump_ = this->get_parameter("max_position_jump").as_double();
  
  // Initialize TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create publisher for our CUSTOM message
  diagnostics_pub_ = this->create_publisher<tf_explorer::msg::TFDiagnostics>(
    "/tf_diagnostics", 10);
  
  // Check transforms at 2 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TFAnomalyDetector::check_transforms, this));
  
  RCLCPP_INFO(this->get_logger(), 
    "TF Anomaly Detector started (max_age=%.2fs, max_jump=%.2fm)",
    max_transform_age_, max_position_jump_);
}

void TFAnomalyDetector::check_transforms()
{
  // Monitor critical navigation transforms
  check_single_transform("map", "odom");
  check_single_transform("odom", "base_link");
  check_single_transform("map", "base_link");
}

void TFAnomalyDetector::check_single_transform(
  const std::string & parent, 
  const std::string & child)
{
  tf_explorer::msg::TFDiagnostics diag;
  diag.header.stamp = this->now();
  diag.parent_frame = parent;
  diag.child_frame = child;
  diag.max_allowed_age_seconds = max_transform_age_;
  diag.max_allowed_jump_meters = max_position_jump_;
  
  std::string key = parent + "_" + child;
  
  try {
    // Lookup the transform
    auto transform = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
    
    // Check 1: Transform age (staleness)
    double age = (this->now() - transform.header.stamp).seconds();
    diag.transform_age_seconds = age;
    
    bool is_stale = (age > max_transform_age_);
    
    // Check 2: Position jump detection
    diag.jump_detected = false;
    diag.position_jump_meters = 0.0;
    
    if (previous_transforms_.count(key) > 0) {
      auto & prev = previous_transforms_[key];
      double dx = transform.transform.translation.x - prev.transform.translation.x;
      double dy = transform.transform.translation.y - prev.transform.translation.y;
      double dz = transform.transform.translation.z - prev.transform.translation.z;
      double jump = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      diag.position_jump_meters = jump;
      diag.jump_detected = (jump > max_position_jump_);
      
      if (diag.jump_detected) {
        RCLCPP_WARN(this->get_logger(), 
          "JUMP DETECTED: %s->%s moved %.3fm (threshold: %.2fm)",
          parent.c_str(), child.c_str(), jump, max_position_jump_);
      }
    }
    
    // Store for next comparison
    previous_transforms_[key] = transform;
    
    // Set overall health
    diag.is_healthy = !is_stale && !diag.jump_detected;
    
    if (is_stale) {
      diag.status_message = "STALE: Transform is " + std::to_string(age) + "s old";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "STALE: %s->%s is %.2fs old", parent.c_str(), child.c_str(), age);
    } else if (diag.jump_detected) {
      diag.status_message = "JUMP: Position jumped " + 
        std::to_string(diag.position_jump_meters) + "m";
    } else {
      diag.status_message = "OK";
    }
    
  } catch (const tf2::TransformException & ex) {
    // Transform not available
    diag.is_healthy = false;
    diag.status_message = std::string("MISSING: ") + ex.what();
    diag.transform_age_seconds = -1.0;
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "MISSING: %s->%s: %s", parent.c_str(), child.c_str(), ex.what());
  }
  
  diagnostics_pub_->publish(diag);
}

}  // namespace tf_explorer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_explorer::TFAnomalyDetector>());
  rclcpp::shutdown();
  return 0;
}