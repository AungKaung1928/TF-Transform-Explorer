#include "tf_explorer/keepout_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/costmap_math.hpp>

namespace tf_explorer
{

void KeepoutLayer::onInitialize()
{
  RCLCPP_INFO(logger_, "KeepoutLayer: Initializing...");

  // Get parameters with defaults
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("zone_min_x", rclcpp::ParameterValue(1.0));
  declareParameter("zone_max_x", rclcpp::ParameterValue(2.0));
  declareParameter("zone_min_y", rclcpp::ParameterValue(1.0));
  declareParameter("zone_max_y", rclcpp::ParameterValue(2.0));

  auto node = node_.lock();
  if (node) {
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".zone_min_x", zone_min_x_);
    node->get_parameter(name_ + ".zone_max_x", zone_max_x_);
    node->get_parameter(name_ + ".zone_min_y", zone_min_y_);
    node->get_parameter(name_ + ".zone_max_y", zone_max_y_);
  }

  RCLCPP_INFO(logger_,
    "KeepoutLayer: zone=[%.1f,%.1f]->[%.1f,%.1f], enabled=%d",
    zone_min_x_, zone_min_y_, zone_max_x_, zone_max_y_, enabled_);

  current_ = true;
}

void KeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  *min_x = std::min(*min_x, zone_min_x_);
  *min_y = std::min(*min_y, zone_min_y_);
  *max_x = std::max(*max_x, zone_max_x_);
  *max_y = std::max(*max_y, zone_max_y_);
}

void KeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned int zone_min_i, zone_min_j, zone_max_i, zone_max_j;

  if (!master_grid.worldToMap(zone_min_x_, zone_min_y_, zone_min_i, zone_min_j)) {
    return;
  }
  if (!master_grid.worldToMap(zone_max_x_, zone_max_y_, zone_max_i, zone_max_j)) {
    return;
  }

  // Clamp to update bounds
  zone_min_i = std::max(zone_min_i, static_cast<unsigned int>(min_i));
  zone_min_j = std::max(zone_min_j, static_cast<unsigned int>(min_j));
  zone_max_i = std::min(zone_max_i, static_cast<unsigned int>(max_i));
  zone_max_j = std::min(zone_max_j, static_cast<unsigned int>(max_j));

  // Mark keepout zone as LETHAL
  for (unsigned int j = zone_min_j; j <= zone_max_j; j++) {
    for (unsigned int i = zone_min_i; i <= zone_max_i; i++) {
      master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

}  // namespace tf_explorer

PLUGINLIB_EXPORT_CLASS(tf_explorer::KeepoutLayer, nav2_costmap_2d::Layer)