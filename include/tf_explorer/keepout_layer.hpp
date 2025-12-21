#ifndef TF_EXPLORER__KEEPOUT_LAYER_HPP_
#define TF_EXPLORER__KEEPOUT_LAYER_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tf_explorer
{

/**
 * Simple Keepout Zone Costmap Layer
 * 
 * Marks a rectangular area as completely blocked (LETHAL_OBSTACLE).
 * Demonstrates how to write a CUSTOM COSTMAP LAYER PLUGIN.
 */
class KeepoutLayer : public nav2_costmap_2d::Layer
{
public:
  KeepoutLayer() = default;
  virtual ~KeepoutLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override { current_ = false; }

  bool isClearable() override { return false; }

private:
  double zone_min_x_{1.0};
  double zone_max_x_{2.0};
  double zone_min_y_{1.0};
  double zone_max_y_{2.0};
  bool enabled_{true};
};

}  // namespace tf_explorer

#endif  // TF_EXPLORER__KEEPOUT_LAYER_HPP_