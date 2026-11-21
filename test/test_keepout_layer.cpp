// KeepoutLayer::updateCosts on a bare Costmap2D (no node needed: the layer's defaults are
// zone x, y in [1.0, 2.0] m, enabled).
#include <gtest/gtest.h>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "tf_explorer/keepout_layer.hpp"

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

// A zone that only partly overlaps the costmap (e.g. a 3x3 m rolling local costmap) must
// still mark the overlapping cells. Map covers [0, 2.0) m, so the zone's max corner (2.0, 2.0)
// is just off the map.
TEST(KeepoutLayer, ZonePartlyOffMapIsStillDrawn)
{
  Costmap2D grid(20, 20, 0.1, 0.0, 0.0, FREE_SPACE);
  tf_explorer::KeepoutLayer layer;
  layer.updateCosts(grid, 0, 0, 20, 20);
  EXPECT_EQ(grid.getCost(15, 15), LETHAL_OBSTACLE);  // (1.55, 1.55) m, inside the zone
  EXPECT_EQ(grid.getCost(5, 5), FREE_SPACE);         // (0.55, 0.55) m, outside
}

// Nav2 passes the update window with exclusive upper bounds: cells at i == max_i or
// j == max_j belong to the next window and must not be written.
TEST(KeepoutLayer, StaysInsideTheUpdateWindow)
{
  Costmap2D grid(40, 40, 0.1, 0.0, 0.0, FREE_SPACE);  // [0, 4.0) m, zone fully inside
  tf_explorer::KeepoutLayer layer;
  layer.updateCosts(grid, 0, 0, 15, 15);
  EXPECT_EQ(grid.getCost(14, 14), LETHAL_OBSTACLE);
  EXPECT_EQ(grid.getCost(15, 14), FREE_SPACE);
  EXPECT_EQ(grid.getCost(14, 15), FREE_SPACE);
}
