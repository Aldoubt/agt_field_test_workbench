// SPDX-License-Identifier: Apache-2.0
#ifndef AGT_FIELD_TEST_BRINGUP__UNKNOWN_PRESERVING_OBSTACLE_LAYER_HPP_
#define AGT_FIELD_TEST_BRINGUP__UNKNOWN_PRESERVING_OBSTACLE_LAYER_HPP_

#include <nav2_costmap_2d/obstacle_layer.hpp>

namespace agt_field_test_bringup
{

// Used after the static layer in Bunker's non-rolling global costmap. Humble's
// max merge otherwise replaces unknown master cells with raytraced free space.
// Observation buffering and raytracing stay native. Only combination_method=1
// is supported, retaining the static layer's known occupied cells as well.
class UnknownPreservingObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    combination_validation_handle_;
};

}  // namespace agt_field_test_bringup

#endif  // AGT_FIELD_TEST_BRINGUP__UNKNOWN_PRESERVING_OBSTACLE_LAYER_HPP_
