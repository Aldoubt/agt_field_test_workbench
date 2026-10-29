// SPDX-License-Identifier: BSD-3-Clause
// Adapted from nav2_costmap_2d's Humble ObstacleLayer::updateCosts and
// CostmapLayer merge. Copyright (c) 2008, 2013, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "agt_field_test_bringup/unknown_preserving_obstacle_layer.hpp"

#include <algorithm>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace agt_field_test_bringup
{

void UnknownPreservingObstacleLayer::onInitialize()
{
  nav2_costmap_2d::ObstacleLayer::onInitialize();
  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("UnknownPreservingObstacleLayer node has expired during initialization");
  }
  const std::string combination_parameter = name_ + ".combination_method";
  // Register after ObstacleLayer's callback: Humble executes parameter
  // callbacks in reverse registration order and stops at the first rejection.
  // Validate before its callback mutates combination_method_. Capture only the
  // name so no callback can outlive and dereference this layer instance.
  combination_validation_handle_ = node->add_on_set_parameters_callback(
    [combination_parameter](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == combination_parameter &&
          (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
          parameter.as_int() != 1))
        {
          result.successful = false;
          result.reason = combination_parameter + " must remain integer 1 (maximum)";
          break;
        }
      }
      return result;
    });
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> layer_lock(*getMutex());
  if (combination_method_ != 1) {
    throw std::invalid_argument(
            "UnknownPreservingObstacleLayer requires combination_method=1 (maximum)");
  }
}

void UnknownPreservingObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // LayeredCostmap already holds the recursive master mutex during updates.
  // Taking it here also makes direct calls safe and keeps both grids stable.
  // Keep the native master -> layer lock order; both mutexes are recursive.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> master_lock(
    *master_grid.getMutex());
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> layer_lock(*getMutex());

  if (!enabled_) {
    return;
  }
  if (combination_method_ != 1) {
    // Normal parameter writes are rejected before reaching the native
    // callback. Keep maximum semantics even if internal state is changed by
    // another caller; do not terminate the costmap update thread.
    RCLCPP_ERROR_ONCE(
      logger_, "UnknownPreservingObstacleLayer received unsupported combination_method; "
      "continuing with maximum merge to preserve static obstacles");
  }
  // Keep Humble's reset-current transition and footprint clearing. Only the
  // final merge differs from the native implementation.
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(max_i, static_cast<int>(std::min(
      master_grid.getSizeInCellsX(), getSizeInCellsX())));
  max_j = std::min(max_j, static_cast<int>(std::min(
      master_grid.getSizeInCellsY(), getSizeInCellsY())));
  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  // No full-map copy, persistent pointer, or unknown-cell index list. Scan only
  // the caller's clipped update window with O(1) auxiliary memory.
  const std::size_t master_width = master_grid.getSizeInCellsX();
  const std::size_t layer_width = getSizeInCellsX();
  auto * const master = master_grid.getCharMap();
  const auto * const layer = getCharMap();
  for (int j = min_j; j < max_j; ++j) {
    const std::size_t master_row = static_cast<std::size_t>(j) * master_width;
    const std::size_t layer_row = static_cast<std::size_t>(j) * layer_width;
    for (int i = min_i; i < max_i; ++i) {
      auto & master_cost = master[master_row + static_cast<std::size_t>(i)];
      const auto layer_cost = layer[layer_row + static_cast<std::size_t>(i)];
      if (master_cost == nav2_costmap_2d::NO_INFORMATION ||
        layer_cost == nav2_costmap_2d::NO_INFORMATION)
      {
        continue;
      }
      if (layer_cost > master_cost) {
        master_cost = layer_cost;
      }
    }
  }
}

}  // namespace agt_field_test_bringup

PLUGINLIB_EXPORT_CLASS(
  agt_field_test_bringup::UnknownPreservingObstacleLayer, nav2_costmap_2d::Layer)
