// Preserve Smac2D search/smoothing, then connect its grid-center endpoint to
// the requested sub-cell goal. Used only by fixed-distance accuracy trials.
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace agt_field_test_bringup
{
class ExactGoalPlanner : public nav2_smac_planner::SmacPlanner2D
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    costmap_ros_ = costmap_ros;
    SmacPlanner2D::configure(parent, name, tf, costmap_ros);
  }

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    auto path = SmacPlanner2D::createPlan(start, goal);
    if (path.poses.empty()) {return path;}
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto last = path.poses.back().pose;
    const double dx = goal.pose.position.x - last.position.x;
    const double dy = goal.pose.position.y - last.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance > _tolerance + std::sqrt(2.0) * map->getResolution()) {
      throw nav2_core::PlannerException("Exact goal is too far from the Smac endpoint");
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    const auto footprint = costmap_ros_->getRobotFootprint();
    const double yaw = tf2::getYaw(goal.pose.orientation);
    const int steps = std::max(1, static_cast<int>(std::ceil(
      distance / std::min(0.005, map->getResolution() / 4.0))));
    for (int i = 0; i <= steps; ++i) {
      const double ratio = static_cast<double>(i) / steps;
      const double x = last.position.x + ratio * dx;
      const double y = last.position.y + ratio * dy;
      unsigned int mx, my;
      if (!map->worldToMap(x, y, mx, my) ||
        map->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        checker.footprintCostAtPose(x, y, yaw, footprint) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        throw nav2_core::PlannerException("Exact goal connection is occupied or unknown");
      }
    }
    auto exact = goal;
    exact.header = path.header;
    if (distance > 1e-6) {path.poses.push_back(exact);}
    else {path.poses.back() = exact;}
    return path;
  }
private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
}  // namespace agt_field_test_bringup
PLUGINLIB_EXPORT_CLASS(agt_field_test_bringup::ExactGoalPlanner, nav2_core::GlobalPlanner)
