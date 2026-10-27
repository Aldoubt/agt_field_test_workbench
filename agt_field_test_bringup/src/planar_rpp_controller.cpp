#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <nav2_core/exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "agt_field_test_bringup/planar_control_buffer.hpp"

namespace agt_field_test_bringup
{
class PlanarRegulatedPurePursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    const auto node = parent.lock();
    if (!node) {throw nav2_core::PlannerException("Planar RPP parent expired");}
    auto control_view = std::make_shared<PlanarControlBuffer>(
      node->get_clock(), std::move(tf), costmap_ros->getBaseFrameID());
    RegulatedPurePursuitController::configure(parent, name, control_view, costmap_ros);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * checker) override
  {
    if (global_plan_.poses.empty()) {throw nav2_core::PlannerException("Received empty plan");}
    // Humble's per-point transform ignores a false transformPose() result.
    // Fail before invoking it if the real, timestamped path->base TF is absent.
    const auto stamp = tf2::TimePoint(std::chrono::seconds(pose.header.stamp.sec) +
      std::chrono::nanoseconds(pose.header.stamp.nanosec));
    try {
      (void)tf_->lookupTransform(costmap_ros_->getBaseFrameID(),
        global_plan_.header.frame_id, stamp, transform_tolerance_);
    } catch (const tf2::TransformException & error) {
      throw nav2_core::PlannerException(std::string("Planar RPP path TF unavailable: ") + error.what());
    }
    // The native implementation still owns path pruning, speed/approach
    // regulation, reversing, heading control and footprint collision checking.
    return RegulatedPurePursuitController::computeVelocityCommands(pose, speed, checker);
  }
};
}  // namespace agt_field_test_bringup
PLUGINLIB_EXPORT_CLASS(agt_field_test_bringup::PlanarRegulatedPurePursuitController,
  nav2_core::Controller)
