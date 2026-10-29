#pragma once

#include <memory>
#include <string>
#include <utility>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace agt_field_test_bringup
{
// A private lookup view for Nav2's 2-D controller, never a TF broadcaster.
// Sensor/costmap/localization users retain the original, full 3-D buffer.
class PlanarControlBuffer : public tf2_ros::Buffer
{
public:
  PlanarControlBuffer(rclcpp::Clock::SharedPtr clock,
    std::shared_ptr<tf2_ros::Buffer> source, std::string base_frame)
  : tf2_ros::Buffer(clock), source_(std::move(source)), base_frame_(std::move(base_frame)) {}

  using tf2_ros::Buffer::lookupTransform;
  using tf2_ros::Buffer::canTransform;

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target, const std::string & source,
    const tf2::TimePoint & time, const tf2::Duration timeout) const override
  {
    return forControl(source_->lookupTransform(target, source, time, timeout));
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target, const tf2::TimePoint & target_time,
    const std::string & source, const tf2::TimePoint & source_time,
    const std::string & fixed, const tf2::Duration timeout) const override
  {
    return forControl(source_->lookupTransform(
      target, target_time, source, source_time, fixed, timeout));
  }

  bool canTransform(const std::string & target, const std::string & source,
    const tf2::TimePoint & time, const tf2::Duration timeout,
    std::string * error = nullptr) const override
  {
    return source_->canTransform(target, source, time, timeout, error);
  }

  bool canTransform(const std::string & target, const tf2::TimePoint & target_time,
    const std::string & source, const tf2::TimePoint & source_time,
    const std::string & fixed, const tf2::Duration timeout,
    std::string * error = nullptr) const override
  {
    return source_->canTransform(target, target_time, source, source_time, fixed, timeout, error);
  }

private:
  geometry_msgs::msg::TransformStamped forControl(
    geometry_msgs::msg::TransformStamped transform) const
  {
    if (transform.header.frame_id != base_frame_) {return transform;}
    tf2::Transform base_from_plan;
    tf2::fromMsg(transform.transform, base_from_plan);
    auto plan_from_base = base_from_plan.inverse();
    // Keep the base origin in the plan frame. Remove roll/pitch *before*
    // inversion: zeroing Euler angles after inversion leaves height-induced
    // errors in the inverse translation (the recorded 4 cm endpoint shift).
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, tf2::getYaw(plan_from_base.getRotation()));
    plan_from_base.setRotation(yaw_only);
    transform.transform = tf2::toMsg(plan_from_base.inverse());
    return transform;
  }

  std::shared_ptr<tf2_ros::Buffer> source_;
  std::string base_frame_;
};
}  // namespace agt_field_test_bringup
