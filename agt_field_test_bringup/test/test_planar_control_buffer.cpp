#include <gtest/gtest.h>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "agt_field_test_bringup/planar_control_buffer.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
geometry_msgs::msg::TransformStamped mount(double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map"; tf.child_frame_id = "base_link";
  tf.header.stamp.sec = 1;
  tf.transform.translation.x = 24.230084980826;
  tf.transform.translation.y = -9.677225279210834;
  tf.transform.translation.z = z;
  tf2::Quaternion q; q.setRPY(roll, pitch, yaw);
  tf.transform.rotation = tf2::toMsg(q);
  return tf;
}

auto goal(double z = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map"; p.header.stamp.sec = 1;
  p.pose.position.x = 24.31191492457211;
  p.pose.position.y = -9.686863408424415;
  p.pose.position.z = z; p.pose.orientation.w = 1.;
  return p;
}
}

TEST(PlanarControlBuffer, RecordedNearGoalKeepsCorrectSteeringSide)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto raw = std::make_shared<tf2_ros::Buffer>(clock);
  raw->setUsingDedicatedThread(true);
  ASSERT_TRUE(raw->setTransform(mount(-.8074249785, -.306177147*kPi/180.,
    2.848590164*kPi/180., -8.171474434*kPi/180.), "fixture", true));
  agt_field_test_bringup::PlanarControlBuffer control(clock, raw, "base_link");
  const auto old = raw->transform(goal(), "base_link");
  const auto fixed = control.transform(goal(), "base_link");
  EXPECT_LT(old.pose.position.y, 0.);  // Field failure: requests the wrong turn.
  EXPECT_NEAR(fixed.pose.position.x, .08236906049, 1e-9);
  EXPECT_NEAR(fixed.pose.position.y, .00209071800, 1e-9);
}

TEST(PlanarControlBuffer, PlanarXYIsIndependentOfHeightsRollAndPitch)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const double yaw = .37, dx = goal().pose.position.x - mount(0,0,0,0).transform.translation.x;
  const double dy = goal().pose.position.y - mount(0,0,0,0).transform.translation.y;
  for (double base_z : {-1.5, 0., 1.2}) {
    for (double goal_z : {-2., 0., 3.}) {
      auto raw = std::make_shared<tf2_ros::Buffer>(clock);
  raw->setUsingDedicatedThread(true);
      ASSERT_TRUE(raw->setTransform(mount(base_z, .12, -.25, yaw), "fixture", true));
      agt_field_test_bringup::PlanarControlBuffer control(clock, raw, "base_link");
      const auto p = control.transform(goal(goal_z), "base_link").pose.position;
      EXPECT_NEAR(p.x, std::cos(yaw)*dx + std::sin(yaw)*dy, 1e-12);
      EXPECT_NEAR(p.y, -std::sin(yaw)*dx + std::cos(yaw)*dy, 1e-12);
    }
  }
}

TEST(PlanarControlBuffer, GlobalPoseAndOriginalBufferRemainThreeDimensional)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto raw = std::make_shared<tf2_ros::Buffer>(clock);
  raw->setUsingDedicatedThread(true);
  const auto original = mount(-.8, .12, .2, .37);
  ASSERT_TRUE(raw->setTransform(original, "fixture", true));
  agt_field_test_bringup::PlanarControlBuffer control(clock, raw, "base_link");
  const auto before = raw->lookupTransform("map", "base_link", tf2::TimePointZero);
  const auto delegated = control.lookupTransform("map", "base_link", tf2::TimePointZero,
    tf2::Duration::zero());
  (void)control.transform(goal(), "base_link");
  const auto after = raw->lookupTransform("map", "base_link", tf2::TimePointZero);
  EXPECT_EQ(before, delegated);
  EXPECT_EQ(before, after);
  EXPECT_EQ(after.transform.translation.z, -.8);
  EXPECT_NE(after.transform.rotation.y, 0.);
}

TEST(PlanarControlBuffer, DelegatesAvailabilityAndMissingTransformErrors)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto raw = std::make_shared<tf2_ros::Buffer>(clock);
  raw->setUsingDedicatedThread(true);
  agt_field_test_bringup::PlanarControlBuffer control(clock, raw, "base_link");
  EXPECT_FALSE(control.canTransform("base_link", "missing", tf2::TimePointZero,
    tf2::Duration::zero()));
  EXPECT_THROW(control.lookupTransform("base_link", "missing", tf2::TimePointZero,
    tf2::Duration::zero()), tf2::TransformException);
}
