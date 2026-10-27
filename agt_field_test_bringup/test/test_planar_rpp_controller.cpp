#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
class Checker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & p, geometry_msgs::msg::Twist &) override
  {p.position.x = .02; return true;}
};

class PlanarRpp : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.desired_linear_vel", .03),
      rclcpp::Parameter("FollowPath.min_approach_linear_velocity", .03),
      rclcpp::Parameter("FollowPath.regulated_linear_scaling_min_speed", .03),
      rclcpp::Parameter("FollowPath.approach_velocity_scaling_dist", .8),
      rclcpp::Parameter("FollowPath.use_rotate_to_heading", false),
      rclcpp::Parameter("FollowPath.allow_reversing", true),
      rclcpp::Parameter("FollowPath.use_collision_detection", true),
    });
    parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planar_rpp_fixture", options);
    parent->declare_parameter("controller_frequency", 50.0);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("planar_test_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
    costmap->set_parameter(rclcpp::Parameter("robot_base_frame", "base_link"));
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    costmap->getCostmap()->resizeMap(200, 200, .05, -5., -5.);
    std::fill(costmap->getCostmap()->getCharMap(),
      costmap->getCostmap()->getCharMap() + 40000, nav2_costmap_2d::FREE_SPACE);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = .6; footprint[0].y = .475;
    footprint[1].x = .6; footprint[1].y = -.475;
    footprint[2].x = -.6; footprint[2].y = -.475;
    footprint[3].x = -.6; footprint[3].y = .475;
    costmap->setRobotFootprint(footprint);
    tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    tf->setUsingDedicatedThread(true);
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
  }

  void configure(bool planar = true)
  {
    if (planar) {
      controller = loader->createSharedInstance(
        "agt_field_test_bringup/PlanarRegulatedPurePursuitController");
    } else {
      controller = std::make_shared<
        nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
    }
    controller->configure(parent, "FollowPath", tf, costmap);
    controller->activate();
  }

  geometry_msgs::msg::PoseStamped place(double z = -.8074249785,
    double roll = -.0053438, double pitch = .049716)  // Recorded near-goal attitude.
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map"; t.header.stamp.sec = 1; t.child_frame_id = "base_link";
    t.transform.translation.z = z;
    tf2::Quaternion q; q.setRPY(roll, pitch, 0.);
    t.transform.rotation = tf2::toMsg(q);
    EXPECT_TRUE(tf->setTransform(t, "fixture", true));
    geometry_msgs::msg::PoseStamped pose;
    pose.header = t.header; pose.pose.position.z = z; pose.pose.orientation = t.transform.rotation;
    return pose;
  }

  void plan(double x = .08236906049, double y = .00209071800)
  {
    nav_msgs::msg::Path path; path.header.frame_id = "map";
    path.header.stamp.sec = 1;
    geometry_msgs::msg::PoseStamped p; p.header = path.header; p.pose.orientation.w = 1.;
    p.pose.position.x = -.1; path.poses.push_back(p);
    p.pose.position.x = x; p.pose.position.y = y; path.poses.push_back(p);
    controller->setPlan(path);
  }

  auto command(const geometry_msgs::msg::PoseStamped & p)
  {return controller->computeVelocityCommands(p, geometry_msgs::msg::Twist(), &checker).twist;}

  void TearDown() override
  {
    if (controller) {controller->deactivate(); controller->cleanup(); controller.reset();}
    loader.reset(); tf.reset();
    if (costmap) {costmap->on_cleanup(rclcpp_lifecycle::State()); costmap.reset();}
    parent.reset();
  }

  Checker checker;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader;
  std::shared_ptr<nav2_core::Controller> controller;
};
}

TEST_F(PlanarRpp, NativeControllerReproducesWrongSide)
{
  const auto pose = place(); configure(false); plan();
  EXPECT_LT(command(pose).angular.z, 0.);
}

TEST_F(PlanarRpp, InstalledPluginUsesPlanarSteeringAndKeepsPruning)
{
  const auto pose = place(); configure(); plan();
  for (int i = 0; i < 3; ++i) {
    const auto cmd = command(pose);
    EXPECT_NEAR(cmd.linear.x, .03, 1e-12);
    EXPECT_NEAR(cmd.angular.z, .018477311245, 1e-9);
  }
}

TEST_F(PlanarRpp, SamePlanarGoalGivesSameCommandForDifferentHeightsAndTilt)
{
  configure();
  for (double z : {-1.5, 0., 1.2}) {
    const auto pose = place(z, -.12, .25); plan();
    const auto cmd = command(pose);
    EXPECT_NEAR(cmd.linear.x, .03, 1e-12);
    EXPECT_NEAR(cmd.angular.z, .018477311245, 1e-9);
  }
}

TEST_F(PlanarRpp, FootprintCollisionStillStopsCommands)
{
  const auto pose = place(); configure(); plan();
  auto * map = costmap->getCostmap();
  for (unsigned int x = 110; x <= 114; ++x) {
    for (unsigned int y = 86; y <= 114; ++y) {map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
  }
  EXPECT_THROW(command(pose), nav2_core::PlannerException);
}

TEST_F(PlanarRpp, NativeSpeedLimitRemainsEffective)
{
  const auto pose = place(); configure(); plan();
  controller->setSpeedLimit(.01, false);
  EXPECT_LE(std::abs(command(pose).linear.x), .01 + 1e-12);
}

TEST_F(PlanarRpp, ReverseGoalStillProducesReverseMotion)
{
  const auto pose = place(); configure(); plan(-.082, .002);
  EXPECT_LT(command(pose).linear.x, 0.);
}

TEST_F(PlanarRpp, MissingTransformProducesNoCommand)
{
  configure(); plan();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map"; pose.header.stamp.sec = 1; pose.pose.orientation.w = 1.;
  EXPECT_THROW(command(pose), nav2_core::PlannerException);
}

TEST_F(PlanarRpp, ProjectedFootprintCollisionStillStopsBeforeReachingObstacle)
{
  const auto pose = place(); configure(); plan(2., 0.);
  ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
    "FollowPath.desired_linear_vel", .4)).successful);
  auto * map = costmap->getCostmap();
  for (unsigned int x = 117; x <= 120; ++x) {
    for (unsigned int y = 86; y <= 114; ++y) {map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
  }
  EXPECT_THROW(command(pose), nav2_core::PlannerException);
}

TEST_F(PlanarRpp, NearGoalConvergesInPlanarSimulationWithoutDirectionSwitches)
{
  configure();
  for (double z : {-1.5, 0., 1.2}) {
    auto pose = place(z, -.0053438, .049716); plan();
    double yaw = 0.;
    int steps = 0;
    for (; steps < 200; ++steps) {
      if (std::hypot(pose.pose.position.x-.08236906049,
        pose.pose.position.y-.00209071800) <= .02) {break;}
      const auto cmd = command(pose);
      ASSERT_GT(cmd.linear.x, 0.);
      const double dt = .02;
      pose.pose.position.x += cmd.linear.x * std::cos(yaw) * dt;
      pose.pose.position.y += cmd.linear.x * std::sin(yaw) * dt;
      yaw += cmd.angular.z * dt;
      tf2::Quaternion q; q.setRPY(-.0053438, .049716, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      geometry_msgs::msg::TransformStamped t;
      t.header = pose.header; t.child_frame_id = "base_link";
      t.transform.translation.x = pose.pose.position.x;
      t.transform.translation.y = pose.pose.position.y;
      t.transform.translation.z = z; t.transform.rotation = pose.pose.orientation;
      ASSERT_TRUE(tf->setTransform(t, "simulation", true));
    }
    EXPECT_LT(steps, 200);
    EXPECT_LE(std::hypot(pose.pose.position.x-.08236906049,
      pose.pose.position.y-.00209071800), .02);
  }
}
