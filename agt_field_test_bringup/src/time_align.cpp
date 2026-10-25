#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace {
std::int64_t offset_ns(double seconds)
{
  const long double value = static_cast<long double>(seconds) * 1000000000.0L;
  if (!std::isfinite(seconds) || value > std::numeric_limits<std::int64_t>::max() ||
      value < std::numeric_limits<std::int64_t>::min()) {
    throw std::invalid_argument("timestamp offset must be finite and fit int64 nanoseconds");
  }
  return static_cast<std::int64_t>(value);
}

std::uint64_t shifted(std::uint64_t value, std::int64_t offset)
{
  if (offset >= 0) {
    const auto delta = static_cast<std::uint64_t>(offset);
    if (value > std::numeric_limits<std::uint64_t>::max() - delta) {
      throw std::overflow_error("timestamp offset overflow");
    }
    return value + delta;
  }
  const auto magnitude = static_cast<std::uint64_t>(-(offset + 1)) + 1U;
  if (value < magnitude) throw std::underflow_error("timestamp offset underflow");
  return value - magnitude;
}

void shift_stamp(builtin_interfaces::msg::Time &stamp, std::int64_t offset)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    throw std::invalid_argument("input timestamp is invalid or negative");
  }
  const auto input = static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
  const auto output = shifted(input, offset);
  if (output / 1000000000ULL > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::overflow_error("shifted ROS timestamp seconds overflow");
  }
  stamp.sec = static_cast<std::int32_t>(output / 1000000000ULL);
  stamp.nanosec = static_cast<std::uint32_t>(output % 1000000000ULL);
}

class TimeAlign : public rclcpp::Node {
public:
  TimeAlign() : Node("field_time_align")
  {
    imu_offset_ = offset_ns(declare_parameter<double>("imu_offset_sec", 0.0));
    lidar_offset_ = offset_ns(declare_parameter<double>("lidar_offset_sec", 0.0));
    const auto imu_input = declare_parameter<std::string>("imu_input", "/livox/imu");
    const auto lidar_input = declare_parameter<std::string>("lidar_input", "/livox/lidar");
    const bool align_imu = declare_parameter<bool>("align_imu", true);
    const bool align_lidar = declare_parameter<bool>("align_lidar", true);
    // Keep both streams native: Python conversion of a 20k-point scan must
    // never starve the 200 Hz IMU. No point or per-point time fields change.
    if (align_imu) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/field/aligned/imu", 400);
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_input,
      rclcpp::SensorDataQoS().keep_last(400),
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        try {
          shift_stamp(msg->header.stamp, imu_offset_);
          imu_pub_->publish(std::move(msg));
        } catch (const std::exception &error) {
          RCLCPP_ERROR(get_logger(), "Rejected IMU timestamp: %s", error.what());
        }
      });
    }
    if (align_lidar) {
      lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/field/aligned/lidar", 20);
      lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_input,
      rclcpp::SensorDataQoS().keep_last(20),
      [this](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
        try {
          shift_stamp(msg->header.stamp, lidar_offset_);
          msg->timebase = shifted(msg->timebase, lidar_offset_);
          lidar_pub_->publish(std::move(msg));
        } catch (const std::exception &error) {
          RCLCPP_ERROR(get_logger(), "Rejected LiDAR timestamp: %s", error.what());
        }
      });
    }
    RCLCPP_INFO(get_logger(),
      "Native time alignment; IMU enabled=%d offset_ns=%ld LiDAR enabled=%d offset_ns=%ld",
      align_imu, static_cast<long>(imu_offset_), align_lidar, static_cast<long>(lidar_offset_));
  }
private:
  std::int64_t imu_offset_{0}, lidar_offset_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeAlign>());
  rclcpp::shutdown();
  return 0;
}
