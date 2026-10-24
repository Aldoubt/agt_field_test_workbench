// Preserve original Livox CustomMsg point offsets for FAST-LIO2 deskew.
// Shares the same agt_pointcloud_core RangeFilter/BoxFilter definitions as
// agt_pointcloud_pipeline; only the message transport differs.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include "agt_pointcloud_core/filters.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LivoxSelfFilter final : public rclcpp::Node {
public:
  LivoxSelfFilter()
  : Node("field_livox_self_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    using agt_pointcloud_core::BoxFilter;
    const auto input=declare_parameter<std::string>("input_topic","/field/aligned/lidar");
    const auto output=declare_parameter<std::string>("output_topic","/field/self_filtered/lidar");
    const auto status=declare_parameter<std::string>("status_topic","/mapping/self_filter/status");
    target_frame_=declare_parameter<std::string>("target_frame","base_link");
    const auto near=declare_parameter<double>("range_min_m",0.5);
    const auto far=declare_parameter<double>("range_max_m",120.0);
    box_=std::make_unique<BoxFilter>(
      declare_parameter<double>("x_min",-0.56),declare_parameter<double>("x_max",0.56),
      declare_parameter<double>("y_min",-0.44),declare_parameter<double>("y_max",0.44),
      declare_parameter<double>("z_min",-0.10),declare_parameter<double>("z_max",0.45));
    max_fraction_=declare_parameter<double>("max_removed_fraction",0.30);
    max_end_trim_ns_=declare_parameter<int>("max_end_time_trim_ns",1000000);
    min_remaining_=declare_parameter<int>("min_remaining_points",1000);
    tf_timeout_=declare_parameter<double>("tf_timeout_sec",0.03);
    if(!box_->valid() || !std::isfinite(near) || near<0 || !std::isfinite(far) || far<=near
      || !std::isfinite(max_fraction_) || max_fraction_<=0 || max_fraction_>=1
      || min_remaining_<1 || max_end_trim_ns_<0
      || !std::isfinite(tf_timeout_) || tf_timeout_<0) {
      throw std::runtime_error("invalid self-filter limits");
    }
    range_=std::make_unique<agt_pointcloud_core::RangeFilter>(near,far);
    publisher_=create_publisher<livox_ros_driver2::msg::CustomMsg>(output,rclcpp::QoS(20));
    status_pub_=create_publisher<std_msgs::msg::String>(status,rclcpp::QoS(5));
    subscription_=create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input,rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {handle(msg);});
    RCLCPP_INFO(get_logger(),"Timing-preserving self-filter %s -> %s (frame=%s)",
      input.c_str(),output.c_str(),target_frame_.c_str());
  }
private:
  void report(const std::string &state,std::size_t in,std::size_t out) {
    ++scans_;
    if(state=="ACTIVE") ++active_;
    else if(state=="BYPASS_TF") ++tf_fail_;
    else ++guard_bypass_;
    if(scans_%10!=1 && state==last_state_) return;
    last_state_=state;
    std::ostringstream o;
    o<<"{\"state\":\""<<state<<"\",\"scans\":"<<scans_
     <<",\"active_scans\":"<<active_<<",\"tf_bypass_scans\":"<<tf_fail_
     <<",\"guard_bypass_scans\":"<<guard_bypass_
     <<",\"input_points\":"<<in<<",\"output_points\":"<<out
     <<",\"removed_points_total\":"<<removed_<<"}";
    std_msgs::msg::String report;report.data=o.str();status_pub_->publish(report);
    if(state!="ACTIVE") {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
        "Livox self filter %s (input=%zu output=%zu, TF bypass=%zu guard bypass=%zu)",
        state.c_str(),in,out,tf_fail_,guard_bypass_);
    }
  }
  void handle(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
    if(msg->point_num!=msg->points.size() || msg->header.frame_id.empty()) {
      // Do not conceal upstream packet corruption from the existing adapter.
      publisher_->publish(*msg);
      report("BYPASS_INVALID_PACKET",msg->points.size(),msg->points.size());return;
    }
    tf2::Transform transform;
    try {
      if(msg->header.frame_id==target_frame_) transform.setIdentity();
      else {
        auto tf=tf_buffer_.lookupTransform(target_frame_,msg->header.frame_id,
          rclcpp::Time(msg->header.stamp),rclcpp::Duration::from_seconds(tf_timeout_));
        const auto &t=tf.transform.translation;const auto &r=tf.transform.rotation;
        tf2::Quaternion q(r.x,r.y,r.z,r.w);
        if(!std::isfinite(q.length2()) || q.length2()<0.5) throw std::runtime_error("invalid TF rotation");
        q.normalize();transform=tf2::Transform(q,tf2::Vector3(t.x,t.y,t.z));
      }
    } catch(const std::exception &ex) {
      publisher_->publish(*msg);report("BYPASS_TF",msg->points.size(),msg->points.size());return;
    }
    livox_ros_driver2::msg::CustomMsg output=*msg;
    output.points.clear();output.points.reserve(msg->points.size());
    for(const auto &p:msg->points) {
      const auto transformed=transform*tf2::Vector3(p.x,p.y,p.z);
      agt_pointcloud_core::PointContext context{
        {p.x,p.y,p.z},{transformed.x(),transformed.y(),transformed.z()}};
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)
        ||range_->removes(context)||box_->removes(context)) continue;
      output.points.push_back(p); // preserves offset_time, tag, reflectivity and order
    }
    const std::size_t n=msg->points.size(), kept=output.points.size();
    // FAST-LIO2 derives cloud_end_time from the largest surviving offset_time.
    // A large trim changes its IMU integration window. Bypass the entire scan
    // rather than claiming filtering succeeded with altered timing semantics.
    std::uint32_t old_end=0,new_end=0;
    for(const auto &p:msg->points) old_end=std::max(old_end,p.offset_time);
    for(const auto &p:output.points) new_end=std::max(new_end,p.offset_time);
    if(old_end>new_end &&
       old_end-new_end>static_cast<std::uint32_t>(max_end_trim_ns_)) {
      publisher_->publish(*msg);report("BYPASS_TIME_BOUND",n,n);return;
    }
    if(n==0 || kept<static_cast<std::size_t>(min_remaining_)
       || double(n-kept)/double(n)>max_fraction_) {
      publisher_->publish(*msg);report("BYPASS_GUARD",n,n);return;
    }
    output.point_num=static_cast<std::uint32_t>(kept);
    removed_+=n-kept;
    publisher_->publish(output);
    report("ACTIVE",n,kept);
  }
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_,last_state_;
  std::unique_ptr<agt_pointcloud_core::BoxFilter> box_;
  std::unique_ptr<agt_pointcloud_core::RangeFilter> range_;
  double max_fraction_{0.30},tf_timeout_{0.03};
  int min_remaining_{1000},max_end_trim_ns_{1000000};
  std::size_t scans_{0},active_{0},tf_fail_{0},guard_bypass_{0},removed_{0};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<LivoxSelfFilter>());}
  catch(const std::exception &ex) {std::cerr<<ex.what()<<std::endl;return 1;}
  rclcpp::shutdown();return 0;
}
