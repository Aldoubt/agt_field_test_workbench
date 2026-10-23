#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <thread>

// Humble rclcpp exposes rmw publisher GIDs; its Python message callbacks do not.
class Guard : public rclcpp::Node {
 public:
  Guard() : Node("fwmini_tf_contract") {
    children_ = declare_parameter<std::vector<std::string>>("children");
    parents_ = declare_parameter<std::vector<std::string>>("parents");
    xyz_ = declare_parameter<std::vector<double>>("expected_xyz");
    q_ = declare_parameter<std::vector<double>>("expected_quaternions");
    hash_ = declare_parameter<std::string>("calibration_sha256", "");
    report_ = declare_parameter<std::string>("report_path", "");
    preflight_ = declare_parameter("preflight", false);
    timeout_ = declare_parameter("timeout_sec", 10.0);
    if (children_.size() != 4 || parents_.size() != 4 || xyz_.size() != 12 || q_.size() != 16)
      throw std::runtime_error("invalid expected TF contract");
    static_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
        rclcpp::QoS(100).reliable().transient_local(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m, const rclcpp::MessageInfo &i) { receive(*m, i, true); });
    dynamic_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m, const rclcpp::MessageInfo &i) { receive(*m, i, false); });
  }

  void receive(const tf2_msgs::msg::TFMessage &message, const rclcpp::MessageInfo &info, bool is_static) {
    std::ostringstream gid;
    for (auto byte : info.get_rmw_message_info().publisher_gid.data)
      gid << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    for (const auto &t : message.transforms) {
      auto it = std::find(children_.begin(), children_.end(), t.child_frame_id);
      if (it == children_.end() && t.child_frame_id != "base_footprint" && t.child_frame_id != "odom") continue;
      auto &owners = owners_[t.child_frame_id];
      owners.insert(t.header.frame_id + ":" + gid.str());
      if (owners.size() > 1) error_ = "duplicate TF publisher/parent for " + t.child_frame_id;
      if (it == children_.end()) continue;
      const auto index = static_cast<size_t>(it-children_.begin());
      const auto &p = t.transform.translation;
      const auto &q = t.transform.rotation;
      double values[] = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
      bool matches = is_static && t.header.frame_id == parents_[index];
      for (size_t j=0; j<7; ++j) matches = matches && std::isfinite(values[j]);
      for (size_t j=0; j<3; ++j) matches = matches && std::abs(values[j]-xyz_[index*3+j]) < 1e-6;
      double positive=0, negative=0;
      for (size_t j=0; j<4; ++j) {
        positive = std::max(positive, std::abs(values[3+j]-q_[index*4+j]));
        negative = std::max(negative, std::abs(values[3+j]+q_[index*4+j]));
      }
      if (!matches || std::min(positive, negative) > 1e-6)
        error_ = error_.empty() ? "TF does not match selected calibration: " + t.child_frame_id : error_;
      seen_.insert(t.child_frame_id);
    }
  }

  int run() {
    auto start = std::chrono::steady_clock::now(), ready = start;
    bool complete = false;
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      if (!error_.empty()) { RCLCPP_ERROR(get_logger(), "%s", error_.c_str()); return 2; }
      auto now = std::chrono::steady_clock::now();
      if (seen_.size() == children_.size()) {
        if (!complete) { ready=now; complete=true; }
        if (preflight_ && std::chrono::duration<double>(now-ready).count() >= 1.0) {
          if (!report_.empty()) {
            std::ofstream out(report_);
            if (!out) throw std::runtime_error("cannot write TF report");
            out << "{\"status\":\"SOFTWARE_TF_PASS\",\"physical_calibration_verified\":false,\"calibration_sha256\":\""
                << hash_ << "\",\"owners\":{";
            bool first=true;
            for (const auto &entry : owners_) {
              if (!first) out << ",";
              first=false;
              out << "\"" << entry.first << "\":[";
              bool first_owner=true;
              for (const auto &owner : entry.second) {
                if (!first_owner) out << ",";
                first_owner=false; out << "\"" << owner << "\"";
              }
              out << "]";
            }
            out << "}}\n";
          }
          RCLCPP_INFO(get_logger(), "FWMINI_TF_READY: selected mount and unique TF owners observed");
          return 0;
        }
      } else if (std::chrono::duration<double>(now-start).count() > timeout_) {
        RCLCPP_ERROR(get_logger(), "missing mount TF: observed %zu / %zu required edges", seen_.size(), children_.size());
        return 3;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
  }
 private:
  std::vector<std::string> children_, parents_;
  std::vector<double> xyz_, q_;
  std::string hash_, report_, error_;
  bool preflight_;
  double timeout_;
  std::set<std::string> seen_;
  std::map<std::string, std::set<std::string>> owners_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_, dynamic_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result=1;
  try { result=std::make_shared<Guard>()->run(); }
  catch (const std::exception &error) { std::cerr << error.what() << std::endl; }
  rclcpp::shutdown();
  return result;
}
