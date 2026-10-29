#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <fixture_control.hpp>

namespace cpu {
class BBS3D {
 public:
  void set_num_threads(int) {}
  bool set_voxelmaps_coords(const std::string&) { return true; }
  void set_src_points(const std::vector<Eigen::Vector3d>&) {}
  void set_trans_search_range(const Eigen::Vector3d& low, const Eigen::Vector3d& high) {
    pose_.translation() = (low + high) / 2.0;
  }
  void set_angular_search_range(const Eigen::Vector3d&, const Eigen::Vector3d&) {}
  void set_score_threshold_percentage(double) {}
  void enable_timeout() {}
  void set_timeout_duration_in_msec(int) {}
  void localize() { fixture::call("bbs", pose_.translation().x()); }
  bool has_localized() const { return fixture::scenario() != "no_bbs_pose"; }
  bool has_timed_out() const { return fixture::scenario() == "bbs_timeout"; }
  double get_best_score_percentage() const {
    // The failing first candidate deliberately looks better to coarse search.
    return pose_.translation().x() < 5.0 ? 0.9 : 0.8;
  }
  Eigen::Matrix4d get_global_pose() const { return pose_.matrix(); }
  double get_elapsed_time() const { return 1.0; }
 private:
  Eigen::Isometry3d pose_{Eigen::Isometry3d::Identity()};
};
}  // namespace cpu
