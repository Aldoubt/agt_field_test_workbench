#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <thread>
#include <vector>
#include <fixture_control.hpp>

namespace small_gicp {
struct RegistrationSetting {
  int num_threads{1};
  double downsampling_resolution{0.35};
  double max_correspondence_distance{2.0};
};
struct RegistrationResult {
  bool converged{true};
  std::size_t num_inliers{0};
  double error{0.0};
  Eigen::Isometry3d T_target_source{Eigen::Isometry3d::Identity()};
};
inline RegistrationResult align(const std::vector<Eigen::Vector3d>& target,
                                const std::vector<Eigen::Vector3d>& source,
                                const Eigen::Isometry3d& seed,
                                const RegistrationSetting&) {
  if (target.empty() || source.size() < 300) throw std::runtime_error("bad test cloud");
  fixture::call("gicp", seed.translation().x());
  const auto scenario = fixture::scenario();
  const bool first = seed.translation().x() < 5.0;
  RegistrationResult out;
  out.T_target_source = seed;
  out.num_inliers = source.size() * 4 / 5;
  out.converged = scenario != "all_nonconverged" && !(first && scenario == "first_nonconverged");
  if (first && scenario == "first_zero_inliers") out.num_inliers = 0;
  if (first && scenario == "first_low_overlap") out.num_inliers = source.size() / 10;
  if (first && scenario == "first_nan_pose") out.T_target_source.translation().x() = NAN;
  out.error = out.num_inliers * ((first && scenario == "first_bad_fitness") ? 2.0 : 0.1);
  if (scenario == "refine_over_budget") std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  return out;
}
}  // namespace small_gicp
