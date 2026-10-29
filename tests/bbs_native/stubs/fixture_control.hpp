#pragma once
// Deterministic injected registration outcomes for orchestration regression.
// This fixture deliberately does not model BBS geometry or GICP optimization.
#include <cstdlib>
#include <fstream>
#include <string>

namespace fixture {
inline std::string scenario() {
  const char* value = std::getenv("AGT_TEST_SCENARIO");
  return value ? value : "first_success";
}
inline void call(const char* stage, double candidate_x) {
  if (const char* path = std::getenv("AGT_TEST_CALLS")) {
    std::ofstream(path, std::ios::app) << stage << ' ' << candidate_x << '\n';
  }
}
}  // namespace fixture
