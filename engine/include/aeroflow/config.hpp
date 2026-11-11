#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "aeroflow/box.hpp"

namespace aeroflow {

// Raised for any malformed or out-of-range configuration. The message names the
// offending field (e.g. "resolution.nx: must be an integer >= 1").
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct GridConfig {
  std::size_t nx{0};
  std::size_t ny{0};
  std::size_t nz{0};
  double lx{0.0};
  double ly{0.0};
  double lz{0.0};
  std::optional<Aabb> test_box;
};

// Expected shape:
// {
//   "domain":     {"lx": 4.0, "ly": 2.0, "lz": 2.0},
//   "resolution": {"nx": 64,  "ny": 32,  "nz": 32},
//   "test_box":   {"min": [x, y, z], "max": [x, y, z]}   // optional
// }
// Unknown keys are rejected so typos do not pass silently.
GridConfig parse_config(const nlohmann::json& root);

// Parses JSON text first; syntax errors are reported as ConfigError.
GridConfig parse_config_text(const std::string& text);

}  // namespace aeroflow
